// Test-only replay of cached pixels; truth is never loaded by this worker.
#include "tote_fusion/core.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <numeric>
#include <sstream>
#include <iomanip>
#include <tuple>
#include <type_traits>
using namespace tote_fusion;
using Clock = std::chrono::steady_clock;
namespace fs = std::filesystem;

namespace {
template<class Derived> YAML::Node matrix(const Eigen::MatrixBase<Derived> &value) {
  YAML::Node node(YAML::NodeType::Sequence);
  for (Eigen::Index i=0; i<value.rows(); ++i) {
    YAML::Node row(YAML::NodeType::Sequence);
    for (Eigen::Index j=0; j<value.cols(); ++j) row.push_back(value(i,j));
    node.push_back(row);
  }
  return node;
}
Pose pose(const YAML::Node &node) {
  require(node.size()==4,"INVALID_CACHE","Expected a 4x4 transform");
  Pose value=Pose::Identity();
  for (int i=0;i<4;++i) { require(node[i].size()==4,"INVALID_CACHE","Transform row size");
    for(int j=0;j<4;++j)value.matrix()(i,j)=node[i][j].as<double>(); }
  require(value.matrix().allFinite() && (value.linear().transpose()*value.linear()).isApprox(Eigen::Matrix3d::Identity(),1e-9) &&
    std::abs(value.linear().determinant()-1)<1e-9 && value.matrix().row(3).isApprox(Eigen::RowVector4d(0,0,0,1)),
    "INVALID_CACHE","Invalid rigid transform");
  return value;
}
std::vector<uint8_t> bytes(const fs::path &path,std::size_t size) {
  require(size>0 && size<=16*1024*1024,"INVALID_CACHE","Unbounded image");
  std::ifstream stream(path,std::ios::binary|std::ios::ate);
  require(stream.good() && stream.tellg()==static_cast<std::streamoff>(size),"INVALID_CACHE","Wrong image byte count: "+path.string());
  std::vector<uint8_t> data(size);stream.seekg(0);stream.read(reinterpret_cast<char*>(data.data()),size);
  require(stream.good(),"INVALID_CACHE","Incomplete image read");return data;
}
// Allocate proportionally across observed strata, not truth surface labels.
template<class T,class Key> std::vector<T> thin(const std::vector<T> &input,std::size_t target,Key key) {
  if(input.size()<=target)return input;
  std::map<decltype(key(input.front())),std::vector<T>> groups;
  for(const auto &item:input)groups[key(item)].push_back(item);
  std::vector<T> result;std::size_t assigned=0,seen=0;
  for(const auto &[id,items]:groups){
    (void)id;seen+=items.size();const auto end=target*seen/input.size(),quota=end-assigned;assigned=end;
    for(std::size_t i=0;i<quota;++i)result.push_back(items[(2*i+1)*items.size()/(2*quota)]);
  }
  assert(result.size()==target);return result;
}
template<class T> std::vector<T> subset(const std::vector<T> &input,int variant,bool balanced) {
  std::vector<T> camera[2];for(const auto &p:input)camera[p.camera].push_back(p);
  std::size_t target=0;
  for(const auto &points:camera)if(!points.empty())target=target==0 ? points.size() : std::min(target,points.size());
  if constexpr(std::is_same_v<T,Observation>) target=std::min<std::size_t>(target,300);
  std::vector<T> result;
  for(uint32_t c=0;c<2;++c)if(variant==2 || variant==static_cast<int>(c)){
    const std::size_t quota=variant==2 ? target/2+(c==0 ? target%2 : 0) : target;
    auto values=balanced ? thin(camera[c],quota,[](const T &p){
      if constexpr(std::is_same_v<T,Observation>)return std::make_tuple(p.frame,0u,0,0);
      else return std::make_tuple(p.frame,p.axis,p.side,p.wall_side);
    }) : camera[c];
    result.insert(result.end(),values.begin(),values.end());
  }
  return result;
}
}

int main(int argc,char **argv) {
  require(argc==4,"INVALID_CONFIG","worker INPUT_DIRECTORY MODEL_FILE OUTPUT_YAML");
  const uint16_t endian=1;require(*reinterpret_cast<const uint8_t*>(&endian)==1,"INVALID_CONFIG","Little-endian host required");
  const fs::path directory=argv[1];const auto scene=YAML::LoadFile((directory/"inputs.json").string());
  require(scene["schema"].as<int>()==1,"INVALID_CACHE","Unsupported cache schema");
  auto model=load_model(argv[2]);model.prior=pose(scene["prior"]);
  auto config=FitConfig{};config.allow_pi_symmetry=true;config.estimate_geometry=scene["mode"].as<int>()==1;
  config.camera_noise_sigma_m=scene["configured_sigmas"].as<std::vector<double>>();
  require(config.camera_noise_sigma_m.size()==2,"INVALID_CACHE","Need stable two-camera configuration");
  std::vector<Observation> surfaces[2];std::vector<BoundaryPoint> boundaries[2];
  double extraction_camera[2]{};
  for(const auto &frame:scene["frames"]){
    const auto camera=frame["camera"].as<uint32_t>(),id=frame["frame"].as<uint32_t>();
    require(camera<2 && id<6,"INVALID_CACHE","Invalid camera/frame ID");
    const auto extraction_started=Clock::now();
    const auto width=frame["width"].as<uint32_t>(),height=frame["height"].as<uint32_t>();
    const auto key=frame["key"].as<std::string>();
    const auto depth=bytes(directory/(key+"_depth.bin"),std::size_t(width)*height*4),mask=bytes(directory/(key+"_mask.bin"),std::size_t(width)*height);
    const ImageView d{depth.data(),depth.size(),width*4,width,height,"32FC1",false};
    const ImageView m{mask.data(),mask.size(),width,width,height,"mono8",false};
    const Intrinsics k{frame["fx"].as<double>(),frame["fy"].as<double>(),frame["cx"].as<double>(),frame["cy"].as<double>()};
    const Pose transform=pose(frame["body_camera"]);const int split=id%2;
    for(const auto &point:transform_points(project(d,m,k,.3,3.,4,1000000),transform))
      surfaces[split].push_back({point,transform.translation(),id,camera});
    if(config.estimate_geometry){
      auto edges=image_boundaries(d,m,k,transform,model,id,camera,config.camera_noise_sigma_m[camera],20000);
      boundaries[split].insert(boundaries[split].end(),edges.begin(),edges.end());
    }
    extraction_camera[camera]+=std::chrono::duration<double>(Clock::now()-extraction_started).count();
  }
  YAML::Node rows(YAML::NodeType::Sequence);
  for(bool balanced:{false,true})for(int variant=0;variant<3;++variant)for(bool robust:{false,true}){
    const auto selection_started=Clock::now();
    auto fitting=subset(surfaces[0],variant,balanced),held=subset(surfaces[1],variant,balanced);
    auto edges=subset(boundaries[0],variant,balanced),held_edges=subset(boundaries[1],variant,balanced);
    YAML::Node row;row["scene_id"]=scene["scene_id"];row["cache_digest"]=scene["cache_digest"];
    row["stage"]="pixel_core";row["subset"]=variant==0 ? "A" : variant==1 ? "B" : "AB";
    row["budget"]=balanced ? "equal_samples" : "equal_time";row["loss"]=robust ? "huber" : "squared";
#ifdef TOTE_EQUAL_PRECISION_WORKER
    row["precision"]="equal";
#else
    row["precision"]="declared";
#endif
    std::ostringstream identity;identity<<std::setprecision(17);
    for(const auto *set:{&fitting,&held})for(const auto &p:*set)
      identity<<p.camera<<','<<p.frame<<','<<p.point.transpose()<<','<<p.origin.transpose()<<';';
    for(const auto *set:{&edges,&held_edges})for(const auto &p:*set)
      identity<<p.camera<<','<<p.frame<<','<<p.axis<<','<<p.side<<','<<p.wall_side<<','<<p.sigma<<','<<p.point.transpose()<<','<<p.wall_normal.transpose()<<';';
    row["selected_data_sha256"]=sha256(identity.str());
    std::map<std::pair<uint32_t,uint32_t>,uint32_t> surface_counts;
    std::map<uint32_t,std::size_t> frames_per_camera;
    for(const auto &p:fitting)++surface_counts[{p.camera,p.frame}];
    for(const auto &[key,count]:surface_counts){(void)count;++frames_per_camera[key.first];}
    double surface_cap=0;
    for(const auto &[key,count]:surface_counts)surface_cap+=double(std::min(count,config.effective_points_per_frame))/frames_per_camera.at(key.first);
    std::map<std::tuple<uint32_t,uint32_t,int>,uint32_t> edge_counts;
    for(const auto &p:edges)++edge_counts[{p.camera,p.axis,p.side}];
    uint32_t edge_cap=0;for(const auto &[key,count]:edge_counts){(void)key;edge_cap+=std::min(count,12u);}
    row["surface_effective_cap_before_correspondence"]=surface_cap;
    row["boundary_effective_cap"]=edge_cap;

    row["fitting_points"]=fitting.size();row["held_points"]=held.size();
    row["fitting_boundaries"]=edges.size();row["held_boundaries"]=held_edges.size();
    const double extraction=variant==2 ? extraction_camera[0]+extraction_camera[1] : extraction_camera[variant];
    row["selection_s"]=std::chrono::duration<double>(Clock::now()-selection_started).count();
    row["extraction_s"]=extraction;config.huber_sigma=robust ? 2. : 1e12;
    const auto fit_started=Clock::now();
    try{
      const auto result=estimate(model,fitting,held,config,[]{return false;},edges,held_edges);
      row["accepted"]=true;row["code"]="OK";row["pose"]=matrix(result.pose.matrix());
      row["covariance"]=matrix(result.joint_covariance);row["dimension_axes"]=result.dimension_axes;
      row["dimensions"]=matrix(result.dimensions);row["iterations"]=result.iterations;
      row["held_rmse_m"]=result.held_out_rmse_m;
    }catch(const Rejection &error){row["accepted"]=false;row["code"]=error.code;row["message"]=error.what();}
    row["fit_s"]=std::chrono::duration<double>(Clock::now()-fit_started).count();
    row["compute_s"]=row["fit_s"].as<double>()+extraction+row["selection_s"].as<double>();row["age_s"]=YAML::Null;
    rows.push_back(row);
  }
  std::ofstream output(argv[3]);require(output.good(),"INVALID_CONFIG","Cannot create result output");
  output<<rows<<'\n';require(output.good(),"INVALID_CONFIG","Result output write failed");
}
