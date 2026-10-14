#include "tote_fusion/core.hpp"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <map>
#include <tuple>

namespace tote_fusion {
Model parametric_model(const Model &nominal, const Vec3 &dimensions) {
  require(nominal.id == "EU6423" && nominal.boxes.size() == 17,"INVALID_MODEL","EU6423 v1 requires its 17 nominal boxes");
  require(dimensions.allFinite() && dimensions.z() == nominal.dimensions.z() &&
    (dimensions.head<2>()-nominal.dimensions.head<2>()).cwiseAbs().maxCoeff() <= .06 &&
    (dimensions.array() > 0).all(), "INVALID_GEOMETRY", "EU6423 dimensions outside the declared 60 mm basin or Z changed");
  auto model = nominal; model.dimensions = dimensions;
  for (auto &box : model.boxes) for (int a=0; a<2; ++a) {
    const double delta = (dimensions[a]-nominal.dimensions[a])/2;
    const double low = box.low[a], high = box.high[a];
    if (std::abs(high-low-nominal.wall_thickness) < 1e-9) {
      require(low*high > 0,"INVALID_MODEL","Thin EU6423 wall must lie on one side of zero");
      box.low[a] = low + (low > 0 ? delta : -delta);
      box.high[a] = high + (low > 0 ? delta : -delta);
    } else if (low < 0 && high > 0) {
      box.low[a] -= delta; box.high[a] += delta;
    } else {
      require(std::abs(std::min(std::abs(low),std::abs(high))-.055) < 1e-9,
        "INVALID_MODEL","Unsupported EU6423 parametric interval");
      if (low > 0) box.high[a] += delta; else box.low[a] -= delta;
    }
    require(box.low[a] < box.high[a],"INVALID_GEOMETRY","Parametric box became nonpositive");
  }
  return model;
}
namespace {
using Group = std::tuple<uint32_t,uint32_t,uint32_t,int>; // camera, frame, axis, wall side
struct Line { Vec3 mean, direction; double sigma; };
std::optional<Line> line(const std::vector<BoundaryPoint> &points, const std::vector<std::size_t> &indices) {
  if (indices.size() < 12) return std::nullopt;
  Vec3 mean = Vec3::Zero(); double variance = 0;
  for (auto i : indices) { mean += points[i].point; variance += points[i].sigma*points[i].sigma; }
  mean /= indices.size(); Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  for (auto i : indices) { const Vec3 d = points[i].point-mean; scatter += d*d.transpose()/indices.size(); }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(scatter);
  require(eig.info() == Eigen::Success,"INVALID_GEOMETRY","Boundary line decomposition failed");
  const Vec3 direction = eig.eigenvectors().col(2);
  double lo = 0, hi = 0;
  for (auto i : indices) { const double z = direction.dot(points[i].point-mean); lo = std::min(lo,z); hi = std::max(hi,z); }
  if (hi-lo < .05 || std::sqrt(std::max(0.,eig.eigenvalues()[0]+eig.eigenvalues()[1])) > .003) return std::nullopt;
  return Line{mean,direction,std::sqrt(variance/indices.size()/std::min<std::size_t>(12,indices.size()))};
}
struct Pair { Group key; std::vector<std::size_t> indices; double value, variance; };
std::vector<Pair> pairs(const std::vector<BoundaryPoint> &points) {
  std::map<Group,std::array<std::vector<std::size_t>,2>> groups;
  for (std::size_t i=0; i<points.size(); ++i) {
    const auto &p = points[i];
    require(p.point.allFinite() && p.wall_normal.allFinite() && std::abs(p.wall_normal.norm()-1) < 1e-9 && p.axis < 2 && (p.side == -1 || p.side == 1) &&
      (p.wall_side == -1 || p.wall_side == 1) && std::isfinite(p.sigma) && p.sigma > 0 && p.sigma <= .1,
      "INVALID_GEOMETRY","Invalid boundary observation");
    groups[{p.camera,p.frame,p.axis,p.wall_side}][p.side > 0].push_back(i);
  }
  std::vector<Pair> result;
  for (const auto &[key,sides] : groups) {
    const auto a = line(points,sides[0]), b = line(points,sides[1]);
    if (!a || !b || std::abs(a->direction.dot(b->direction)) < std::cos(.17453292519943295)) continue;
    const Vec3 vertical = (a->direction + (a->direction.dot(b->direction) > 0 ? b->direction : -b->direction)).normalized();
    Vec3 normal = Vec3::Zero(); const Vec3 reference = points[sides[0][0]].wall_normal;
    bool coplanar = true;
    for (const auto &indices : sides) for (auto i : indices) {
      const Vec3 n = points[i].wall_normal; const double alignment = reference.dot(n);
      coplanar &= std::abs(alignment) >= std::cos(.17453292519943295);
      normal += alignment > 0 ? n : -n;
    }
    if (!coplanar || normal.norm() < 1e-6) continue;
    normal.normalize();
    const Vec3 delta = b->mean-a->mean;
    if (std::abs(normal.dot(delta)) > .003) continue;
    const double value = (delta-vertical*vertical.dot(delta)).norm();
    if (value <= .1 || value >= 1.) continue;
    Pair pair{key,sides[0],value,a->sigma*a->sigma+b->sigma*b->sigma};
    pair.indices.insert(pair.indices.end(),sides[1].begin(),sides[1].end()); result.push_back(std::move(pair));
  }
  return result;
}
}
std::vector<DimensionMeasurement> measure_dimensions(const std::vector<BoundaryPoint> &points) {
  std::map<std::pair<uint32_t,uint32_t>,std::vector<Pair>> groups;
  for (auto &pair : pairs(points)) groups[{std::get<2>(pair.key),std::get<0>(pair.key)}].push_back(std::move(pair));
  std::vector<DimensionMeasurement> result;
  for (const auto &[key,items] : groups) {
    double mean = 0, variance = 0; uint32_t support = 0;
    for (const auto &p : items) { mean += p.value/items.size(); variance += p.variance/items.size(); support += p.indices.size(); }
    // Frames share calibration and systematic edge uncertainty. Do not divide
    // this floor or per-frame random sensitivity by the number of frames.
    result.push_back({key.first,key.second,support,mean,std::sqrt(variance+.002*.002)});
  }
  for (std::size_t i=0; i<result.size(); ++i) for (std::size_t j=i+1; j<result.size(); ++j)
    if (result[i].axis == result[j].axis) require(std::abs(result[i].value-result[j].value) <=
      3*std::hypot(result[i].sigma,result[j].sigma),"GEOMETRY_CONFLICT","Independent camera dimensions disagree beyond three propagated sigmas");
  return result;
}
std::vector<BoundaryPoint> image_boundaries(const ImageView &depth, const ImageView &mask,
  const Intrinsics &k, const Pose &body_camera, const Model &nominal,
  uint32_t frame, uint32_t camera, double depth_sigma, uint32_t maximum_points) {
  validate_images(depth,mask,k);
  require(body_camera.matrix().allFinite() &&
    (body_camera.linear().transpose()*body_camera.linear()).isApprox(Eigen::Matrix3d::Identity(),1e-9) &&
    std::abs(body_camera.linear().determinant()-1) < 1e-9 && std::isfinite(depth_sigma) && depth_sigma > 0 && depth_sigma <= .1,
    "INVALID_GEOMETRY","Invalid boundary transform, sigma or budget");
  require(maximum_points > 0,"BUFFER_LIMIT","No boundary point budget remains");
  const auto masked = [&](uint32_t u,uint32_t v) { return mask.data[v*mask.step+u] != 0; };
  const auto point = [&](double u,double v,double z) { return Vec3((u-k.cx)*z/k.fx,(v-k.cy)*z/k.fy,z); };
  std::vector<BoundaryPoint> raw;
  const Pose local = nominal.prior.inverse()*body_camera;
  constexpr double cos10 = .984807753012208;
  for (uint32_t v=6; v+6<depth.height; v+=2) for (uint32_t u=3; u+4<depth.width; ++u) {
    if (masked(u,v) == masked(u+1,v)) continue;
    const uint32_t object_u = masked(u,v) ? u : u+1, background_u = masked(u,v) ? u+1 : u;
    const double z = axial_depth(depth,object_u,v), background = axial_depth(depth,background_u,v);
    if (!std::isfinite(z) || z < .3 || z > 3 || !std::isfinite(background) || background > 3 || background < z+.03) continue;
    std::vector<Vec3> neighborhood;
    const int inward = masked(u,v) ? -1 : 1;
    for (int y=-6; y<=6; ++y) for (int x=1; x<=12; ++x) {
      // Support the wall from inward pixels, excluding the silhouette pixel
      // where an adjacent thin wall/bevel can contaminate its normal.
      const int column = int(object_u)+inward*x;
      if (column < 0 || column >= int(depth.width)) continue;
      const auto px = static_cast<uint32_t>(column), py = static_cast<uint32_t>(int(v)+y);
      if (!masked(px,py)) continue;
      const double d = axial_depth(depth,px,py);
      if (std::isfinite(d) && std::abs(d-z) < .025) neighborhood.push_back(point(px,py,d));
    }
    if (neighborhood.size() < 12) continue;
    Vec3 mean = Vec3::Zero(); for (const auto &p : neighborhood) mean += p/neighborhood.size();
    Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
    for (const auto &p : neighborhood) { const Vec3 d = p-mean; scatter += d*d.transpose()/neighborhood.size(); }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(scatter);
    require(eig.info() == Eigen::Success,"INVALID_GEOMETRY","Local boundary plane decomposition failed");
    if (std::sqrt(std::max(0.,eig.eigenvalues()[0])) > .003) continue;
    const Vec3 normal = eig.eigenvectors().col(0), model_normal = local.linear()*normal;
    int wall_axis = std::abs(model_normal.x()) > cos10 ? 0 : std::abs(model_normal.y()) > cos10 ? 1 : -1;
    if (wall_axis < 0) continue;
    const Vec3 ray = point(u+.5,v,1.);
    const double denominator = normal.dot(ray);
    if (std::abs(denominator) < 1e-6) continue;
    const Vec3 edge = ray*(normal.dot(mean)/denominator), model_edge = local*edge;
    const int axis = 1-wall_axis, side = model_edge[axis] > 0 ? 1 : -1, wall_side = model_edge[wall_axis] > 0 ? 1 : -1;
    if (std::abs(model_edge[wall_axis]-wall_side*nominal.dimensions[wall_axis]/2) > .06 ||
        std::abs(model_edge[axis]-side*nominal.dimensions[axis]/2) > .06) continue;
    require(raw.size() < maximum_points,"BUFFER_LIMIT","Boundary point budget exceeded");
    const double sigma = std::hypot(z/k.fx/std::sqrt(12.),depth_sigma);
    raw.push_back({body_camera*edge,frame,camera,static_cast<uint32_t>(axis),side,wall_side,sigma,body_camera.linear()*normal});
  }
  std::vector<BoundaryPoint> result;
  for (const auto &pair : pairs(raw)) {
    // A camera plane and both near-vertical endpoint lines must belong to the
    // same wall, not two silhouette corners on different walls.
    const Pose inverse = nominal.prior.inverse();
    bool vertical = true;
    std::array<std::vector<std::size_t>,2> sides;
    for (auto i : pair.indices) { const bool side = raw[i].side > 0; sides[side].push_back(i); }
    for (const auto &indices : sides) {
      const auto fitted = line(raw,indices); assert(fitted);
      vertical &= std::abs((inverse.linear()*fitted->direction).z()) >= cos10;
    }
    if (!vertical) continue;
    for (auto i : pair.indices) result.push_back(raw[i]);
  }
  return result;
}
}  // namespace tote_fusion
