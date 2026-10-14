#include "tote_fusion/core.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <tuple>

namespace tote_fusion {
Rejection::Rejection(std::string code, const std::string &message)
: std::runtime_error(message), code(std::move(code)) {}
void require(bool valid, const std::string &code, const std::string &message) {
  if (!valid) throw Rejection(code, message);
}
Eigen::Matrix3d skew(const Vec3 &v) {
  Eigen::Matrix3d result;
  result << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return result;
}
Pose exp(const Vec6 &delta) {
  assert(delta.allFinite());
  const Vec3 phi = delta.tail<3>();
  const double angle = phi.norm();
  const auto cross = skew(phi);
  // Analytic limits avoid division by zero; this is the SE(3) formula, not recovery.
  const double a = angle < 1e-6 ? 0.5 - angle*angle/24 : (1-std::cos(angle))/(angle*angle);
  const double b = angle < 1e-6 ? 1.0/6 - angle*angle/120 : (angle-std::sin(angle))/(angle*angle*angle);
  Pose pose = Pose::Identity();
  if (angle > 0) pose.linear() = Eigen::AngleAxisd(angle, phi/angle).toRotationMatrix();
  pose.translation() = (Eigen::Matrix3d::Identity() + a*cross + b*cross*cross) * delta.head<3>();
  return pose;
}
Vec6 log(const Pose &pose) {
  const Eigen::AngleAxisd rotation(pose.linear());
  const Vec3 phi = rotation.angle()*rotation.axis();
  const auto cross = skew(phi);
  const double angle = rotation.angle();
  const double c = angle < 1e-6 ? 1.0/12 + angle*angle/720 :
    (1 - angle/(2*std::tan(angle/2)))/(angle*angle);
  Vec6 result;
  result << (Eigen::Matrix3d::Identity() - 0.5*cross + c*cross*cross)*pose.translation(), phi;
  return result;
}
std::string sha256(const std::string &bytes) {
  std::array<unsigned char, SHA256_DIGEST_LENGTH> hash;
  SHA256(reinterpret_cast<const unsigned char *>(bytes.data()), bytes.size(), hash.data());
  std::ostringstream result;
  result << std::hex << std::setfill('0');
  for (const auto byte : hash) result << std::setw(2) << static_cast<unsigned>(byte);
  return result.str();
}
namespace {
std::string read_file(const std::filesystem::path &path) {
  std::ifstream stream(path, std::ios::binary);
  require(stream.good(), "INVALID_MODEL", "Cannot open " + path.string());
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}
Vec3 vector(const YAML::Node &node) {
  require(node.IsSequence() && node.size() == 3, "INVALID_MODEL", "Expected a three-component vector");
  Vec3 result(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
  require(result.allFinite(), "INVALID_MODEL", "Nonfinite model vector");
  return result;
}
void validate_pose(const Pose &pose) {
  require(pose.matrix().allFinite() &&
    (pose.linear().transpose()*pose.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-9) &&
    std::abs(pose.linear().determinant()-1) < 1e-9,
    "INVALID_TRANSFORM", "Transform must contain a finite proper rotation");
}
}  // namespace

Model load_model(const std::string &path) {
  // A bad file/schema is a startup failure, never replaced by a default model.
  const auto snapshot = YAML::Load(read_file(path));
  require(snapshot["schema_version"].as<int>() == 1 && snapshot["estimation_mode"].as<std::string>() == "POSE_ONLY" &&
    snapshot["geometry_frame"].as<std::string>() == "tote" && snapshot["source_geometry_frame"].as<std::string>() == "tote_base",
    "INVALID_MODEL", "Unsupported model schema, mode or geometry frame");
  const auto geometry_path = std::filesystem::path(path).parent_path() / snapshot["geometry_file"].as<std::string>();
  const auto bytes = read_file(geometry_path);
  const auto geometry = YAML::Load(bytes);
  Model model;
  model.id = snapshot["model_id"].as<std::string>();
  model.revision = snapshot["model_revision"].as<uint64_t>();
  model.hash = snapshot["geometry_hash"].as<std::string>();
  model.body_frame = snapshot["body_frame"].as<std::string>();
  require(!model.id.empty() && !model.body_frame.empty() && sha256(bytes) == model.hash &&
    geometry["model"].as<std::string>() == model.id && geometry["units"].as<std::string>() == "m" &&
    geometry["frame"].as<std::string>() == "tote_base", "INVALID_MODEL", "Invalid model identity, hash or units");
  const auto pose = snapshot["prior_pose"];
  const auto q = pose["orientation_xyzw"];
  require(q.IsSequence() && q.size() == 4, "INVALID_MODEL", "Prior needs an xyzw quaternion");
  Eigen::Quaterniond quaternion(q[3].as<double>(), q[0].as<double>(), q[1].as<double>(), q[2].as<double>());
  require(quaternion.coeffs().allFinite() && std::abs(quaternion.norm()-1) < 1e-9,
    "INVALID_MODEL", "Prior quaternion is not finite and unit length");
  model.prior = Pose::Identity();
  model.prior.linear() = quaternion.toRotationMatrix();
  model.prior.translation() = vector(pose["position_m"]);
  const auto covariance = snapshot["prior_covariance"];
  require(covariance["convention"].as<std::string>() == "left body-frame tangent perturbation; metres/radians" &&
    covariance["ordering"].as<std::vector<std::string>>() == std::vector<std::string>{"x", "y", "z", "rotX", "rotY", "rotZ"},
    "INVALID_MODEL", "Unsupported prior covariance convention");
  const auto rows = covariance["matrix"];
  require(rows.IsSequence() && rows.size() == 6, "INVALID_MODEL", "Covariance must be 6x6");
  for (int row=0; row<6; ++row) {
    require(rows[row].IsSequence() && rows[row].size() == 6, "INVALID_MODEL", "Covariance must be 6x6");
    for (int column=0; column<6; ++column) model.prior_covariance(row, column) = rows[row][column].as<double>();
  }
  require(model.prior_covariance.allFinite() && model.prior_covariance.isApprox(model.prior_covariance.transpose(), 1e-12) &&
    Eigen::LLT<Mat6>(model.prior_covariance).info() == Eigen::Success,
    "INVALID_MODEL", "Covariance must be finite symmetric positive definite");
  const auto dimensions = geometry["nominal_outer_dimensions_m"];
  model.dimensions = Vec3(dimensions["length"].as<double>(), dimensions["width"].as<double>(), dimensions["height"].as<double>());
  model.wall_thickness = geometry["wall_thickness_m"].as<double>();
  require(std::isfinite(model.wall_thickness) && model.wall_thickness > 0 && model.wall_thickness < .01,
    "INVALID_MODEL", "Invalid nominal wall thickness");
  model.bevel = geometry["estimator_exclusions"]["edge_bevel_radius_m"].as<double>();
  const auto label = geometry["surfaces"][1]["raised_label_exclusion_z_m"];
  model.label_low = label[0].as<double>(); model.label_high = label[1].as<double>();
  require(model.dimensions.allFinite() && (model.dimensions.array() > 0).all() &&
    std::isfinite(model.bevel) && model.bevel >= 0 && std::isfinite(model.label_low) &&
    std::isfinite(model.label_high) && model.label_low < model.label_high,
    "INVALID_MODEL", "Invalid nominal dimensions or feature exclusions");
  const auto parts = geometry["collision"]["parts"];
  require(parts.IsSequence() && parts.size() > 0 && parts.size() <= 100,
    "INVALID_MODEL", "Expected bounded nominal box geometry");
  for (const auto part : parts) {
    const Vec3 size = vector(part["size_m"]), center = vector(part["center_m"]);
    require((size.array() > 0).all(), "INVALID_MODEL", "Nonpositive model box size");
    model.boxes.push_back({center-size/2, center+size/2});
  }
  return model;
}

std::optional<SurfaceHit> raycast(const Model &model, const Vec3 &origin,
                                 const Vec3 &direction) {
  assert(origin.allFinite() && direction.allFinite() && direction.squaredNorm() > 0);
  double nearest = std::numeric_limits<double>::infinity();
  std::optional<SurfaceHit> hit;
  for (std::size_t index=0; index<model.boxes.size(); ++index) {
    const auto &box = model.boxes[index];
    double enter = -std::numeric_limits<double>::infinity(), leave = nearest;
    int axis = -1; double sign = 0;
    for (int a=0; a<3; ++a) {
      if (std::abs(direction[a]) < 1e-12) {
        if (origin[a] < box.low[a] || origin[a] > box.high[a]) { leave = -1; break; }
        continue;
      }
      double low = (box.low[a]-origin[a])/direction[a], high = (box.high[a]-origin[a])/direction[a];
      const double normal_sign = direction[a] > 0 ? -1 : 1;
      if (low > high) std::swap(low, high);
      if (low > enter) { enter = low; axis = a; sign = normal_sign; }
      leave = std::min(leave, high);
    }
    if (enter <= 0 || enter > leave || enter >= nearest) continue;
    assert(axis >= 0);
    nearest = enter;
    Vec3 normal = Vec3::Zero(); normal[axis] = sign;
    hit = SurfaceHit{origin + enter*direction, normal, index*6 + axis*2 + static_cast<std::size_t>(sign > 0)};
  }
  if (!hit) return hit;
  const auto &box = model.boxes[hit->face/6];
  for (int axis=0; axis<3; ++axis) {
    if (hit->normal[axis] == 0 &&
        std::min(hit->point[axis]-box.low[axis], box.high[axis]-hit->point[axis]) < model.bevel) return std::nullopt;
  }
  const int axis = static_cast<int>((hit->face % 6)/2);
  if (axis < 2 && std::abs(hit->point[axis]) > model.dimensions[axis]/2 - 1e-6 &&
      hit->point.z() >= model.label_low && hit->point.z() <= model.label_high) return std::nullopt;
  return hit;
}

void validate_images(const ImageView &depth, const ImageView &mask, const Intrinsics &k) {
  require(std::isfinite(k.fx) && std::isfinite(k.fy) && std::isfinite(k.cx) && std::isfinite(k.cy) &&
    k.fx > 0 && k.fy > 0 && k.cx >= 0 && k.cy >= 0 && k.cx < depth.width && k.cy < depth.height,
    "INVALID_INTRINSICS", "Invalid pinhole intrinsics");
  require(depth.width > 0 && depth.height > 0 && depth.width <= 4096 && depth.height <= 4096 &&
    !depth.big_endian && (depth.encoding == "32FC1" || depth.encoding == "16UC1"),
    "INVALID_DEPTH", "Expected bounded little-endian axial depth: 32FC1 metres or 16UC1 millimetres");
  const std::size_t element = depth.encoding == "32FC1" ? 4 : 2;
  require(depth.data && depth.step >= depth.width*element && depth.step <= depth.bytes/depth.height,
    "INVALID_DEPTH", "Truncated depth image or invalid row stride");
  require(mask.data && mask.width == depth.width && mask.height == depth.height &&
    (mask.encoding == "mono8" || mask.encoding == "8UC1") && mask.step >= mask.width && mask.step <= mask.bytes/mask.height,
    "MASK_MISMATCH", "Mask format, dimensions or row stride differs from depth");
}
double axial_depth(const ImageView &depth, uint32_t u, uint32_t v) {
  assert(u < depth.width && v < depth.height);
  if (depth.encoding == "32FC1") {
    float value; std::memcpy(&value, depth.data+v*depth.step+u*4, 4); return value;
  }
  assert(depth.encoding == "16UC1");
  uint16_t value; std::memcpy(&value, depth.data+v*depth.step+u*2, 2);
  const double z = value*0.001; return z;
}
std::vector<Vec3> project(const ImageView &depth, const ImageView &mask, const Intrinsics &k,
                         double minimum_z, double maximum_z, uint32_t pixel_stride, uint32_t maximum_points) {
  validate_images(depth,mask,k);
  require(pixel_stride > 0 && std::isfinite(minimum_z) && std::isfinite(maximum_z) &&
    minimum_z > 0 && maximum_z > minimum_z, "INVALID_CONFIG", "Invalid sampling or depth limits");
  require(maximum_points > 0, "BUFFER_LIMIT", "No projection point budget remains");
  std::vector<Vec3> result;
  for (uint32_t v=0; v<depth.height; v+=pixel_stride) for (uint32_t u=0; u<depth.width; u+=pixel_stride) {
    if (!mask.data[v*mask.step+u]) continue;
    const double z = axial_depth(depth,u,v);
    if (!std::isfinite(z) || z < minimum_z || z > maximum_z) continue;
    require(result.size() < maximum_points, "BUFFER_LIMIT", "Projection point budget exceeded");
    result.emplace_back((u-k.cx)*z/k.fx, (v-k.cy)*z/k.fy, z);
  }
  require(!result.empty(), "INVALID_DEPTH", "No valid masked axial-depth samples");
  return result;
}
std::vector<Vec3> transform_points(std::vector<Vec3> points, const Pose &body_camera) {
  validate_pose(body_camera);
  for (auto &point : points) {
    require(point.allFinite(), "INVALID_DEPTH", "Nonfinite point at transform boundary");
    point = body_camera * point;
  }
  return points;
}
void validate_capture(int64_t now, int64_t capture, int64_t start, int64_t maximum_age, int64_t future_tolerance) {
  assert(maximum_age > 0 && future_tolerance >= 0);
  require(now > 0 && capture > 0 && capture >= start && now-capture <= maximum_age &&
    capture-now <= future_tolerance, "STALE_OBSERVATION", "Capture is stale, future, or outside this request");
}

namespace {
struct CameraQuality { double squared_error = 0; uint32_t count = 0, inliers = 0; };
struct Linearization {
  Eigen::MatrixXd h; Eigen::VectorXd g;
  explicit Linearization(int n) : h(Eigen::MatrixXd::Zero(n,n)), g(Eigen::VectorXd::Zero(n)) {}
  double squared_error = 0;
  uint32_t count = 0, inliers = 0;
  std::set<std::size_t> faces;
  std::map<uint32_t, CameraQuality> cameras;
  std::map<std::tuple<uint32_t,uint32_t,int>,CameraQuality> boundary_quality;
  std::map<std::pair<uint32_t,uint32_t>, std::pair<double,uint32_t>> normal_bias;
};
Linearization linearize(const Model &model, const Pose &pose, const std::vector<Observation> &observations,
                        const FitConfig &config, const std::function<bool()> &cancel,
                        const std::vector<uint32_t> &axes = {}, const std::vector<BoundaryPoint> &boundaries = {}) {
  Linearization result(6+axes.size());
  std::map<std::pair<uint32_t,uint32_t>, uint32_t> frame_counts;
  std::map<uint32_t, std::set<uint32_t>> camera_frames;
  for (const auto &observation : observations) {
    require(observation.camera < config.camera_noise_sigma_m.size(), "INVALID_CAMERA", "Unknown observation camera index");
    ++frame_counts[{observation.camera,observation.frame}]; camera_frames[observation.camera].insert(observation.frame);
  }
  const Pose inverse = pose.inverse();
  for (std::size_t index=0; index<observations.size(); ++index) {
    if (index % 128 == 0) require(!cancel(), "CANCELED", "Estimation canceled");
    const auto &sample = observations[index];
    const Vec3 ray = sample.point-sample.origin;
    const double length = ray.norm();
    require(sample.point.allFinite() && sample.origin.allFinite() && std::isfinite(length) && length > 0,
      "INVALID_DEPTH", "Invalid or numerically unrepresentable observation ray");
    const Vec3 origin = inverse * sample.origin, point = inverse * sample.point;
    // Rotate the ray directly; subtracting two translated large coordinates
    // can lose its direction. Invalid external rays must reject before assert.
    const auto hit = raycast(model, origin, inverse.linear()*(ray/length));
    if (!hit) continue;
    const double residual = hit->normal.dot(point-hit->point);
    if (std::abs(residual) > config.correspondence_limit_m) continue;
    const Vec3 normal = pose.linear()*hit->normal;
    Eigen::VectorXd jacobian = Eigen::VectorXd::Zero(6+axes.size());
    jacobian.head<3>() = -normal; jacobian.segment<3>(3) = normal.cross(sample.point);
    for (std::size_t k=0; k<axes.size(); ++k) {
      const int a = (hit->face%6)/2;
      if (a != int(axes[k])) continue;
      const auto &box = model.boxes[hit->face/6];
      // Supported dimension faces move by signed half-delta; fixed handle
      // sides at +/-55 mm do not move.
      const double coordinate = hit->normal[a] < 0 ? box.low[a] : box.high[a];
      const double derivative = std::abs(std::abs(coordinate)-.055) < 1e-9 ? 0 : coordinate > 0 ? .5 : -.5;
      jacobian[6+k] = -hit->normal[a]*derivative;
    }
    const double sigma = config.camera_noise_sigma_m.at(sample.camera);
    const double standardized = std::abs(residual)/sigma;
    const double huber = standardized > config.huber_sigma ? config.huber_sigma/standardized : 1;
    const auto count = frame_counts.at({sample.camera,sample.frame});
    // M1 keeps its calibrated frame weighting. Multi-view information is
    // capped per camera, independently of its frame/pixel density.
    const double frames = config.camera_noise_sigma_m.size() == 1 ? 1 : camera_frames.at(sample.camera).size();
    const double weight = huber*std::min(count, config.effective_points_per_frame)/(count*sigma*sigma*frames);
    result.h.noalias() += weight*jacobian*jacobian.transpose();
    result.g.noalias() += weight*jacobian*residual;
    result.squared_error += residual*residual;
    auto &camera = result.cameras[sample.camera]; camera.squared_error += residual*residual; ++camera.count;
    camera.inliers += std::abs(residual) <= config.inlier_limit_m;
    auto &bias = result.normal_bias[{sample.camera,static_cast<uint32_t>((hit->face%6)/2)}];
    bias.first += residual; ++bias.second;
    ++result.count;
    result.inliers += std::abs(residual) <= config.inlier_limit_m;
    result.faces.insert(hit->face);
  }
  std::map<std::tuple<uint32_t,uint32_t,int>,uint32_t> boundary_counts;
  for (const auto &p : boundaries) ++boundary_counts[{p.camera,p.axis,p.side}];
  for (std::size_t i=0; i<boundaries.size(); ++i) {
    if (i%128 == 0) require(!cancel(),"CANCELED","Geometry fitting canceled");
    const auto &p = boundaries[i];
    auto axis = std::find(axes.begin(),axes.end(),p.axis);
    assert(axis != axes.end());
    const Vec3 local = inverse*p.point;
    const double residual = p.side*local[p.axis]-model.dimensions[p.axis]/2;
    auto &quality = result.boundary_quality[{p.camera,p.axis,p.side}];
    quality.squared_error += residual*residual; ++quality.count;
    const Vec3 normal = pose.linear()*(p.side*Vec3::Unit(p.axis));
    Eigen::VectorXd jacobian = Eigen::VectorXd::Zero(6+axes.size());
    jacobian.head<3>() = -normal; jacobian.segment<3>(3) = normal.cross(p.point);
    jacobian[6+std::distance(axes.begin(),axis)] = -.5;
    const double standardized = std::abs(residual)/p.sigma;
    const double huber = standardized > config.huber_sigma ? config.huber_sigma/standardized : 1;
    const auto count = boundary_counts.at({p.camera,p.axis,p.side});
    const double weight = huber*std::min(count,12u)/(count*p.sigma*p.sigma);
    result.h.noalias() += weight*jacobian*jacobian.transpose();
    result.g.noalias() += weight*jacobian*residual;
  }
  return result;
}
}  // namespace

static Estimate fit_hypothesis(const Model &model, const std::vector<Observation> &fitting,
                  const std::vector<Observation> &held_out, const FitConfig &config,
                  const std::function<bool()> &cancel, const std::vector<BoundaryPoint> &boundary_fitting,
                  const std::vector<BoundaryPoint> &boundary_held_out) {
  require(!config.camera_noise_sigma_m.empty() && config.camera_noise_sigma_m.size() <= 3,
    "INVALID_CONFIG", "Need one to three camera noise profiles");
  for (const auto sigma : config.camera_noise_sigma_m) require(std::isfinite(sigma) && sigma >= 1e-6 && sigma <= .1,
    "INVALID_CONFIG", "Camera sigma must be within the 1 micrometre..100 mm exercise range");
  for (const double value : {config.position_uncertainty_floor_m, config.camera_bias_limit_m, config.huber_sigma, config.correspondence_limit_m,
      config.inlier_limit_m, config.maximum_rmse_m, config.minimum_inlier_fraction,
      config.minimum_information_ratio, config.step_tolerance, config.rotation_uncertainty_floor_rad})
    require(std::isfinite(value), "INVALID_CONFIG", "Nonfinite fit configuration");
  require(config.position_uncertainty_floor_m > 0 && config.camera_bias_limit_m > 0 && config.huber_sigma > 0 && config.correspondence_limit_m > config.inlier_limit_m &&
    config.inlier_limit_m > 0 && config.maximum_rmse_m > 0 && config.minimum_inlier_fraction > 0 &&
    config.minimum_inlier_fraction <= 1 && config.minimum_information_ratio > 0 && config.minimum_information_ratio < 1 &&
    config.step_tolerance > 0 && config.maximum_iterations > 0 && config.minimum_points >= 6 &&
    config.effective_points_per_frame > 0 && config.rotation_uncertainty_floor_rad > 0,
    "INVALID_CONFIG", "Invalid fit profile");
  require(fitting.size() >= config.minimum_points && held_out.size() >= config.minimum_points,
    "INSUFFICIENT_DATA", "Need separate fitting and held-out frame samples");
  std::set<std::pair<uint32_t,uint32_t>> fit_frames;
  for (const auto &sample : fitting) fit_frames.insert({sample.camera,sample.frame});
  for (const auto &sample : held_out) require(!fit_frames.count({sample.camera,sample.frame}),
    "INVALID_HOLDOUT", "Fitting and held-out sets must use disjoint acquisition frames");
  std::vector<uint32_t> axes;
  std::vector<DimensionMeasurement> measurements;
  if (config.estimate_geometry) {
    const auto train_measurements = measure_dimensions(boundary_fitting);
    const auto held_measurements = measure_dimensions(boundary_held_out);
    require(!train_measurements.empty() && !held_measurements.empty(),"INSUFFICIENT_GEOMETRY","Need paired boundaries in fitting and held-out frames");
    for (uint32_t axis : {1u,0u}) {
      bool supported = false;
      for (const auto &m : train_measurements) if (m.axis == axis) {
        const auto counterpart = std::find_if(held_measurements.begin(),held_measurements.end(),[&](const auto &v) { return v.axis == m.axis && v.camera == m.camera; });
        require(counterpart != held_measurements.end(),"INSUFFICIENT_GEOMETRY","Camera dimension lacks independent held-out boundaries");
        require(std::abs(m.value-counterpart->value) <= 3*std::hypot(m.sigma,counterpart->sigma),"GEOMETRY_CONFLICT","Fitting and held-out dimensions disagree");
        measurements.push_back(*counterpart); supported = true;
      }
      if (supported) axes.push_back(axis);
    }
    require(measurements.size() == held_measurements.size(),"INSUFFICIENT_GEOMETRY","Held-out camera dimension lacks fitting support");
    for (const auto *set : {&boundary_fitting,&boundary_held_out}) for (const auto &p : *set) {
      require(p.camera < config.camera_noise_sigma_m.size(),"INVALID_CAMERA","Unknown boundary camera");
      require(std::find(axes.begin(),axes.end(),p.axis) != axes.end(),"INSUFFICIENT_GEOMETRY","Unpaired parameter support");
    }
    for (const auto &p : boundary_fitting) fit_frames.insert({p.camera,p.frame});
    for (const auto &p : held_out) require(!fit_frames.count({p.camera,p.frame}),"INVALID_HOLDOUT","Held-out surface frame overlaps boundary fitting");
    for (const auto &p : boundary_held_out) require(!fit_frames.count({p.camera,p.frame}),"INVALID_HOLDOUT","Boundary held-out frame overlaps fitting data");
  } else require(boundary_fitting.empty() && boundary_held_out.empty(),"UNSUPPORTED_MODE","Pose-only fitting cannot consume geometry constraints");
  Pose pose = model.prior;
  Vec3 dimensions = model.dimensions;
  const Mat6 prior_information = model.prior_covariance.inverse();
  uint32_t iterations = 0;
  const auto solve = [&](const std::vector<uint32_t> &active, const std::vector<BoundaryPoint> &boundaries) {
    bool converged = false;
    const int n = 6+active.size();
    for (uint32_t iteration=0; iteration<config.maximum_iterations; ++iteration) {
      ++iterations;
      const auto current = active.empty() ? model : parametric_model(model,dimensions);
      auto fit = linearize(current,pose,fitting,config,cancel,active,boundaries);
      require(fit.count >= config.minimum_points,"INSUFFICIENT_DATA","Too few visible model correspondences");
      const Vec6 prior_error = log(pose*model.prior.inverse());
      Mat6 prior_jacobian;
      for (int axis=0; axis<6; ++axis) {
        Vec6 step = Vec6::Zero(); step[axis] = 1e-6;
        prior_jacobian.col(axis) = (log(exp(step)*pose*model.prior.inverse())-prior_error)/1e-6;
      }
      fit.h.topLeftCorner<6,6>().noalias() += prior_jacobian.transpose()*prior_information*prior_jacobian;
      fit.g.head<6>().noalias() += prior_jacobian.transpose()*prior_information*prior_error;
      const Eigen::LDLT<Eigen::MatrixXd> factor(fit.h+1e-6*Eigen::MatrixXd::Identity(n,n));
      require(factor.info() == Eigen::Success && factor.isPositive(),"UNOBSERVABLE","Registration normal equation is invalid");
      const Eigen::VectorXd step = factor.solve(-fit.g);
      require(step.allFinite() && step.norm() < .5,"POOR_FIT","Registration step is nonfinite or outside the local basin");
      pose = exp(step.head<6>())*pose;
      for (std::size_t k=0; k<active.size(); ++k) dimensions[active[k]] += step[6+k];
      if (step.norm() < config.step_tolerance) { converged = true; break; }
    }
    return converged;
  };
  const bool fixed_converged = solve({},{});
  const uint32_t fixed_iterations = iterations;
  require(config.estimate_geometry || fixed_converged,"POOR_FIT","Fixed geometry registration did not converge within its iteration budget");
  const auto fixed_validation = linearize(model,pose,held_out,config,cancel);
  require(fixed_validation.count >= config.minimum_points,"INSUFFICIENT_DATA","Fixed stage lacks held-out visible support");
  const double fixed_surface_rmse = std::sqrt(fixed_validation.squared_error/fixed_validation.count);
  const auto boundary_rmse = [&](const std::vector<BoundaryPoint> &points) {
    assert(!points.empty()); double squared = 0;
    for (const auto &p : points) { const double r = p.side*(pose.inverse()*p.point)[p.axis]-dimensions[p.axis]/2; squared += r*r; }
    return std::sqrt(squared/points.size());
  };
  const double fixed_boundary_rmse = axes.empty() ? 0 : boundary_rmse(boundary_held_out);
  if (!axes.empty()) require(solve(axes,boundary_fitting),"POOR_FIT","Joint geometry registration did not converge within its iteration budget");
  const auto fitted_model = axes.empty() ? model : parametric_model(model,dimensions);
  const auto train = linearize(fitted_model,pose,fitting,config,cancel,axes,boundary_fitting);
  const auto test = linearize(fitted_model,pose,held_out,config,cancel,axes,boundary_held_out);
  require(train.count >= config.minimum_points && test.count >= config.minimum_points,
    "INSUFFICIENT_DATA", "Insufficient fitting or held-out visible support");
  std::vector<double> camera_rmse(config.camera_noise_sigma_m.size(), 0);
  std::vector<uint32_t> input_count(config.camera_noise_sigma_m.size(),0);
  for (const auto *set : {&fitting,&held_out}) for (const auto &sample : *set) ++input_count.at(sample.camera);
  for (uint32_t camera=0; camera<input_count.size(); ++camera) {
    if (input_count[camera] == 0) continue;
    require(train.cameras.count(camera) && test.cameras.count(camera),
      "INSUFFICIENT_DATA", "Camera " + std::to_string(camera) + " lacks fitting/held-out model support; raw_points=" + std::to_string(input_count[camera]));
    const auto &stats = train.cameras.at(camera);
    const auto &validation = test.cameras.at(camera);
    require(stats.count >= config.minimum_points && validation.count >= config.minimum_points,
      "INSUFFICIENT_DATA", "Camera " + std::to_string(camera) + " fit_points=" + std::to_string(stats.count) +
      " held_points=" + std::to_string(validation.count) + " minimum=" + std::to_string(config.minimum_points));
    camera_rmse[camera] = std::sqrt(validation.squared_error/validation.count);
    require(camera_rmse[camera] <= config.maximum_rmse_m &&
      static_cast<double>(stats.inliers+validation.inliers)/input_count.at(camera) >= config.minimum_inlier_fraction,
      config.camera_noise_sigma_m.size() == 1 ? "POOR_FIT" : "CAMERA_CONFLICT",
      "Camera " + std::to_string(camera) + " held_rmse_m=" + std::to_string(camera_rmse[camera]) +
      " inliers=" + std::to_string(stats.inliers+validation.inliers) + "/" + std::to_string(input_count[camera]));
  }
  if (train.cameras.size() > 1) for (const auto *set : {&train,&test})
    for (const auto &[group, bias] : set->normal_bias) {
      require(bias.second < 6 || std::abs(bias.first/bias.second) <= config.camera_bias_limit_m,
        "CAMERA_CONFLICT", "Camera " + std::to_string(group.first) + " axis=" + std::to_string(group.second) +
        " plane_bias_m=" + std::to_string(bias.first/bias.second) + " limit_m=" + std::to_string(config.camera_bias_limit_m));
    }
  Eigen::VectorXd scales = Eigen::VectorXd::Ones(train.h.rows());
  if (!axes.empty()) { scales.head<3>().setConstant(.01); scales.segment<3>(3).setConstant(.05); scales.tail(axes.size()).setConstant(.01); }
  const Eigen::MatrixXd scaled_information = scales.asDiagonal()*train.h*scales.asDiagonal();
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> information(scaled_information);
  require(information.info() == Eigen::Success && information.eigenvalues().allFinite(),"UNOBSERVABLE","Data-only information decomposition is invalid");
  std::ostringstream modes;
  modes << "Data-only rank=" << (information.eigenvalues().array() > config.minimum_information_ratio*information.eigenvalues().tail(1)[0]).count()
        << "/" << train.h.rows() << " eigenvalues=" << information.eigenvalues().transpose();
  require(information.eigenvalues()[0] > config.minimum_information_ratio*information.eigenvalues().tail(1)[0],"UNOBSERVABLE",modes.str());
  const double training_rmse = std::sqrt(train.squared_error/train.count);
  const double held_out_rmse = std::sqrt(test.squared_error/test.count);
  const double inlier_fraction = static_cast<double>(train.inliers+test.inliers)/(fitting.size()+held_out.size());
  require(training_rmse <= config.maximum_rmse_m && held_out_rmse <= config.maximum_rmse_m &&
    inlier_fraction >= config.minimum_inlier_fraction, "POOR_FIT", "Training/held-out residual or support gate failed");
  // Local sensitivity only, inflated by explicit exercise floors. Not calibrated confidence.
  Eigen::MatrixXd covariance = train.h.inverse();
  Eigen::MatrixXd conversion = Eigen::MatrixXd::Identity(train.h.rows(),train.h.rows());
  conversion.block<3,3>(0,3) = -skew(pose.translation());
  covariance = (conversion*covariance*conversion.transpose()).eval();
  covariance.diagonal().head<3>().array() += config.position_uncertainty_floor_m*config.position_uncertainty_floor_m;
  covariance.diagonal().segment<3>(3).array() += std::pow(config.rotation_uncertainty_floor_rad,2);
  if (!axes.empty()) covariance.diagonal().tail(axes.size()).array() += .002*.002;
  // M1/M2 calibration populations do not calibrate joint geometry confidence.
  covariance *= axes.empty() ? (config.camera_noise_sigma_m.size() == 1 ? 1.25 : 1.35) : 1.;
  require(covariance.allFinite(),"UNOBSERVABLE","Nonfinite local covariance");
  Estimate result;
  result.pose = pose; result.pi_equivalent_pose = Pose::Identity(); result.covariance = covariance.topLeftCorner<6,6>();
  const Eigen::SelfAdjointEigenSolver<Mat6> pose_information(train.h.topLeftCorner<6,6>());
  result.information_eigenvalues = pose_information.eigenvalues();
  result.training_rmse_m = training_rmse; result.held_out_rmse_m = held_out_rmse; result.inlier_fraction = inlier_fraction;
  result.fitting_points = train.count; result.held_out_points = test.count; result.surface_count = train.faces.size(); result.iterations = iterations;
  result.ambiguity_status = "PI_YAW_EQUIVALENT_LOCAL_COVARIANCE"; result.camera_held_out_rmse_m = camera_rmse;
  result.dimensions = dimensions; result.dimension_axes = axes; result.dimension_measurements = measurements;
  result.joint_covariance = covariance; result.joint_information_eigenvalues = information.eigenvalues();
  result.fixed_boundary_rmse_m = fixed_boundary_rmse; result.fixed_geometry_converged = fixed_converged;
  result.fixed_geometry_iterations = fixed_iterations; result.fixed_geometry_held_out_rmse_m = fixed_surface_rmse;
  if (!axes.empty()) {
    for (const auto *set : {&train,&test}) for (const auto &[key,quality] : set->boundary_quality) {
      const double rmse = std::sqrt(quality.squared_error/quality.count);
      require(rmse <= .005,"CAMERA_CONFLICT","Boundary camera="+std::to_string(std::get<0>(key))+
        " axis="+std::to_string(std::get<1>(key))+" side="+std::to_string(std::get<2>(key))+" rmse_m="+std::to_string(rmse));
    }
    result.held_out_boundary_rmse_m = boundary_rmse(boundary_held_out);
    require(result.held_out_boundary_rmse_m <= .005,"POOR_GEOMETRY","Held-out boundary residual exceeds 5 mm");
  }
  return result;
}
Estimate estimate(const Model &model, const std::vector<Observation> &fitting,
                  const std::vector<Observation> &held_out, const FitConfig &config,
                  const std::function<bool()> &cancel, const std::vector<BoundaryPoint> &boundary_fitting,
                  const std::vector<BoundaryPoint> &boundary_held_out) {
  auto primary = fit_hypothesis(model, fitting, held_out, config, cancel,boundary_fitting,boundary_held_out);
  Pose symmetry = Pose::Identity();
  symmetry.linear() = Eigen::AngleAxisd(std::acos(-1), Vec3::UnitZ()).toRotationMatrix();
  // Both hypotheses are supported by the nominal rectangular geometry. Each
  // uses its own equivalent prior: a prior cannot resolve global symmetry.
  auto alternative_model = model;
  alternative_model.prior = model.prior*symmetry;
  auto alternate_fit = boundary_fitting, alternate_held = boundary_held_out;
  for (auto *set : {&alternate_fit,&alternate_held}) for (auto &p : *set) { p.side = -p.side; p.wall_side = -p.wall_side; }
  const auto alternative = fit_hypothesis(alternative_model,fitting,held_out,config,cancel,alternate_fit,alternate_held);
  require(log(alternative.pose*(primary.pose*symmetry).inverse()).norm() < 1e-5,
    "AMBIGUOUS_FIT", "Equivalent initial hypotheses converged to inconsistent local fits");
  require(config.allow_pi_symmetry, "AMBIGUOUS_SYMMETRY", "Nominal tote has a pi-yaw equivalent pose; strict profile rejects it");
  primary.pi_equivalent_pose = alternative.pose;
  primary.iterations += alternative.iterations;
  return primary;
}
}  // namespace tote_fusion
