#pragma once
#include <Eigen/Geometry>
#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace tote_fusion {
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Pose = Eigen::Isometry3d;

// Expected external-input/estimation failures; programming errors are not caught.
struct Rejection : std::runtime_error {
  std::string code;
  Rejection(std::string code, const std::string &message);
};
void require(bool valid, const std::string &code, const std::string &message);
Eigen::Matrix3d skew(const Vec3 &value);
Pose exp(const Vec6 &delta);
Vec6 log(const Pose &pose);
std::string sha256(const std::string &bytes);

struct Box { Vec3 low, high; };
struct SurfaceHit { Vec3 point, normal; std::size_t face; };
struct Model {
  std::string id, hash, body_frame;
  uint64_t revision;
  Pose prior;
  Mat6 prior_covariance;
  Vec3 dimensions;
  double bevel, label_low, label_high, wall_thickness;
  std::vector<Box> boxes;
};
Model load_model(const std::string &path);
// First front-facing hit of the union of opaque nominal boxes. No truth queries.
std::optional<SurfaceHit> raycast(const Model &model, const Vec3 &origin,
                                 const Vec3 &direction);

struct ImageView {
  const uint8_t *data;
  std::size_t bytes, step;
  uint32_t width, height;
  std::string encoding;
  bool big_endian;
};
struct Intrinsics { double fx, fy, cx, cy; };
void validate_images(const ImageView &depth, const ImageView &mask, const Intrinsics &intrinsics);
double axial_depth(const ImageView &depth, uint32_t u, uint32_t v);
std::vector<Vec3> project(const ImageView &depth, const ImageView &mask,
                         const Intrinsics &intrinsics, double minimum_z,
                         double maximum_z, uint32_t pixel_stride, uint32_t maximum_points);
std::vector<Vec3> transform_points(std::vector<Vec3> points, const Pose &body_camera);
void validate_capture(int64_t now, int64_t capture, int64_t start,
                      int64_t maximum_age, int64_t future_tolerance);

struct Observation { Vec3 point, origin; uint32_t frame; uint32_t camera = 0; };
// Only validated paired silhouette lines enter geometry fitting. Internal
// corner/crop/occlusion boundaries are not physical dimension measurements.
struct BoundaryPoint {
  Vec3 point;
  uint32_t frame, camera, axis;
  int side, wall_side;
  double sigma;
  Vec3 wall_normal;
};
struct DimensionMeasurement { uint32_t axis, camera, support; double value, sigma; };
Model parametric_model(const Model &nominal, const Vec3 &dimensions);
std::vector<BoundaryPoint> image_boundaries(const ImageView &depth, const ImageView &mask,
  const Intrinsics &intrinsics, const Pose &body_camera, const Model &nominal,
  uint32_t frame, uint32_t camera, double depth_sigma, uint32_t maximum_points);
std::vector<DimensionMeasurement> measure_dimensions(const std::vector<BoundaryPoint> &points);
struct FitConfig {
  std::vector<double> camera_noise_sigma_m{0.003};
  double position_uncertainty_floor_m = 0.003, camera_bias_limit_m = 0.003;
  double huber_sigma = 2.0, correspondence_limit_m = 0.08;
  double inlier_limit_m = 0.008, maximum_rmse_m = 0.005, minimum_inlier_fraction = 0.7;
  double minimum_information_ratio = 1e-5, step_tolerance = 1e-7;
  double rotation_uncertainty_floor_rad = 0.008726646259971648;
  uint32_t maximum_iterations = 40, minimum_points = 30;
  uint32_t effective_points_per_frame = 100;
  bool allow_pi_symmetry = false;
  bool estimate_geometry = false;
};
struct Estimate {
  Pose pose, pi_equivalent_pose;
  Mat6 covariance;
  Vec6 information_eigenvalues;
  double training_rmse_m, held_out_rmse_m, inlier_fraction;
  uint32_t fitting_points, held_out_points, surface_count, iterations;
  std::string ambiguity_status;
  std::vector<double> camera_held_out_rmse_m;
  Vec3 dimensions = Vec3::Zero();
  std::vector<uint32_t> dimension_axes;
  std::vector<DimensionMeasurement> dimension_measurements;
  Eigen::MatrixXd joint_covariance;
  Eigen::VectorXd joint_information_eigenvalues;
  bool fixed_geometry_converged = false;
  uint32_t fixed_geometry_iterations = 0;
  double fixed_geometry_held_out_rmse_m = 0;
  double fixed_boundary_rmse_m = 0, held_out_boundary_rmse_m = 0;
};
Estimate estimate(const Model &model, const std::vector<Observation> &fitting,
                  const std::vector<Observation> &held_out, const FitConfig &config,
                  const std::function<bool()> &cancel,
                  const std::vector<BoundaryPoint> &boundary_fitting = {},
                  const std::vector<BoundaryPoint> &boundary_held_out = {});
}  // namespace tote_fusion
