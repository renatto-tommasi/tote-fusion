#include "geometry_fixture.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
using namespace tote_fusion;
using namespace tote_fixture;
using namespace geometry_fixture;
int main() {
  std::cout << std::setprecision(17) << "seed,population,method,code,translation_m,rotation_rad,width_m,length_m,width_sigma_m,length_sigma_m,fixed_boundary_rmse_m,held_boundary_rmse_m,wall_s,covariance\n";
  const Pose truth = actual_pose(); const Vec3 dimensions(.62,.42,.23);
  for (int seed=4000; seed<4100; ++seed) {
    std::mt19937 random(seed); std::normal_distribution<double> gaussian;
    const std::string population = seed < 4025 ? "clean" : seed < 4075 ? "noise" : "outliers";
    Pose shared = Pose::Identity(); double width_bias = 0, length_bias = 0;
    if (population != "clean") {
      shared.translation() = Vec3(gaussian(random),gaussian(random),gaussian(random))*.001;
      shared.linear() = Eigen::AngleAxisd(gaussian(random)*.0034906585,Vec3::UnitZ()).toRotationMatrix();
      width_bias = gaussian(random)*.001; length_bias = gaussian(random)*.001;
    }
    std::vector<Observation> fitting,held;
    std::vector<BoundaryPoint> edges,held_edges;
    for (uint32_t camera=0; camera<2; ++camera) for (uint32_t split=0; split<2; ++split) {
      auto points = surfaces(truth,split,dimensions);
      for (std::size_t i=0; i<points.size(); ++i) {
        auto &p = points[i]; p.camera = camera;
        if (population != "clean") p.point += (p.point-p.origin).normalized()*gaussian(random)*(.0015+camera*.0005);
        if (population == "outliers" && i%20 == 0) p.point += (p.point-p.origin).normalized()*.02;
        p.point = shared*p.point; p.origin = shared*p.origin;
      }
      Vec3 measured_dimensions = dimensions;
      measured_dimensions[1-camera] += camera == 0 ? width_bias : length_bias;
      auto boundaries_ = boundaries(truth,split,1-camera,measured_dimensions,camera);
      for (auto &p : boundaries_) {
        if (population != "clean") p.point += truth.linear()*Vec3(gaussian(random),gaussian(random),gaussian(random))*.0005;
        p.point = shared*p.point; p.wall_normal = shared.linear()*p.wall_normal;
      }
      auto &destination = split == 0 ? fitting : held; destination.insert(destination.end(),points.begin(),points.end());
      auto &boundary_destination = split == 0 ? edges : held_edges; boundary_destination.insert(boundary_destination.end(),boundaries_.begin(),boundaries_.end());
    }
    for (const std::string method : {"prior","unweighted","robust"}) {
      auto config = equivalent_profile(); config.estimate_geometry = true; config.camera_noise_sigma_m = {.003,.004};
      if (method == "unweighted") { config.camera_noise_sigma_m = {.003,.003}; config.huber_sigma = 1e12; }
      const auto started = std::chrono::steady_clock::now();
      std::cout << seed << ',' << population << ',' << method << ',';
      if (method == "prior") {
        std::cout << "PRIOR," << (model().prior.translation()-truth.translation()).norm() << ','
          << Eigen::AngleAxisd(model().prior.linear()*truth.linear().transpose()).angle() << ",.4,.6,,,,,0,\n";
        continue;
      }
      try {
        const auto r = estimate(model(),fitting,held,config,[]{return false;},edges,held_edges);
        std::cout << "OK," << (r.pose.translation()-truth.translation()).norm() << ','
          << Eigen::AngleAxisd(r.pose.linear()*truth.linear().transpose()).angle() << ',' << r.dimensions.y() << ',' << r.dimensions.x() << ','
          << std::sqrt(r.joint_covariance(6,6)) << ',' << std::sqrt(r.joint_covariance(7,7)) << ',' << r.fixed_boundary_rmse_m << ',' << r.held_out_boundary_rmse_m << ','
          << std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count() << ',';
        for (int row=0; row<8; ++row) for (int column=0; column<8; ++column) { if (row || column) std::cout << ';'; std::cout << r.joint_covariance(row,column); }
        std::cout << '\n';
      } catch (const Rejection &error) {
        std::cout << error.code << ",,,,,,,,," << std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count() << ",\n";
      }
    }
  }
}
