#pragma once
#include "tote_fusion/core.hpp"
#include <cmath>

namespace tote_fixture {
using namespace tote_fusion;
inline Model model() { return load_model(TOTE_MODEL_FILE); }
inline Pose actual_pose() {
  Pose result = Pose::Identity();
  result.linear() = Eigen::AngleAxisd(0.4363323129985824, Vec3::UnitZ()).toRotationMatrix();
  result.translation() = Vec3(1.02, 0, 0.5238);
  return result;
}
inline std::vector<Observation> samples(const Pose &truth, uint32_t frame, bool planar = false) {
  std::vector<Observation> result;
  const Vec3 origin = truth*Vec3(-0.9, -0.9, 1.5);
  // Independent analytic fixture, not points produced by registration/raycast.
  for (int i=0; i<12; ++i) for (int j=0; j<10; ++j) {
    std::vector<Vec3> points{Vec3(-0.12+i*0.02, -0.08+j*0.018, 0.018)};
    if (!planar) {
      points.emplace_back(-0.3, -0.16+i*0.028, 0.03+j*0.016);
      points.emplace_back(-0.24+i*0.04, -0.2, 0.03+j*0.016);
    }
    for (const auto &point : points) result.push_back({truth*point, origin, frame});
  }
  return result;
}
inline FitConfig equivalent_profile() { FitConfig config; config.allow_pi_symmetry = true; return config; }
}  // namespace tote_fixture
