#pragma once
#include "fixtures.hpp"
namespace geometry_fixture {
using namespace tote_fusion;
using namespace tote_fixture;
inline std::vector<Observation> surfaces(const Pose &pose,uint32_t frame,const Vec3 &dimensions) {
  auto points = samples(Pose::Identity(),frame);
  for (auto &p : points) {
    if (std::abs(p.point.x()+.3) < 1e-9) p.point.x() = -dimensions.x()/2;
    if (std::abs(p.point.y()+.2) < 1e-9) p.point.y() = -dimensions.y()/2;
    p.point = pose*p.point; p.origin = pose*p.origin;
  }
  return points;
}
inline std::vector<BoundaryPoint> boundaries(const Pose &pose,uint32_t frame,uint32_t axis,const Vec3 &dimensions,uint32_t camera=0) {
  std::vector<BoundaryPoint> result;
  for (int side : {-1,1}) for (int i=0; i<25; ++i) {
    Vec3 p = Vec3::Zero(); p[axis] = side*dimensions[axis]/2; p[1-axis] = -dimensions[1-axis]/2; p.z() = .025+i*.006;
    result.push_back({pose*p,frame,camera,axis,side,-1,.002,pose.linear()*(-Vec3::Unit(1-axis))});
  }
  return result;
}
}
