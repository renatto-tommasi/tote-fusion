#include "geometry_fixture.hpp"
#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>
using namespace tote_fusion;
using namespace tote_fixture;
using namespace geometry_fixture;
namespace {
Estimate fit(const Vec3 &dimensions,bool both=true) {
  auto c = equivalent_profile(); c.estimate_geometry = true;
  auto a = boundaries(actual_pose(),0,1,dimensions), b = boundaries(actual_pose(),1,1,dimensions);
  if (both) { auto aa = boundaries(actual_pose(),0,0,dimensions), bb = boundaries(actual_pose(),1,0,dimensions); a.insert(a.end(),aa.begin(),aa.end()); b.insert(b.end(),bb.begin(),bb.end()); }
  return estimate(model(),surfaces(actual_pose(),0,dimensions),surfaces(actual_pose(),1,dimensions),c,[]{return false;},a,b);
}
}
TEST(Geometry,WidthSeparatesRigidPoseFromPhysicalSize) {
  const auto result = fit(Vec3(.6,.42,.23),false);
  ASSERT_EQ(result.dimension_axes,std::vector<uint32_t>{1});
  EXPECT_NEAR(result.dimensions.y(),.42,.0002);
  EXPECT_GT(result.fixed_boundary_rmse_m,.008);
  EXPECT_LT(result.held_out_boundary_rmse_m,.0002);
  EXPECT_LT((result.pose.translation()-actual_pose().translation()).norm(),.001);
  EXPECT_NEAR(model().dimensions.y(),.4,1e-12);
  EXPECT_EQ(model().revision,0u);
}
TEST(Geometry,LengthAndWidthHaveJointCovarianceAndObservedRank) {
  const auto r = fit(Vec3(.62,.42,.23));
  EXPECT_NEAR(r.dimensions.x(),.62,.0002); EXPECT_NEAR(r.dimensions.y(),.42,.0002);
  ASSERT_EQ(r.joint_covariance.rows(),8);
  EXPECT_TRUE(r.joint_covariance.isApprox(r.joint_covariance.transpose(),1e-12));
  EXPECT_GT(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(r.joint_covariance).eigenvalues()[0],0);
  EXPECT_GT(r.joint_covariance.topRightCorner(6,2).norm(),1e-9);
  EXPECT_GT(r.joint_information_eigenvalues[0],1e-5*r.joint_information_eigenvalues[7]);
}
TEST(Geometry,SingleWallCanMeasureWidthButCannotEstablishFullPose) {
  auto c = equivalent_profile(); c.estimate_geometry = true;
  auto a = boundaries(actual_pose(),0,1,Vec3(.6,.42,.23)), b = boundaries(actual_pose(),1,1,Vec3(.6,.42,.23));
  ASSERT_EQ(measure_dimensions(a).size(),1u); EXPECT_NEAR(measure_dimensions(a)[0].value,.42,1e-12);
  auto train = surfaces(actual_pose(),0,Vec3(.6,.42,.23)), held = surfaces(actual_pose(),1,Vec3(.6,.42,.23));
  const auto wall = [](const Observation &p) { return std::abs((actual_pose().inverse()*p.point).x()+.3) > 1e-9; };
  train.erase(std::remove_if(train.begin(),train.end(),wall),train.end()); held.erase(std::remove_if(held.begin(),held.end(),wall),held.end());
  try { estimate(model(),train,held,c,[]{return false;},a,b); FAIL() << "Planar joint pose accepted"; }
  catch (const Rejection &e) { EXPECT_EQ(e.code,"UNOBSERVABLE"); }
}
TEST(Geometry,PatchOrOneEdgeCannotInventDimensions) {
  auto c = equivalent_profile(); c.estimate_geometry = true;
  auto a = boundaries(actual_pose(),0,1,Vec3(.6,.42,.23)), b = boundaries(actual_pose(),1,1,Vec3(.6,.42,.23));
  a.erase(a.begin()+25,a.end()); b.erase(b.begin()+25,b.end());
  EXPECT_TRUE(measure_dimensions(a).empty());
  try { estimate(model(),samples(actual_pose(),0),samples(actual_pose(),1),c,[]{return false;},a,b); FAIL(); }
  catch (const Rejection &e) { EXPECT_EQ(e.code,"INSUFFICIENT_GEOMETRY"); }
}
TEST(Geometry,Camera420Versus405CannotBeAveraged) {
  auto a = boundaries(actual_pose(),0,1,Vec3(.6,.42,.23)), b = boundaries(actual_pose(),0,1,Vec3(.6,.405,.23),1);
  a.insert(a.end(),b.begin(),b.end());
  try { measure_dimensions(a); FAIL() << "Conflicting widths averaged"; }
  catch (const Rejection &e) { EXPECT_EQ(e.code,"GEOMETRY_CONFLICT"); }
}
TEST(Geometry,SharedRigidCalibrationBiasPreservesWidthButCanBiasPose) {
  Pose bias = Pose::Identity(); bias.translation() = Vec3(.008,-.004,.003);
  bias.linear() = Eigen::AngleAxisd(.01,Vec3::UnitZ()).toRotationMatrix();
  const auto measured = measure_dimensions(boundaries(bias*actual_pose(),0,1,Vec3(.6,.42,.23)));
  ASSERT_EQ(measured.size(),1u); EXPECT_NEAR(measured[0].value,.42,1e-12);
  auto c = equivalent_profile(); c.estimate_geometry = true;
  const Pose pose = bias*actual_pose(); const Vec3 dims(.6,.42,.23);
  const auto r = estimate(model(),surfaces(pose,0,dims),surfaces(pose,1,dims),c,[]{return false;},boundaries(pose,0,1,Vec3(.6,.42,.23)),boundaries(pose,1,1,Vec3(.6,.42,.23)));
  EXPECT_NEAR(r.dimensions.y(),.42,.0003); EXPECT_GT((r.pose.translation()-actual_pose().translation()).norm(),.005);
}
TEST(Geometry,CommonLateralScaleBiasIsNotOnlineCalibration) {
  const auto m = measure_dimensions(boundaries(actual_pose(),0,1,Vec3(.6,.4*1.05,.23)));
  ASSERT_EQ(m.size(),1u); EXPECT_NEAR(m[0].value,.42,1e-12);
  // Same observations as a genuinely wider tote: sensor-only size estimation
  // cannot tell which caused them. This explicitly limits confidence claims.
  EXPECT_GT(std::abs(m[0].value-.4),.019);
}
TEST(Geometry,ParametricBoxesPreserveThicknessHandlesAndUnseenZ) {
  const auto n = model(), grown = parametric_model(n,Vec3(.62,.42,.23)), shrunk = parametric_model(n,Vec3(.58,.38,.23));
  for (const auto *m : {&grown,&shrunk}) for (std::size_t i=0; i<n.boxes.size(); ++i) {
    EXPECT_EQ(m->boxes[i].low.z(),n.boxes[i].low.z()); EXPECT_EQ(m->boxes[i].high.z(),n.boxes[i].high.z());
    for (int a=0; a<2; ++a) {
      if (std::abs(n.boxes[i].high[a]-n.boxes[i].low[a]-.0025) < 1e-9) { EXPECT_NEAR(m->boxes[i].high[a]-m->boxes[i].low[a],.0025,1e-12); }
    }
  }
  EXPECT_DOUBLE_EQ(grown.boxes[2].high.x(),-.055); EXPECT_DOUBLE_EQ(grown.boxes[3].low.x(),.055);
  EXPECT_EQ(n.hash,grown.hash); EXPECT_EQ(n.revision,grown.revision);
  EXPECT_THROW(parametric_model(n,Vec3(.62,.42,.24)),Rejection);
}
namespace {
struct WallImage {
  uint32_t w=640,h=480; std::vector<float> z; std::vector<uint8_t> mask;
  Intrinsics k{600,600,319.5,239.5}; Pose camera;
  WallImage() : z(w*h,3.f),mask(w*h,0),camera(actual_pose()) {
    Pose local = Pose::Identity();
    local.linear().col(0) = -Vec3::UnitY(); local.linear().col(1) = -Vec3::UnitZ(); local.linear().col(2) = Vec3::UnitX();
    local.translation() = Vec3(-.9,0,.12); camera = actual_pose()*local;
    // Independent plane scene: physical width 420 mm; no model raycaster.
    for (uint32_t v=0; v<h; ++v) for (uint32_t u=0; u<w; ++u) {
      const double y = -(u-k.cx)*.6/k.fx, height = .12-(v-k.cy)*.6/k.fy;
      if (std::abs(y) <= .21 && height >= .025 && height <= .18) { z[v*w+u] = .6f; mask[v*w+u] = 255; }
    }
  }
  std::vector<BoundaryPoint> edges() const {
    return image_boundaries({reinterpret_cast<const uint8_t*>(z.data()),z.size()*4,w*4,w,h,"32FC1",false},
      {mask.data(),mask.size(),w,w,h,"mono8",false},k,camera,model(),0,0,.003,2000);
  }
};
}
TEST(GeometryImages,PositiveBackgroundAndPairedLinesEstablishWidth) {
  WallImage image; const auto edges = image.edges(); ASSERT_GE(edges.size(),24u);
  const auto measured = measure_dimensions(edges); ASSERT_EQ(measured.size(),1u);
  EXPECT_EQ(measured[0].axis,1u); EXPECT_NEAR(measured[0].value,.42,.0001);
}
TEST(GeometryImages,MaskCropWithoutFreeSpaceIsNotAnEdge) {
  WallImage image;
  for (uint32_t v=0; v<image.h; ++v) for (uint32_t u=0; u<image.w; ++u)
    if (std::abs((u-image.k.cx)*.6/image.k.fx) > .18) image.mask[v*image.w+u] = 0;
  EXPECT_TRUE(image.edges().empty());
}
TEST(GeometryImages,InvalidBackgroundOrCloserOcclusionDoesNotEstablishWidth) {
  for (float background : {0.f,.5f}) {
    WallImage image; for (std::size_t i=0; i<image.z.size(); ++i) if (!image.mask[i]) image.z[i] = background;
    EXPECT_TRUE(image.edges().empty()) << background;
  }
}
TEST(GeometryImages,ImageBorderIsNotAVisibleEndpoint) {
  WallImage image;
  // Width exceeds the left image boundary. One genuine visible edge remains.
  for (uint32_t v=180; v<334; ++v) for (uint32_t u=0; u<110; ++u) { image.z[v*image.w+u] = .6f; image.mask[v*image.w+u] = 255; }
  EXPECT_TRUE(image.edges().empty());
}
namespace {
Estimate unequal_camera_geometry(bool biased) {
  const Vec3 dims(.6,.42,.23); auto config = equivalent_profile(); config.estimate_geometry = true;
  config.camera_noise_sigma_m = {.003,.03};
  std::vector<Observation> fitting,held; std::vector<BoundaryPoint> edges,held_edges;
  for (uint32_t frame=0; frame<2; ++frame) {
    auto &destination = frame == 0 ? fitting : held;
    auto &boundary_destination = frame == 0 ? edges : held_edges;
    for (uint32_t camera=0; camera<2; ++camera) {
      const Vec3 shift = actual_pose().linear()*Vec3(0,biased && camera == 1 ? .012 : 0,0);
      auto points = surfaces(actual_pose(),frame,dims);
      for (auto &p : points) {
        if (std::abs((actual_pose().inverse()*p.point).y()+.21) < 1e-9) continue;
        p.camera = camera; p.point += shift; p.origin += shift; destination.push_back(p);
      }
      auto boundary = boundaries(actual_pose(),frame,1,dims,camera);
      for (auto &p : boundary) { p.point += shift; if (camera == 1) p.sigma = .02; }
      for (int repeat=0; repeat<(camera == 0 ? 20 : 1); ++repeat)
        boundary_destination.insert(boundary_destination.end(),boundary.begin(),boundary.end());
    }
  }
  return estimate(model(),fitting,held,config,[]{return false;},edges,held_edges);
}
}
TEST(Geometry,BoundaryCameraPlacementCannotHideBehindDensityAndSigma) {
  try {
    const auto result = unequal_camera_geometry(true);
    FAIL() << "Biased endpoint camera accepted; global held boundary RMSE=" << result.held_out_boundary_rmse_m;
  } catch (const Rejection &e) { EXPECT_EQ(e.code,"CAMERA_CONFLICT"); }
}
TEST(Geometry,ValidUnequalCameraDensityAndSigmaStillPass) {
  const auto result = unequal_camera_geometry(false);
  EXPECT_NEAR(result.dimensions.y(),.42,.0003); EXPECT_LT(result.held_out_boundary_rmse_m,.001);
}
