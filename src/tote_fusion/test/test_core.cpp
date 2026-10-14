#include "fixtures.hpp"
#include <gtest/gtest.h>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>

using namespace tote_fusion;
using namespace tote_fixture;
namespace {
template<class Function> void rejects(const std::string &code, Function action) {
  try { action(); FAIL() << "Expected " << code; }
  catch (const Rejection &error) { EXPECT_EQ(error.code, code); }
}
}  // namespace

TEST(Model, ExactGeometryHashDimensionsAndBottomCentrePrior) {
  const auto value = model();
  EXPECT_EQ(value.body_frame, "base_link"); EXPECT_EQ(value.revision, 0u);
  EXPECT_TRUE(value.dimensions.isApprox(Vec3(0.6, 0.4, 0.23), 1e-12));
  EXPECT_TRUE(value.prior.translation().isApprox(Vec3(1.04, 0, 0.5238), 1e-12));
  EXPECT_EQ(value.boxes.size(), 17u);
  EXPECT_EQ(sha256("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}
TEST(Model, MissingModelFailsWithoutConstructingADefault) {
  rejects("INVALID_MODEL", [] { load_model("/nonexistent/tote_model.yaml"); });
}
TEST(Geometry, OpenFloorOuterFaceAndHandleAperture) {
  const auto value = model();
  const auto floor = raycast(value, Vec3(0,0,1), Vec3(0,0,-1));
  ASSERT_TRUE(floor); EXPECT_NEAR(floor->point.z(), 0.018, 1e-12); EXPECT_EQ(floor->normal.z(), 1);
  const auto wall = raycast(value, Vec3(-1,0,0.05), Vec3::UnitX());
  ASSERT_TRUE(wall); EXPECT_NEAR(wall->point.x(), -0.3, 1e-12); EXPECT_EQ(wall->normal.x(), -1);
  EXPECT_FALSE(raycast(value, Vec3(0,-1,0.204), Vec3::UnitY()));
  EXPECT_FALSE(raycast(value, Vec3(-1,0,0.10), Vec3::UnitX()));  // excluded raised-label band
}
TEST(Geometry, FirstOpaqueSurfaceWinsOverAnOccludedBackFace) {
  const auto hit = raycast(model(), Vec3(-1,0,0.14), Vec3::UnitX());
  ASSERT_TRUE(hit); EXPECT_NEAR(hit->point.x(), -0.3, 1e-12);
}
TEST(Projection, OffAxisAxialDepthUsesMetricUnitsAndImageRowStride) {
  std::vector<float> depth{1,1,99,2,2,99};
  std::vector<uint8_t> mask{0,0,99,0,255,99};
  ImageView d{reinterpret_cast<const uint8_t *>(depth.data()), 24, 12, 2, 2, "32FC1", false};
  ImageView m{mask.data(), 6, 3, 2, 2, "mono8", false};
  const auto points = project(d,m,{2,4,0,0},0.3,3,1,1000);
  ASSERT_EQ(points.size(),1u); EXPECT_TRUE(points[0].isApprox(Vec3(1,0.5,2),1e-12));
  std::vector<uint16_t> millimetres{1000,1000,99,2000,2000,99};
  d = {reinterpret_cast<const uint8_t *>(millimetres.data()), 12, 6, 2, 2, "16UC1", false};
  EXPECT_TRUE(project(d,m,{2,4,0,0},0.3,3,1,1000)[0].isApprox(Vec3(1,0.5,2),1e-12));
}
TEST(Projection, InvalidEncodingsTruncationIntrinsicsAndMaskReject) {
  std::vector<float> depth(4,1); std::vector<uint8_t> mask(4,255);
  ImageView d{reinterpret_cast<const uint8_t *>(depth.data()),16,8,2,2,"32FC1",false};
  ImageView m{mask.data(),4,2,2,2,"mono8",false};
  rejects("INVALID_INTRINSICS", [&] { project(d,m,{0,1,0,0},0.3,3,1,1000); });
  d.bytes=8; rejects("INVALID_DEPTH", [&] { project(d,m,{1,1,0,0},0.3,3,1,1000); });
  d.bytes=16; d.encoding="radial_range";
  rejects("INVALID_DEPTH", [&] { project(d,m,{1,1,0,0},0.3,3,1,1000); });
  d.encoding="32FC1"; m.width=1;
  rejects("MASK_MISMATCH", [&] { project(d,m,{1,1,0,0},0.3,3,1,1000); });
}
TEST(Projection, InvalidMaskedDepthDoesNotBecomeGuessedPoints) {
  std::vector<float> depth{NAN,INFINITY,0,10}; std::vector<uint8_t> mask(4,255);
  ImageView d{reinterpret_cast<const uint8_t *>(depth.data()),16,8,2,2,"32FC1",false};
  ImageView m{mask.data(),4,2,2,2,"mono8",false};
  rejects("INVALID_DEPTH", [&] { project(d,m,{1,1,0,0},0.3,3,1,1000); });
}
TEST(Projection, DenseImageFailsAtTheProjectionBudget) {
  std::vector<float> depth(4,1); std::vector<uint8_t> mask(4,255);
  ImageView d{reinterpret_cast<const uint8_t *>(depth.data()),16,8,2,2,"32FC1",false};
  ImageView m{mask.data(),4,2,2,2,"mono8",false};
  rejects("BUFFER_LIMIT", [&] { project(d,m,{1,1,0,0},0.3,3,1,2); });
}
TEST(Frames, NonidentityTransformUsesBodyFromCameraDirection) {
  Pose pose = Pose::Identity();
  pose.linear() = Eigen::AngleAxisd(std::acos(-1)/2, Vec3::UnitZ()).toRotationMatrix();
  pose.translation() = Vec3(0.4,-0.2,0.7);
  const auto result = transform_points({Vec3(0.2,0.3,1.4)},pose);
  ASSERT_EQ(result.size(),1u); EXPECT_TRUE(result[0].isApprox(Vec3(0.1,0,2.1),1e-12));
  pose.linear()(0,0)=2;
  rejects("INVALID_TRANSFORM", [&] { transform_points({Vec3(1,2,3)},pose); });
}
TEST(Frames, StaleFutureAndPreviousRequestStampsReject) {
  validate_capture(1000,950,900,100,10);
  rejects("STALE_OBSERVATION", [] { validate_capture(1000,800,700,100,10); });
  rejects("STALE_OBSERVATION", [] { validate_capture(1000,1020,900,100,10); });
  rejects("STALE_OBSERVATION", [] { validate_capture(1000,950,980,100,10); });
}
TEST(Frames, Se3ExpLogRoundTripWithCoupledTranslationRotation) {
  Vec6 delta; delta << 0.12,-0.07,0.04,0.2,-0.1,0.3;
  EXPECT_TRUE(log(exp(delta)).isApprox(delta,1e-12));
  EXPECT_TRUE(exp(Vec6::Zero()).isApprox(Pose::Identity(),1e-12));
}
TEST(Registration, ExactPriorDoesNotDrift) {
  auto value=model(); value.prior=actual_pose();
  const auto result=estimate(value,samples(actual_pose(),0),samples(actual_pose(),1),equivalent_profile(),[] { return false; });
  EXPECT_LT((result.pose.translation()-actual_pose().translation()).norm(),1e-10);
  EXPECT_LT(result.held_out_rmse_m,1e-10);
  EXPECT_TRUE(result.covariance.allFinite());
}
TEST(Registration, SeparateTranslationAndYawBiasesImproveOnHeldOutFrames) {
  for (const int mode : {0,1,2}) {
    auto value=model(); value.prior=actual_pose();
    if (mode!=1) value.prior.translation().x()+=0.02;
    if (mode!=0) value.prior.linear()=Eigen::AngleAxisd(3*std::acos(-1)/180,Vec3::UnitZ()).toRotationMatrix()*value.prior.linear();
    const auto result=estimate(value,samples(actual_pose(),0),samples(actual_pose(),1),equivalent_profile(),[] { return false; });
    EXPECT_LT((result.pose.translation()-actual_pose().translation()).norm(),0.001);
    EXPECT_LT(Eigen::AngleAxisd(result.pose.linear()*actual_pose().linear().transpose()).angle(),0.002);
    EXPECT_LT(result.held_out_rmse_m,0.001);
    EXPECT_TRUE(value.dimensions.isApprox(Vec3(0.6,0.4,0.23),1e-12));
  }
}
TEST(Registration, PlanarDataCannotBecomeObservableThroughThePrior) {
  auto value=model(); value.prior=actual_pose();
  rejects("UNOBSERVABLE", [&] { estimate(value,samples(actual_pose(),0,true),samples(actual_pose(),1,true),equivalent_profile(),[] { return false; }); });
}
TEST(Registration, StrictProfileRejectsGlobalPiYawAmbiguity) {
  auto value=model(); value.prior=actual_pose();
  rejects("AMBIGUOUS_SYMMETRY", [&] { estimate(value,samples(actual_pose(),0),samples(actual_pose(),1),FitConfig{},[] { return false; }); });
}
TEST(Registration, CorrelatedPixelsCannotBeSharedWithHoldOut) {
  auto value=model(); value.prior=actual_pose();
  rejects("INVALID_HOLDOUT", [&] { estimate(value,samples(actual_pose(),0),samples(actual_pose(),0),equivalent_profile(),[] { return false; }); });
}
TEST(Registration, CancellationProducesNoEstimate) {
  rejects("CANCELED", [&] { estimate(model(),samples(actual_pose(),0),samples(actual_pose(),1),equivalent_profile(),[] { return true; }); });
}
TEST(Registration, CancellationInterruptsAnInProgressBatch) {
  int checks=0;
  rejects("CANCELED", [&] { estimate(model(),samples(actual_pose(),0),samples(actual_pose(),1),equivalent_profile(),
    [&] { return ++checks >= 3; }); });
  EXPECT_EQ(checks,3);
}
TEST(Registration, SparseInputsRejectWithoutReusingAPriorCandidate) {
  rejects("INSUFFICIENT_DATA", [&] { estimate(model(),{},samples(actual_pose(),1),equivalent_profile(),[] { return false; }); });
}

TEST(Registration, EquivalentHypothesisIsExplicitAndIndependentlyFitted) {
  const auto result=estimate(model(),samples(actual_pose(),0),samples(actual_pose(),1),equivalent_profile(),[] { return false; });
  EXPECT_LT((result.pi_equivalent_pose.translation()-result.pose.translation()).norm(),1e-6);
  EXPECT_NEAR(Eigen::AngleAxisd(result.pose.linear().transpose()*result.pi_equivalent_pose.linear()).angle(),std::acos(-1),1e-6);
  EXPECT_GT(result.iterations,1u);
}
TEST(Registration, LargeSegmentationContaminationRejects) {
  auto fit=samples(actual_pose(),0), hold=samples(actual_pose(),1);
  for (auto *set : {&fit,&hold}) for (std::size_t i=0; i<set->size(); ++i)
    if (i%2==0) (*set)[i].point += Vec3(0.02,0.02,0.02);
  rejects("POOR_FIT", [&] { estimate(model(),fit,hold,equivalent_profile(),[] { return false; }); });
}

TEST(Registration, DuplicatedPixelsDoNotMultiplyConfidence) {
  auto fit=samples(actual_pose(),0), held=samples(actual_pose(),1);
  const auto single=estimate(model(),fit,held,equivalent_profile(),[] { return false; });
  const auto original=fit;
  for (int copy=0;copy<4;++copy) fit.insert(fit.end(),original.begin(),original.end());
  const auto repeated=estimate(model(),fit,held,equivalent_profile(),[] { return false; });
  EXPECT_TRUE(repeated.pose.isApprox(single.pose,1e-8));
  EXPECT_TRUE(repeated.covariance.isApprox(single.covariance,1e-8));
  EXPECT_TRUE(repeated.covariance.isApprox(repeated.covariance.transpose(),1e-12));
  EXPECT_EQ(Eigen::LLT<Mat6>(repeated.covariance).info(),Eigen::Success);
}

namespace {
std::vector<Observation> camera_samples(uint32_t camera, uint32_t frame, double z_bias=0, bool planar=false) {
  auto data=samples(actual_pose(),frame,planar);
  for (auto &point : data) { point.camera=camera; point.point.z()+=z_bias; point.origin.z()+=z_bias; }
  return data;
}
std::vector<Observation> join(std::vector<Observation> a, const std::vector<Observation> &b) {
  a.insert(a.end(),b.begin(),b.end()); return a;
}
FitConfig multi_profile() { auto config=equivalent_profile(); config.camera_noise_sigma_m={.003,.003,.003}; return config; }
}
TEST(Multiview, PerCameraNoiseChangesInfluenceWithoutChangingGeometry) {
  auto value=model(); value.prior=actual_pose(); auto config=multi_profile();
  config.camera_noise_sigma_m={.002,.002};
  const auto fit=join(camera_samples(0,0),camera_samples(1,0,.001));
  const auto held=join(camera_samples(0,1),camera_samples(1,1,.001));
  const auto equal=estimate(value,fit,held,config,[] { return false; });
  config.camera_noise_sigma_m[1]=.01;
  const auto weighted=estimate(value,fit,held,config,[] { return false; });
  EXPECT_GT(equal.pose.translation().z()-actual_pose().translation().z(),.00035);
  EXPECT_LT(std::abs(weighted.pose.translation().z()-actual_pose().translation().z()),.0001);
  EXPECT_EQ(weighted.camera_held_out_rmse_m.size(),2u);
  EXPECT_TRUE(value.dimensions.isApprox(Vec3(.6,.4,.23),1e-12));
}
TEST(Multiview, ExtraIdenticalCameraFramesCannotDominateOrMultiplyConfidence) {
  auto fit=join(camera_samples(0,0),camera_samples(1,0,.001));
  const auto held=join(camera_samples(0,1),camera_samples(1,1,.001));
  const auto single=estimate(model(),fit,held,multi_profile(),[] { return false; });
  for (uint32_t frame=2;frame<10;++frame) fit=join(std::move(fit),camera_samples(0,frame));
  const auto repeated=estimate(model(),fit,held,multi_profile(),[] { return false; });
  EXPECT_TRUE(repeated.pose.isApprox(single.pose,1e-8));
  EXPECT_TRUE(repeated.covariance.isApprox(single.covariance,1e-8));
}
TEST(Multiview, ConflictingViewsCannotBecomeASuccessfulAverage) {
  auto value=model(); value.prior=actual_pose();
  const auto fit=join(camera_samples(0,0),camera_samples(1,0,.008));
  const auto held=join(camera_samples(0,1),camera_samples(1,1,.008));
  rejects("CAMERA_CONFLICT", [&] { estimate(value,fit,held,multi_profile(),[] { return false; }); });
}
TEST(Multiview, RotationalViewDisagreementRejects) {
  auto value=model(); value.prior=actual_pose();
  auto fit=camera_samples(1,0), held=camera_samples(1,1);
  Pose disagreement=Pose::Identity();
  disagreement.linear()=Eigen::AngleAxisd(.05235987756,Vec3::UnitZ()).toRotationMatrix();
  disagreement.translation()=actual_pose().translation()-disagreement.linear()*actual_pose().translation();
  for (auto *set : {&fit,&held}) for (auto &sample : *set) {
    sample.point=disagreement*sample.point; sample.origin=disagreement*sample.origin;
  }
  try {
    estimate(value,join(camera_samples(0,0),fit),join(camera_samples(0,1),held),multi_profile(),[] { return false; });
    FAIL() << "Rotational disagreement must reject";
  } catch (const Rejection &error) {
    EXPECT_TRUE(error.code=="CAMERA_CONFLICT" || error.code=="POOR_FIT") << error.code;
  }
}
TEST(Multiview, DroppedCameraDoesNotCreateObservedDegreesOfFreedom) {
  auto value=model(); value.prior=actual_pose();
  rejects("UNOBSERVABLE", [&] { estimate(value,camera_samples(1,0,0,true),camera_samples(1,1,0,true),multi_profile(),[] { return false; }); });
}
TEST(Multiview, ReceivedCameraWithoutModelSupportCannotDisappear) {
  auto fitting=camera_samples(0,0), held=camera_samples(0,1);
  for (auto *set : {&fitting,&held}) for (int i=0;i<40;++i) {
    set->push_back({actual_pose()*Vec3(2,2,2),actual_pose()*Vec3(2,2,3),set==&fitting ? 0u : 1u,1});
  }
  // The valid camera alone passes global support/inlier gates. The received
  // second camera must reject instead of claiming zero residual and success.
  rejects("INSUFFICIENT_DATA", [&] { estimate(model(),fitting,held,multi_profile(),[] { return false; }); });
}
TEST(Multiview, CommonCalibrationBiasRemainsInvisibleToResiduals) {
  auto value=model(); value.prior=actual_pose();
  const auto fit=join(camera_samples(0,0,.008),camera_samples(1,0,.008));
  const auto held=join(camera_samples(0,1,.008),camera_samples(1,1,.008));
  const auto result=estimate(value,fit,held,multi_profile(),[] { return false; });
  EXPECT_LT(result.held_out_rmse_m,.0001);
  EXPECT_GT((result.pose.translation()-actual_pose().translation()).norm(),.007);
  // This is an evaluator-only limitation oracle, not permission to hide bias.
}
TEST(Multiview, InvalidCameraAndNoiseProfilesReject) {
  auto config=multi_profile(); config.camera_noise_sigma_m[2]=NAN;
  rejects("INVALID_CONFIG", [&] { estimate(model(),camera_samples(0,0),camera_samples(0,1),config,[] { return false; }); });
  rejects("INVALID_CAMERA", [&] { estimate(model(),camera_samples(3,0),camera_samples(3,1),multi_profile(),[] { return false; }); });
}

TEST(Multiview, LostCameraCannotMakeCovarianceMorePrecise) {
  auto value=model(); value.prior=actual_pose();
  const auto two_fit=join(camera_samples(0,0),camera_samples(1,0));
  const auto two_hold=join(camera_samples(0,1),camera_samples(1,1));
  const auto dropped=estimate(value,two_fit,two_hold,multi_profile(),[] { return false; });
  const auto full=estimate(value,join(two_fit,camera_samples(2,0)),join(two_hold,camera_samples(2,1)),multi_profile(),[] { return false; });
  const Eigen::SelfAdjointEigenSolver<Mat6> difference(dropped.covariance-full.covariance);
  ASSERT_EQ(difference.info(),Eigen::Success);
  EXPECT_GE(difference.eigenvalues().minCoeff(),-1e-12);
  EXPECT_GT(difference.eigenvalues().sum(),0);
}

TEST(Multiview, FiniteButUnrepresentableExternalRaysAndSigmasRejectBeforeAssertions) {
  auto fit=camera_samples(0,0); fit[0].point=Vec3(1e200,1e200,1e200);
  rejects("INVALID_DEPTH", [&] { estimate(model(),fit,camera_samples(0,1),multi_profile(),[] { return false; }); });
  auto config=multi_profile(); config.camera_noise_sigma_m[1]=1e-200;
  rejects("INVALID_CONFIG", [&] { estimate(model(),camera_samples(0,0),camera_samples(0,1),config,[] { return false; }); });
}
