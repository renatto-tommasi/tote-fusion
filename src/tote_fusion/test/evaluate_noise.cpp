// Test-only independent analytic truth and seeded sensor faults. Never linked
// into the estimator. CSV preserves every trial, including explicit rejections.
#include "fixtures.hpp"
#include <chrono>
#include <iostream>
#include <iomanip>
#include <random>
using namespace tote_fusion;
using namespace tote_fixture;

std::vector<Observation> noisy(uint64_t seed, bool held, bool outliers, uint32_t cameras) {
  std::mt19937_64 random(seed);
  std::normal_distribution<double> gaussian(0,1);
  Vec6 shared;
  for (int axis=0; axis<6; ++axis) shared[axis] = gaussian(random)*(axis<3 ? 0.001 : 0.0034906585);
  const Pose calibration=exp(shared);
  random.seed(seed*2+static_cast<uint64_t>(held)+5000);
  std::vector<Observation> result;
  // Independent fit/hold frame noise; the same run-level calibration bias is
  // deliberately shared. Repeated frames cannot validate that common bias.
  for (uint32_t frame=0; frame<6; ++frame) {
    for (uint32_t camera=0;camera<cameras;++camera) {
    const double frame_bias=gaussian(random)*0.00075;
    auto points=samples(actual_pose(),frame+(held ? 6 : 0));
    std::mt19937_64 camera_random(seed+camera*10000+90000);
    Vec6 camera_bias=Vec6::Zero();
    if (cameras>1) for (int axis=0;axis<6;++axis) camera_bias[axis]=gaussian(camera_random)*(axis<3 ? .0005 : .00174532925);
    const Pose sensor_bias=exp(camera_bias);
    for (std::size_t i=0; i<points.size(); ++i) {
      auto sample=points[i]; sample.camera=camera;
      if (cameras>1) sample.origin=actual_pose()*Vec3(-.9-.08*camera,-.9+.06*camera,1.5-.03*camera);
      const double noise=gaussian(random)*(.0015+.0005*camera)+frame_bias+(outliers && i%20==0 ? 0.02 : 0);
      if ((i/3)%2) continue;
      sample.point += noise*(sample.point-sample.origin).normalized();
      sample.point=calibration*sensor_bias*sample.point; sample.origin=calibration*sensor_bias*sample.origin;
      result.push_back(sample);
    }
    }
  }
  return result;
}
int main(int argc, char **argv) {
  const uint32_t cameras=argc==2 ? std::stoul(argv[1]) : 1;
  require(cameras>=1 && cameras<=3,"INVALID_CONFIG","Noise evaluation camera count");
  std::cout << std::setprecision(17) << "seed,partition,prior_offset,outliers,method,code,translation_m,rotation_rad,held_rmse_m,fit_points,hold_points,wall_s,nees";
  for (int i=0;i<6;++i) std::cout << ",error" << i;
  for (int i=0;i<36;++i) std::cout << ",cov" << i;
  std::cout << '\n';
  for (uint64_t seed=1000; seed<1140; ++seed) {
    const bool outliers=seed%2;
    const auto fitting=noisy(seed,false,outliers,cameras), held=noisy(seed,true,outliers,cameras);
    auto value=model();
    if (seed>=1120) value.prior=actual_pose();
    for (const std::string method : {"prior","unweighted","robust"}) {
      const auto started=std::chrono::steady_clock::now();
      Pose pose=value.prior; Mat6 covariance=Mat6::Identity();
      double rmse=0; uint32_t fp=0,hp=0; std::string code="OK";
      try {
        if (method!="prior") {
          auto config=equivalent_profile();
          if (cameras>1) {
            config.camera_noise_sigma_m.clear();
            for (uint32_t camera=0;camera<cameras;++camera) config.camera_noise_sigma_m.push_back(.003+.001*camera);
          }
          if (method=="unweighted") {
            config.huber_sigma=1e12;
            config.camera_noise_sigma_m.assign(cameras,.003); // Equal weights, same camera/frame budgets and prior.
          }
          const auto fit=estimate(value,fitting,held,config,[] { return false; });
          pose=fit.pose; covariance=fit.covariance; rmse=fit.held_out_rmse_m;
          fp=fit.fitting_points; hp=fit.held_out_points;
        } else {
          // Independent truth-surface distance for unchanged prior baseline.
          // All methods' pose errors below use independent fixture truth.
          const auto inv=pose.inverse();
          for (const auto &p : held) {
            const auto hit=raycast(value,inv*p.origin,(inv*p.point-inv*p.origin).normalized());
            if (hit) { rmse+=std::pow(hit->normal.dot(inv*p.point-hit->point),2); ++hp; }
          }
          assert(hp>0); rmse=std::sqrt(rmse/hp);
        }
      } catch (const Rejection &error) { code=error.code; }
      Vec6 error; error << pose.translation()-actual_pose().translation(),
        log(pose*actual_pose().inverse()).tail<3>();
      const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
      std::cout << seed << ',' << (seed<1060 ? "calibration" : "validation") << ',' << (seed<1120) << ',' << outliers << ',' << method << ',' << code
        << ',' << error.head<3>().norm() << ',' << error.tail<3>().norm() << ',' << rmse << ',' << fp << ',' << hp
        << ',' << elapsed << ',' << error.dot(covariance.ldlt().solve(error));
      for (int i=0;i<6;++i) std::cout << ',' << error[i];
      for (int row=0;row<6;++row) for (int col=0;col<6;++col) std::cout << ',' << covariance(row,col);
      std::cout << std::endl;
    }
  }
}
