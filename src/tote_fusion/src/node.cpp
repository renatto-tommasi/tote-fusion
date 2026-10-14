#include "tote_fusion/core.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2_ros/qos.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tote_fusion_interfaces/action/estimate_tote.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <atomic>
#include <cassert>
#include <chrono>
#include <future>
#include <map>
#include <mutex>
#include <set>
#include <sstream>

namespace tote_fusion {
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
using Action = tote_fusion_interfaces::action::EstimateTote;
using Goal = rclcpp_action::ServerGoalHandle<Action>;
using Candidate = tote_fusion_interfaces::msg::TotePoseCandidate;
using Steady = std::chrono::steady_clock;
using Transform = geometry_msgs::msg::TransformStamped;
using TfMessage = tf2_msgs::msg::TFMessage;
using namespace std::chrono_literals;

class FusionNode : public rclcpp::Node {
  using Key = std::pair<uint32_t,int64_t>;
public:
  FusionNode() : Node("tote_fusion"), model_(load_model(parameter<std::string>("model_file", ""))) {
    require(get_parameter("use_sim_time").as_bool(), "INVALID_CONFIG", "M1 simulation profile requires use_sim_time=true");
    const auto ids = parameter<std::vector<std::string>>("camera_ids", {"head"});
    require(!ids.empty() && ids.size() <= 3 && std::set<std::string>(ids.begin(),ids.end()).size() == ids.size(),
      "INVALID_CONFIG", "Need one to three distinct configured cameras");
    const auto camera_file = parameter<std::string>("camera_file",
      (std::filesystem::path(get_parameter("model_file").as_string()).parent_path()/"cameras.yaml").string());
    const auto catalog = YAML::LoadFile(camera_file);
    require(catalog["schema_version"].as<int>() == 1 && catalog["cameras"].IsMap(),
      "INVALID_CONFIG", "Unsupported camera catalog");
    profile_ = parameter<std::string>("validation_profile", "head_sim_strict");
    geometry_ = profile_ == "geometry_sim_strict" || profile_ == "geometry_sim_pi_equivalent";
    fit_.estimate_geometry = geometry_;
    multi_ = profile_ == "multi_sim_strict" || profile_ == "multi_sim_pi_equivalent";
    multi_ |= geometry_ && ids.size() > 1;
    require(geometry_ || (multi_ ? ids.size() >= 2 : (ids == std::vector<std::string>{"head"} &&
      (profile_ == "head_sim_strict" || profile_ == "head_sim_pi_equivalent"))),
      "INVALID_CONFIG", "Camera list and validation profile disagree");
    fit_.allow_pi_symmetry = profile_ == "geometry_sim_pi_equivalent" || profile_ == "head_sim_pi_equivalent" || profile_ == "multi_sim_pi_equivalent";
    fit_.camera_noise_sigma_m.clear();
    std::set<std::string> topics, optical_frames;
    for (const auto &id : ids) {
      require(catalog["cameras"][id].IsMap(), "INVALID_CONFIG", "Camera absent from immutable catalog: " + id);
      const auto config = catalog["cameras"][id];
      Camera camera; camera.id = id;
      const std::string prefix = id == "head" ? "" : id + "."; // Preserve the existing head topic parameters.
      const auto field = [&](const std::string &name) { return parameter<std::string>(prefix+name,config[name].as<std::string>()); };
      camera.optical = field("optical_frame");
      camera.depth_topic = field("depth_topic"); camera.mask_topic = field("mask_topic"); camera.info_topic = field("camera_info_topic");
      require(!camera.optical.empty() && optical_frames.insert(camera.optical).second,
        "INVALID_CONFIG", "Cameras must have distinct nonempty optical frames");
      for (const auto &topic : {camera.depth_topic,camera.mask_topic,camera.info_topic})
        require(!topic.empty() && topic.front() == '/' && topics.insert(topic).second,
          "INVALID_CONFIG", "Camera streams must have distinct absolute topics");
      const double sigma = parameter(prefix+"noise_sigma_m",config["noise_sigma_m"].as<double>());
      require(std::isfinite(sigma) && sigma >= 1e-6 && sigma <= .1, "INVALID_CONFIG", "Camera noise sigma must be within 1 micrometre..100 mm");
      fit_.camera_noise_sigma_m.push_back(sigma); cameras_.push_back(std::move(camera));
    }
    for (const auto &[name, value] : std::map<std::string, double *>{
      {"position_uncertainty_floor_m", &fit_.position_uncertainty_floor_m},
      {"camera_bias_limit_m", &fit_.camera_bias_limit_m}, {"huber_sigma", &fit_.huber_sigma},
      {"correspondence_limit_m", &fit_.correspondence_limit_m}, {"inlier_limit_m", &fit_.inlier_limit_m},
      {"maximum_rmse_m", &fit_.maximum_rmse_m}, {"minimum_inlier_fraction", &fit_.minimum_inlier_fraction},
      {"minimum_information_ratio", &fit_.minimum_information_ratio}, {"step_tolerance", &fit_.step_tolerance},
      {"rotation_uncertainty_floor_rad", &fit_.rotation_uncertainty_floor_rad}}) {
      *value = parameter(name, *value);
      require(std::isfinite(*value) && *value > 0, "INVALID_CONFIG", "Invalid fit parameter " + name);
    }
    fit_.maximum_iterations = count("maximum_iterations", fit_.maximum_iterations, 200);
    fit_.minimum_points = count("minimum_points", fit_.minimum_points, 10000);
    fit_.effective_points_per_frame = count("effective_points_per_frame", fit_.effective_points_per_frame, 1000);
    stride_ = count("pixel_stride", 8, 64);
    max_frames_ = count("max_frames", multi_ ? 120 : 40, 200);
    max_points_ = count("max_points", multi_ ? 60000 : 20000, 1000000);
    max_bytes_ = count("max_buffer_bytes", 67108864, 268435456);
    depth_min_ = parameter("depth_min_m", 0.3); depth_max_ = parameter("depth_max_m", 3.0);
    max_age_ = seconds("max_observation_age_s", 0.5);
    future_slop_ = seconds("future_tolerance_s", 0.05);
    capture_limit_ = seconds("max_capture_window_s", 2.0);
    expiry_ = seconds("candidate_validity_s", 0.5);
    tf_gap_ = seconds("max_tf_interpolation_gap_s", .1);
    tf_age_ = seconds("max_tf_support_age_s", .1);
    wall_limit_ = parameter("wall_timeout_s", 10.0);
    require(std::isfinite(depth_min_) && std::isfinite(depth_max_) && depth_min_ > 0 && depth_max_ > depth_min_ &&
      std::isfinite(wall_limit_) && wall_limit_ > 0 && wall_limit_ <= 60 && fit_.minimum_points >= 6 &&
      fit_.correspondence_limit_m > fit_.inlier_limit_m && fit_.minimum_inlier_fraction <= 1 &&
      fit_.minimum_information_ratio < 1, "INVALID_CONFIG", "Inconsistent capture/fit limits");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(50));
    if (!parameter("reliable_sensor_qos", false)) qos.best_effort();
    for (uint32_t index=0; index<cameras_.size(); ++index) {
      auto &camera = cameras_[index];
      camera.depth_sub = create_subscription<Image>(camera.depth_topic, qos,
        [this,index](Image::ConstSharedPtr message) { receive(index,message,&Pending::depth); });
      camera.mask_sub = create_subscription<Image>(camera.mask_topic, qos,
        [this,index](Image::ConstSharedPtr message) { receive(index,message,&Pending::mask); });
      camera.info_sub = create_subscription<Info>(camera.info_topic, qos,
        [this,index](Info::ConstSharedPtr message) { receive(index,message,&Pending::info); });
    }
    tf_dynamic_ = create_subscription<TfMessage>("/tf", tf2_ros::DynamicListenerQoS(),
      [this](TfMessage::ConstSharedPtr message) { receive_tf(*message,false); });
    tf_static_ = create_subscription<TfMessage>("/tf_static", tf2_ros::StaticListenerQoS(),
      [this](TfMessage::ConstSharedPtr message) { receive_tf(*message,true); });
    rcl_jump_threshold_t jump_threshold{};
    jump_threshold.on_clock_change = true;
    jump_threshold.min_backward.nanoseconds = -1;
    jump_handler_ = get_clock()->create_jump_callback([this]() noexcept {
      // Clock callbacks may run on the TimeSource thread while holding its
      // clock lock. Only signal; executor code owns history and request state.
      std::lock_guard lock(publication_mutex_);
      clock_reset_requested_ = true; stop_ = true;
    },nullptr,jump_threshold);
    for (const auto &[name, value] : get_node_parameters_interface()->get_parameter_overrides()) {
      (void)value;
      require(has_parameter(name), "INVALID_CONFIG", "Unknown parameter " + name);
    }
    candidates_ = create_publisher<Candidate>("/tote_fusion/candidate", 10);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    server_ = rclcpp_action::create_server<Action>(this, "/tote_fusion/estimate",
      [this](const auto &, std::shared_ptr<const Action::Goal> goal) {
        try {
          observe_clock(now().nanoseconds());
          require(!reserved_, "BUSY", "One bounded request at a time");
          require(!goal->object_id.empty() && goal->object_id.size() <= 128, "INVALID_GOAL", "Need a bounded object_id");
          require(goal->expected_model_revision == model_.revision, "STALE_MODEL_REVISION", "Expected model revision differs from immutable snapshot");
          require(goal->mode == (geometry_ ? Action::Goal::POSE_AND_GEOMETRY : Action::Goal::POSE_ONLY), "UNSUPPORTED_MODE", "Requested mode must match the separately configured profile");
          require(goal->min_distinct_views >= 1 && goal->min_distinct_views <= cameras_.size() && goal->validation_profile == profile_, "INVALID_GOAL", "Request must select this profile and a valid minimum camera count");
          require(goal->collection_duration.sec >= 0 && goal->collection_duration.nanosec < 1000000000,
            "INVALID_GOAL", "Invalid collection duration fields");
          const auto duration = rclcpp::Duration(goal->collection_duration).nanoseconds();
          require(duration >= 100000000 && duration <= capture_limit_, "INVALID_GOAL", "Collection duration is outside configured bounds");
          require(now().nanoseconds() > 0, "SIM_TIME_UNAVAILABLE", "Wait for simulation clock before triggering");
          reserved_ = true;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } catch (const Rejection &error) {
          diagnostic(error.code, error.what());
          return rclcpp_action::GoalResponse::REJECT;
        }
      },
      [this]([[maybe_unused]] const std::shared_ptr<Goal> goal) {
        assert(goal == goal_);
        stop_ = true;
        forced_error_ = "CANCELED";
        forced_message_ = "Request canceled by client";
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<Goal> goal) {
        assert(reserved_ && !goal_ && !worker_.valid());
        goal_ = std::move(goal); stop_ = false; forced_error_.clear(); forced_message_.clear(); used_tf_.clear();
        accepted_ = 0; point_count_ = 0; unmatched_ = 0; first_ = 0; last_ = 0; bytes_ = 0;
        for (auto &camera : cameras_) { camera.calibration.clear(); camera.accepted = 0; }
        pending_.clear(); completed_.clear(); outside_.clear(); fitting_.clear(); held_out_.clear(); boundary_fitting_.clear(); boundary_held_out_.clear(); boundary_count_ = 0; peak_bytes_ = 0; capture_stamps_.clear();
        started_ = now().nanoseconds(); end_ = started_ + rclcpp::Duration(goal_->get_goal()->collection_duration).nanoseconds();
        wall_started_ = Steady::now();
        feedback("COLLECTING");
      });
    timer_ = create_wall_timer(20ms, [this] { poll(); });
    parameter_lock_ = add_on_set_parameters_callback([](const auto &) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false; result.reason = "Estimator configuration is immutable; restart with a new snapshot/profile";
      return result;
    });
    diagnostic("IDLE", "Ready; immutable model revision " + std::to_string(model_.revision));
  }
  ~FusionNode() override { stop_ = true; if (worker_.valid()) worker_.wait(); }

private:
  struct TfEdge {
    std::string parent;
    bool fixed;
    // A conflicting timestamp remains explicitly invalid until it ages out.
    std::map<int64_t,std::optional<Transform>> samples;
  };
  struct Camera {
    std::string id, optical, depth_topic, mask_topic, info_topic, calibration;
    uint32_t accepted = 0;
    rclcpp::Subscription<Image>::SharedPtr depth_sub, mask_sub;
    rclcpp::Subscription<Info>::SharedPtr info_sub;
  };
  struct Pending { Image::ConstSharedPtr depth, mask; Info::ConstSharedPtr info; };
  template<class T> T parameter(const std::string &name, const T &value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor; descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  uint32_t count(const std::string &name, uint32_t value, uint32_t maximum) {
    const auto result = parameter<int64_t>(name, value);
    require(result > 0 && result <= maximum, "INVALID_CONFIG", "Invalid count " + name);
    return static_cast<uint32_t>(result);
  }
  int64_t seconds(const std::string &name, double value) {
    const double result = parameter(name, value);
    require(std::isfinite(result) && result >= 1e-9 && result <= 60, "INVALID_CONFIG", "Invalid duration " + name);
    return static_cast<int64_t>(result*1e9);
  }
  static ImageView view(const Image &image) {
    return {image.data.data(), image.data.size(), image.step, image.width, image.height, image.encoding, image.is_bigendian != 0};
  }
  void observe_clock(int64_t clock) {
    if (clock_reset_requested_.exchange(false) || clock < last_clock_) {
      for (auto &[child,edge] : tf_history_) { (void)child; if (!edge.fixed) edge.samples.clear(); }
      if (goal_) { stop_ = true; forced_error_ = "SIM_TIME_RESET"; forced_message_ = "Simulation clock moved backwards"; }
    }
    last_clock_ = clock;
  }
  void receive_tf(const TfMessage &message, bool fixed) {
    observe_clock(now().nanoseconds());
    if (message.transforms.size() > 512) {
      const std::string reason = "TF message exceeds 512-transform bound";
      diagnostic("TF_UNAVAILABLE",reason);
      if (goal_) { stop_ = true; forced_error_ = "TF_UNAVAILABLE"; forced_message_ = reason; }
      poll(); return;
    }
    for (const auto &transform : message.transforms) {
      const auto &child = transform.child_frame_id, &parent = transform.header.frame_id;
      try {
        const auto &q = transform.transform.rotation;
        const auto &t = transform.transform.translation;
        const Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
        require(!child.empty() && !parent.empty() && child != parent && child.size() <= 128 && parent.size() <= 128 &&
          child.front() != '/' && parent.front() != '/' && transform.header.stamp.sec >= 0 &&
          transform.header.stamp.nanosec < 1000000000 && rotation.coeffs().allFinite() &&
          std::abs(rotation.norm()-1) < 1e-9 && std::isfinite(t.x) && std::isfinite(t.y) && std::isfinite(t.z),
          "TF_UNAVAILABLE", "Invalid TF identifiers, timestamp or numeric transform");
        const int64_t stamp = fixed ? 0 : int64_t(transform.header.stamp.sec)*1000000000 + transform.header.stamp.nanosec;
        require(fixed || stamp > 0,"TF_UNAVAILABLE","Dynamic TF needs a positive timestamp");
        auto found = tf_history_.find(child);
        if (found == tf_history_.end()) {
          require(tf_history_.size() < 512,"TF_UNAVAILABLE","Bounded TF graph exceeds 512 edges");
          found = tf_history_.emplace(child,TfEdge{parent,fixed,{}}).first;
        }
        auto &edge = found->second;
        const auto same = [](const Transform &a, const Transform &b) {
          return tf2::transformToEigen(a).matrix().isApprox(tf2::transformToEigen(b).matrix(),1e-12);
        };
        const bool changed = edge.parent != parent || edge.fixed != fixed ||
          (fixed && !edge.samples.empty() && !same(*edge.samples.begin()->second,transform));
        if (changed) {
          if (goal_ && used_tf_.count(child)) {
            stop_ = true; forced_error_ = "TF_CHANGED"; forced_message_ = "Used TF topology/static calibration changed: " + child;
          }
          edge = TfEdge{parent,fixed,{}};
        }
        const auto previous = edge.samples.find(stamp);
        if (previous != edge.samples.end()) {
          if (!previous->second || !same(*previous->second,transform)) {
            previous->second.reset();
            throw Rejection("TF_CHANGED","Conflicting TF replacement at one timestamp: " + child);
          }
          continue; // An identical publication adds no new information.
        }
        edge.samples.emplace(stamp,transform);
        if (edge.samples.size() > 200) edge.samples.erase(edge.samples.begin());
      } catch (const Rejection &error) {
        diagnostic(error.code,error.what());
        if (goal_ && used_tf_.count(child)) { stop_ = true; forced_error_ = error.code; forced_message_ = error.what(); }
      }
    }
    poll();
  }
  Transform capture_transform(const std::string &camera, int64_t stamp, std::string &calibration) {
    // The bounded snapshot contains exactly the records checked here. TF2 owns
    // interpolation; no listener/cache race or private TF2 accessor is needed.
    tf2::BufferCore snapshot(tf2::durationFromSec(60.0));
    std::set<std::string> visited;
    std::string child = camera;
    std::ostringstream identity; identity.precision(17);
    const auto insert = [&](const std::optional<Transform> &sample, bool fixed) {
      require(sample.has_value(),"TF_UNAVAILABLE","Conflicting TF timestamp has no valid support");
      require(snapshot.setTransform(*sample,"tote_fusion_snapshot",fixed),"TF_UNAVAILABLE","TF2 rejected snapshot input");
    };
    while (child != model_.body_frame) {
      require(visited.insert(child).second && visited.size() <= 512,"TF_UNAVAILABLE","TF chain contains a cycle or exceeds its bound");
      const auto found = tf_history_.find(child);
      require(found != tf_history_.end(),"TF_UNAVAILABLE","Missing TF chain metadata: " + child);
      const auto &edge = found->second;
      require(!edge.samples.empty(),"TF_UNAVAILABLE","No retained TF support: " + child);
      identity << child << ' ' << edge.parent << ' ' << edge.fixed << ' ';
      if (edge.fixed) {
        insert(edge.samples.begin()->second,true);
        identity << tf2::transformToEigen(*edge.samples.begin()->second).matrix() << ' ';
      } else {
        const auto after = edge.samples.lower_bound(stamp);
        require(after != edge.samples.end(),"TF_UNAVAILABLE","No following TF support: " + child);
        if (after->first == stamp) insert(after->second,false);
        else {
          require(after != edge.samples.begin(),"TF_UNAVAILABLE","No preceding TF support: " + child);
          const auto before = std::prev(after);
          require(after->first-before->first <= tf_gap_ && stamp-before->first <= tf_age_ && after->first-stamp <= tf_age_,
            "TF_UNAVAILABLE","Dynamic TF interpolation gap/support age exceeds profile: " + child);
          insert(before->second,false); insert(after->second,false);
        }
      }
      used_tf_.insert(child); child = edge.parent;
    }
    calibration = identity.str();
    try { return snapshot.lookupTransform(model_.body_frame,camera,tf2::TimePoint(tf2::Duration(stamp))); }
    catch (const tf2::TransformException &error) { throw Rejection("TF_UNAVAILABLE",error.what()); }
  }
  template<class Message> static std::size_t bytes(const Message &message) {
    if constexpr (std::is_same_v<Message, Image>) return sizeof(Message) + message.data.size();
    else return sizeof(Message) + message.d.size()*sizeof(double);
  }
  template<class Message> void receive(uint32_t camera, std::shared_ptr<const Message> message, std::shared_ptr<const Message> Pending::*slot) {
    poll();  // Enforce reset/deadline/cancellation before admitting another sample.
    if (!goal_ || worker_.valid()) return;
    const int64_t stamp = int64_t(message->header.stamp.sec)*1000000000 + message->header.stamp.nanosec;
    const Key key{camera,stamp};
    try {
      require(message->header.stamp.sec >= 0 && message->header.stamp.nanosec < 1000000000,
        "STALE_OBSERVATION", "Invalid acquisition timestamp fields");
      validate_capture(now().nanoseconds(), stamp, 0, max_age_, future_slop_);
      // Fresh in-flight messages acquired before the trigger are outside the
      // requested interval. Count and diagnose them; never fit or retimestamp them.
      if (stamp < started_) {
        require(outside_.size() < max_frames_, "BUFFER_LIMIT", "Too many pre-trigger acquisition stamps");
        if (outside_.insert(key).second) diagnostic("OUTSIDE_COLLECTION", "Excluded fresh pre-trigger frame");
        return;
      }
      require(stamp <= end_ && !completed_.count(key), "DUPLICATE_OBSERVATION", "Capture outside interval or already accepted");
      require(message->header.frame_id == cameras_.at(camera).optical, "MASK_MISMATCH", "Unexpected optical frame");
      require(bytes_ + bytes(*message) + point_count_*sizeof(Observation)+boundary_count_*sizeof(BoundaryPoint) <= max_bytes_ &&
        (pending_.count(key) || pending_.size() < max_frames_),
        "BUFFER_LIMIT", "Bounded image queue would overflow");
      auto &frame = pending_[key];
      require(!(frame.*slot), "DUPLICATE_OBSERVATION", "Duplicate stream sample within the request");
      frame.*slot = std::move(message); bytes_ += bytes(*(frame.*slot));
      peak_bytes_ = std::max(peak_bytes_, bytes_);
      if (!(frame.depth && frame.mask && frame.info)) return;
      accept_frame(key, frame);
      completed_.insert(key);
      bytes_ -= bytes(*frame.depth) + bytes(*frame.mask) + bytes(*frame.info);
      pending_.erase(key);
    } catch (const Rejection &error) { fail(error.code, error.what(), key); }
  }
  void accept_frame(const Key &key, const Pending &frame) {
    auto &camera = cameras_.at(key.first); const int64_t stamp = key.second;
    const auto &info = *frame.info;
    require(info.width == frame.depth->width && info.height == frame.depth->height &&
      std::all_of(info.d.begin(), info.d.end(), [](double d) { return std::isfinite(d) && d == 0; }) &&
      info.r == std::array<double,9>{1,0,0,0,1,0,0,0,1} && info.k[8] == 1 &&
      info.k[1] == 0 && info.k[3] == 0 && info.k[6] == 0 && info.k[7] == 0 &&
      info.p[1] == 0 && std::isfinite(info.p[3]) && info.p[4] == 0 && std::isfinite(info.p[7]) &&
      info.p[8] == 0 && info.p[9] == 0 && info.p[10] == 1 && info.p[11] == 0 &&
      info.k[0] == info.p[0] && info.k[4] == info.p[5] && info.k[2] == info.p[2] && info.k[5] == info.p[6],
      "INVALID_INTRINSICS", "Expected consistent undistorted registered pinhole CameraInfo");
    std::ostringstream calibrated; calibrated.precision(17); calibrated << camera.optical << ' ' << info.width << ' ' << info.height;
    const auto append = [&](const auto &values) { for (const auto value : values) calibrated << ' ' << value; };
    append(info.k); append(info.p); append(info.r); append(info.d);
    auto local = project(view(*frame.depth), view(*frame.mask),
      {info.k[0],info.k[4],info.k[2],info.k[5]}, depth_min_, depth_max_, stride_, max_points_-point_count_);
    std::string tf_calibration;
    const auto transform = capture_transform(camera.optical,stamp,tf_calibration);
    const auto hash = sha256(calibrated.str()+tf_calibration);
    require(camera.calibration.empty() || hash == camera.calibration, "CALIBRATION_CHANGED", "Calibration changed during collection");
    const Pose body_camera = tf2::transformToEigen(transform);
    const auto points = transform_points(std::move(local), body_camera);
    require(accepted_ < max_frames_ && bytes_ + (point_count_+points.size())*sizeof(Observation) <= max_bytes_,
      "BUFFER_LIMIT", "Frame or point budget exceeded");
    constexpr int64_t holdout_block_ns = 100000000;
    const bool fitting = multi_ ? ((stamp-started_)/holdout_block_ns)%2 == 0 : camera.accepted%2 == 0;
    auto &destination = fitting ? fitting_ : held_out_;
    for (const auto &point : points) destination.push_back({point,body_camera.translation(),accepted_,key.first});
    if (geometry_) {
      auto boundaries = image_boundaries(view(*frame.depth),view(*frame.mask),
        {info.k[0],info.k[4],info.k[2],info.k[5]},body_camera,model_,accepted_,key.first,
        fit_.camera_noise_sigma_m.at(key.first),max_points_-boundary_count_);
      require(bytes_+(point_count_+points.size())*sizeof(Observation)+(boundary_count_+boundaries.size())*sizeof(BoundaryPoint) <= max_bytes_,
        "BUFFER_LIMIT","Geometry point storage exceeds aggregate budget");
      auto &edges = fitting ? boundary_fitting_ : boundary_held_out_;
      edges.insert(edges.end(),boundaries.begin(),boundaries.end()); boundary_count_ += boundaries.size();
    }
    point_count_ += points.size(); camera.calibration = hash; ++camera.accepted;
    if (accepted_ == 0) first_ = stamp;
    first_ = std::min(first_, stamp); last_ = std::max(last_, stamp); ++accepted_; capture_stamps_.push_back(key);
    feedback("COLLECTING");
  }
  void diagnostic(const std::string &code, const std::string &message) {
    diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "tote_fusion"; status.hardware_id = "tote_cameras";
    status.level = code == "IDLE" || code == "OK" ? status.OK : code == "OUTSIDE_COLLECTION" || code == "DEGRADED_OK" ? status.WARN : status.ERROR;
    status.message = code + ": " + message;
    for (const auto &[key, value] : std::map<std::string,std::string>{
      {"code",code},{"model_revision",std::to_string(model_.revision)},
      {"accepted_frames",std::to_string(accepted_)},{"point_count",std::to_string(point_count_)},
      {"peak_queued_bytes",std::to_string(peak_bytes_)},{"queued_bytes",std::to_string(bytes_)},{"last_capture_ns",std::to_string(last_)}}) {
      diagnostic_msgs::msg::KeyValue item; item.key = key; item.value = value; status.values.push_back(item);
    }
    for (const auto &camera : cameras_) {
      diagnostic_msgs::msg::KeyValue frames, calibration;
      frames.key = camera.id+".accepted_frames"; frames.value = std::to_string(camera.accepted);
      calibration.key = camera.id+".calibration_id"; calibration.value = camera.calibration;
      status.values.push_back(frames); status.values.push_back(calibration);
    }
    array.status.push_back(status); diagnostics_->publish(array);
    if (status.level == status.ERROR) RCLCPP_ERROR(get_logger(), "%s", status.message.c_str());
  }
  void feedback(const std::string &state) {
    auto message = std::make_shared<Action::Feedback>(); message->state = state;
    message->accepted_frames = accepted_; message->point_count = point_count_;
    goal_->publish_feedback(message);
  }
  void fail(const std::string &code, const std::string &message, std::optional<Key> bad_stamp = std::nullopt) {
    assert(goal_ && !worker_.valid());
    auto result = std::make_shared<Action::Result>(); result->code = code; result->message = message;
    result->accepted_frames = accepted_; result->rejected_frames = outside_.size()+pending_.size()+unmatched_;
    if (bad_stamp && !outside_.count(*bad_stamp) && !pending_.count(*bad_stamp)) ++result->rejected_frames;
    diagnostic(code,message);
    if (goal_->is_canceling()) goal_->canceled(result); else goal_->abort(result);
    clear();
  }
  void clear() {
    pending_.clear(); fitting_.clear(); held_out_.clear(); bytes_ = 0;
    boundary_fitting_.clear(); boundary_held_out_.clear(); boundary_count_ = 0;
    goal_.reset(); reserved_ = false; used_tf_.clear();
  }
  void publish(const Estimate &estimate) {
    assert(goal_ && !worker_.valid());
    Candidate candidate;
    candidate.header.frame_id = model_.body_frame; candidate.header.stamp = rclcpp::Time(last_);
    candidate.object_id = goal_->get_goal()->object_id;
    const auto uuid = goal_->get_goal_id();
    candidate.candidate_id = sha256(std::string(reinterpret_cast<const char *>(uuid.data()), uuid.size()));
    candidate.model_id = model_.id; candidate.parent_model_revision = model_.revision; candidate.geometry_hash = model_.hash;
    const auto &t = estimate.pose.translation(); const Eigen::Quaterniond q(estimate.pose.linear());
    candidate.pose.pose.position.x = t.x(); candidate.pose.pose.position.y = t.y(); candidate.pose.pose.position.z = t.z();
    candidate.pose.pose.orientation.x = q.x(); candidate.pose.pose.orientation.y = q.y();
    candidate.pose.pose.orientation.z = q.z(); candidate.pose.pose.orientation.w = q.w();
    const auto &alt = estimate.pi_equivalent_pose.translation();
    const Eigen::Quaterniond aq(estimate.pi_equivalent_pose.linear());
    candidate.pi_equivalent_pose.position.x = alt.x(); candidate.pi_equivalent_pose.position.y = alt.y();
    candidate.pi_equivalent_pose.position.z = alt.z();
    candidate.pi_equivalent_pose.orientation.x = aq.x(); candidate.pi_equivalent_pose.orientation.y = aq.y();
    candidate.pi_equivalent_pose.orientation.z = aq.z(); candidate.pi_equivalent_pose.orientation.w = aq.w();
    candidate.solver_iterations = estimate.iterations;
    for (int row=0; row<6; ++row) for (int column=0; column<6; ++column) candidate.pose.covariance[row*6+column] = estimate.covariance(row,column);
    candidate.first_capture = rclcpp::Time(first_); candidate.last_capture = candidate.header.stamp;
    candidate.issue_time = now(); candidate.valid_until = rclcpp::Time(last_+expiry_);
    require(rclcpp::Time(candidate.issue_time).nanoseconds() >= last_ &&
      rclcpp::Time(candidate.issue_time).nanoseconds() <= last_+expiry_,
      "STALE_OBSERVATION", "Candidate capture is future or expired at publication");
    for (uint32_t index=0; index<cameras_.size(); ++index) {
      const auto &camera = cameras_[index];
      if (camera.accepted == 0) { candidate.missing_camera_ids.push_back(camera.id); continue; }
      candidate.source_camera_ids.push_back(camera.id); candidate.calibration_ids.push_back(camera.calibration);
      candidate.optical_frames.push_back(camera.optical); candidate.source_frame_counts.push_back(camera.accepted);
      candidate.camera_noise_sigma_m.push_back(fit_.camera_noise_sigma_m[index]);
      candidate.camera_held_out_rmse_m.push_back(estimate.camera_held_out_rmse_m.at(index));
    }
    for (const auto &[camera,stamp] : capture_stamps_) {
      candidate.tf_lookup_stamps.push_back(rclcpp::Time(stamp)); candidate.tf_lookup_camera_ids.push_back(cameras_.at(camera).id);
    }
    candidate.depth_convention = "registered_axial_Z_metres";
    candidate.noise_profile = profile_; candidate.sample_count = accepted_; candidate.point_count = estimate.fitting_points;
    candidate.held_out_point_count = estimate.held_out_points; candidate.training_rmse_m = estimate.training_rmse_m;
    candidate.held_out_rmse_m = estimate.held_out_rmse_m; candidate.inlier_fraction = estimate.inlier_fraction;
    candidate.covered_surface_count = estimate.surface_count; candidate.observable_rank = 6+estimate.dimension_axes.size();
    candidate.estimation_mode = geometry_ ? Candidate::POSE_AND_GEOMETRY : Candidate::POSE_ONLY;
    for (int axis=0; axis<6; ++axis) candidate.data_information_eigenvalues[axis] = estimate.information_eigenvalues[axis];
    candidate.ambiguity_status = estimate.ambiguity_status;
    candidate.covariance_convention = "body-frame xyz/rotation perturbation; local pi-yaw class; uncalibrated";
    candidate.covariance_calibrated = false;
    if (geometry_) {
      const auto vector_message = [](const Vec3 &v) { geometry_msgs::msg::Vector3 result; result.x = v.x(); result.y = v.y(); result.z = v.z(); return result; };
      const std::vector<std::string> names{"length","width","height"};
      candidate.joint_state_order = {"x","y","z","rotX","rotY","rotZ"};
      candidate.joint_information_convention = "scaled data-only left body tangent rho/phi followed by dimensions; not ROS position covariance coordinates";
      candidate.joint_information_scales = {.01,.01,.01,.05,.05,.05};
      Vec3 envelope = model_.dimensions;
      for (std::size_t k=0; k<estimate.dimension_axes.size(); ++k) {
        const auto axis = estimate.dimension_axes[k]; const double sigma = std::sqrt(estimate.joint_covariance(6+k,6+k));
        candidate.parameter_names.push_back(names[axis]); candidate.parameter_values_m.push_back(estimate.dimensions[axis]);
        candidate.parameter_sigma_m.push_back(sigma); candidate.joint_state_order.push_back(names[axis]); candidate.joint_information_scales.push_back(.01);
        envelope[axis] = std::max(envelope[axis],estimate.dimensions[axis]+3*sigma);
      }
      candidate.proposed_outer_dimensions_m = vector_message(estimate.dimensions);
      candidate.conditional_inner_opening_m = vector_message(Vec3(estimate.dimensions.x()-2*model_.wall_thickness,estimate.dimensions.y()-2*model_.wall_thickness,0));
      candidate.inner_opening_assumption = "XY only; nominal wall thickness and unchanged internal features; Z unresolved";
      candidate.unresolved_geometry = {"height","wall_thickness","floor_and_stacking_foot_offsets","underside","hidden_defects"};
      for (uint32_t axis : {0u,1u}) if (std::find(estimate.dimension_axes.begin(),estimate.dimension_axes.end(),axis) == estimate.dimension_axes.end()) candidate.unresolved_geometry.push_back(names[axis]);
      candidate.collision_lower_m = vector_message(Vec3(-envelope.x()/2-.002,-envelope.y()/2-.002,-.002));
      candidate.collision_upper_m = vector_message(Vec3(envelope.x()/2+.002,envelope.y()/2+.002,envelope.z()+.002));
      candidate.geometry_generator = "EU6423_17_boxes_v1";
      std::ostringstream proposal; proposal.precision(17); proposal << candidate.geometry_generator << ' ' << model_.hash;
      for (const auto &box : parametric_model(model_,estimate.dimensions).boxes) proposal << ' ' << box.low.transpose() << ' ' << box.high.transpose();
      candidate.proposed_geometry_hash = sha256(proposal.str());
      for (int row=0; row<estimate.joint_covariance.rows(); ++row) for (int column=0; column<estimate.joint_covariance.cols(); ++column) candidate.joint_covariance.push_back(estimate.joint_covariance(row,column));
      for (const auto value : estimate.joint_information_eigenvalues) candidate.joint_data_information_eigenvalues.push_back(value);
      candidate.fixed_geometry_converged = estimate.fixed_geometry_converged;
      candidate.fixed_geometry_iterations = estimate.fixed_geometry_iterations;
      candidate.fixed_geometry_held_out_rmse_m = estimate.fixed_geometry_held_out_rmse_m;
      candidate.fixed_geometry_boundary_rmse_m = estimate.fixed_boundary_rmse_m;
      candidate.held_out_boundary_rmse_m = estimate.held_out_boundary_rmse_m;
      for (const auto &m : estimate.dimension_measurements) {
        candidate.boundary_parameter_names.push_back(names[m.axis]); candidate.boundary_camera_ids.push_back(cameras_.at(m.camera).id);
        candidate.boundary_support_counts.push_back(m.support); candidate.boundary_measurements_m.push_back(m.value); candidate.boundary_measurement_sigma_m.push_back(m.sigma);
      }
      candidate.covariance_convention += "; joint ROS xyz/left-rotation and metres; conditional on nominal unresolved geometry";
    }
    auto result = std::make_shared<Action::Result>(); result->success = true; result->code = candidate.missing_camera_ids.empty() ? "OK" : "DEGRADED_OK";
    result->candidate = candidate; result->accepted_frames = accepted_; result->rejected_frames = outside_.size()+unmatched_;
    {
      // Time reads and validation are above this lock. Publication commits the
      // candidate; a later jump affects subsequent requests, not this result.
      std::lock_guard lock(publication_mutex_);
      require(!clock_reset_requested_.load(),"SIM_TIME_RESET","Clock reset before candidate publication");
      candidates_->publish(candidate);
    }
    goal_->succeed(result); diagnostic(result->code,estimate.ambiguity_status); clear();
  }
  void poll() {
    const auto clock = now().nanoseconds();
    observe_clock(clock);
    if (!goal_) return;
    if (std::chrono::duration<double>(Steady::now()-wall_started_).count() > wall_limit_) {
      stop_ = true; forced_error_ = "TIMEOUT";
      forced_message_ = "Steady wall deadline expired";
    }
    if (worker_.valid()) {
      if (worker_.wait_for(0s) != std::future_status::ready) return;
      try {
        const auto result = worker_.get();
        require(forced_error_.empty(), forced_error_, forced_message_);
        publish(result);
      } catch (const Rejection &error) { fail(forced_error_.empty() ? error.code : forced_error_, error.what()); }
      return;
    }
    if (!forced_error_.empty()) { fail(forced_error_, forced_message_); return; }
    if (clock < end_) return;
    const auto participating = std::count_if(cameras_.begin(),cameras_.end(),[](const auto &camera) { return camera.accepted >= 2; });
    if (participating < goal_->get_goal()->min_distinct_views) {
      fail("INSUFFICIENT_DATA", "Too few cameras with separate acquisition samples"); return;
    }
    unmatched_ = pending_.size(); pending_.clear(); bytes_ = 0;
    feedback("ESTIMATING");
    worker_ = std::async(std::launch::async, [this, fitting = std::move(fitting_), held = std::move(held_out_), edges = std::move(boundary_fitting_), held_edges = std::move(boundary_held_out_)] {
      return estimate(model_,fitting,held,fit_,[this] { return stop_.load(); },edges,held_edges);
    });
  }
  const Model model_;
  FitConfig fit_;
  std::map<std::string,TfEdge> tf_history_;
  std::set<std::string> used_tf_;
  rclcpp::Subscription<TfMessage>::SharedPtr tf_dynamic_, tf_static_;
  std::string profile_, forced_error_, forced_message_;
  std::vector<Camera> cameras_;
  bool multi_ = false, geometry_ = false;
  uint32_t boundary_count_ = 0;
  std::vector<BoundaryPoint> boundary_fitting_, boundary_held_out_;
  uint32_t stride_, max_frames_, max_points_, max_bytes_, accepted_ = 0, point_count_ = 0, unmatched_ = 0;
  double depth_min_, depth_max_, wall_limit_;
  int64_t max_age_, future_slop_, capture_limit_, expiry_, tf_gap_, tf_age_, started_ = 0, end_ = 0, first_ = 0, last_ = 0, last_clock_ = 0;
  std::size_t bytes_ = 0, peak_bytes_ = 0; bool reserved_ = false;
  Steady::time_point wall_started_;
  std::map<Key,Pending> pending_;
  std::set<Key> completed_, outside_;
  std::vector<Observation> fitting_, held_out_;
  std::vector<Key> capture_stamps_;
  std::atomic<bool> stop_{false}; std::future<Estimate> worker_;
  std::shared_ptr<Goal> goal_; rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Publisher<Candidate>::SharedPtr candidates_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_lock_;
  std::atomic<bool> clock_reset_requested_{false};
  std::mutex publication_mutex_;
  rclcpp::JumpHandler::SharedPtr jump_handler_;
};
}  // namespace tote_fusion

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tote_fusion::FusionNode>());
  rclcpp::shutdown();
}
