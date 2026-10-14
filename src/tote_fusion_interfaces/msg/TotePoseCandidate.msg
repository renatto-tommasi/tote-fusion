# Pose or geometry candidate only. Header stamp is last_capture in the configured body frame.
uint32 schema_version 3
uint8 POSE_ONLY=0
uint8 estimation_mode
std_msgs/Header header
string object_id
string candidate_id
string model_id
uint64 parent_model_revision
string geometry_hash
geometry_msgs/PoseWithCovariance pose
# Independently fitted supported pi-yaw hypothesis; covariance is conditional.
geometry_msgs/Pose pi_equivalent_pose
uint32 solver_iterations
builtin_interfaces/Time first_capture
builtin_interfaces/Time last_capture
builtin_interfaces/Time issue_time
builtin_interfaces/Time valid_until
string[] source_camera_ids
string[] missing_camera_ids
uint32[] source_frame_counts
float64[] camera_noise_sigma_m
float64[] camera_held_out_rmse_m
string[] calibration_ids
string[] optical_frames
builtin_interfaces/Time[] tf_lookup_stamps
# One camera ID per acquisition-time lookup stamp.
string[] tf_lookup_camera_ids
string depth_convention
string noise_profile
uint32 sample_count
uint32 point_count
uint32 held_out_point_count
float64 training_rmse_m
float64 held_out_rmse_m
float64 inlier_fraction
uint32 covered_surface_count

# Full accepted state rank: 6 for pose, 6+estimated parameters for geometry.
uint8 observable_rank
# Pose block information with dimensions held fixed; full joint modes below.
float64[6] data_information_eigenvalues
string ambiguity_status
string covariance_convention
bool covariance_calibrated
# Geometry fields are empty in POSE_ONLY. Proposals are never committed here.
uint8 POSE_AND_GEOMETRY=1
string[] parameter_names
float64[] parameter_values_m
float64[] parameter_sigma_m
geometry_msgs/Vector3 proposed_outer_dimensions_m
geometry_msgs/Vector3 conditional_inner_opening_m
string inner_opening_assumption
string[] unresolved_geometry
string geometry_generator
string proposed_geometry_hash
geometry_msgs/Vector3 collision_lower_m
geometry_msgs/Vector3 collision_upper_m
# Row-major covariance, ROS xyz/left-rotation followed by estimated dimensions.
string[] joint_state_order
float64[] joint_covariance
float64[] joint_data_information_eigenvalues
string joint_information_convention
float64[] joint_information_scales
float64 fixed_geometry_boundary_rmse_m
float64 held_out_boundary_rmse_m
# Aligned arrays: parameter, camera, count, independent held-out measurement.
string[] boundary_parameter_names
string[] boundary_camera_ids
uint32[] boundary_support_counts
float64[] boundary_measurements_m
float64[] boundary_measurement_sigma_m
bool fixed_geometry_converged
uint32 fixed_geometry_iterations
float64 fixed_geometry_held_out_rmse_m
