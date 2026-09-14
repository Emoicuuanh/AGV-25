include "map_builder.lua"
include "trajectory_builder.lua"

options = {
    map_builder = MAP_BUILDER,
    trajectory_builder = TRAJECTORY_BUILDER,
    map_frame = "map",
    tracking_frame = "base_footprint",
    published_frame = "odom",
    odom_frame = "odom",
    provide_odom_frame = false,
    publish_frame_projected_to_2d = false,
    use_odometry = false,
    use_nav_sat = false,
    use_landmarks = false,
    num_laser_scans = 1,
    num_multi_echo_laser_scans = 0,
    num_subdivisions_per_laser_scan = 16,
    num_point_clouds = 0,
    lookup_transform_timeout_sec = 0.2,
    submap_publish_period_sec = 0.3,
    pose_publish_period_sec = 5e-3,
    trajectory_publish_period_sec = 0.1,
    rangefinder_sampling_ratio = 1.,
    odometry_sampling_ratio = 1.,
    fixed_frame_pose_sampling_ratio = 1.,
    imu_sampling_ratio = .5,
    landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 3

-- Không cần cấu hình scan matching nữa, nhưng giữ các thiết lập submap để đảm bảo tính tương thích
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Nếu bạn muốn giữ nguyên cấu trúc để debug/visualize:
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90  -- hoặc giá trị bạn đã dùng trước đây

-- Không cần scan matching hoặc motion filter
-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
-- v.v.

-- Giữ nguyên các giá trị liên quan đến pose graph nếu bạn có ý định chỉnh sửa pose hoặc submap
POSE_GRAPH.optimization_problem.huber_scale = 5
POSE_GRAPH.constraint_builder.min_score = .6
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e1

return options
