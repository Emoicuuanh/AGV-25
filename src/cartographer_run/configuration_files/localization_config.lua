include "slam_config.lua"

-- Pure localization mode: không tạo map mới, chỉ giữ vài submap gần nhất.
TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 7,  -- tăng từ 3 → 7 để khớp ổn định hơn khi quay đầu hoặc đi lại vùng cũ
}

-- Cho phép cập nhật pose nhanh và chính xác hơn
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05

-- Không cần global search liên tục
POSE_GRAPH.global_sampling_ratio = 0.001
POSE_GRAPH.optimize_every_n_nodes = 20

-- Giảm tần suất global search để tránh relocalize nhầm
POSE_GRAPH.global_constraint_search_after_n_seconds = 3000000  -- tăng từ 60 → 300

-- Giới hạn matching nghiêm ngặt hơn để tránh match sai vùng
POSE_GRAPH.constraint_builder.min_score = 0.5                     -- tăng từ 0.5
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.5 -- tăng từ 0.45

-- Tăng số mẫu kiểm tra để matching chắc chắn hơn
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05  -- tăng từ 0.01

-- (Tùy chọn) Nếu bạn không cần global search nữa, có thể disable hoàn toàn:
-- POSE_GRAPH.global_sampling_ratio = 0.0
-- POSE_GRAPH.global_constraint_search_after_n_seconds = 999999

return options

