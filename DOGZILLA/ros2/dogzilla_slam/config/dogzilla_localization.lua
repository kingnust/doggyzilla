-- Pure localization against a frozen DOGZILLA Cartographer PBStream.
--
-- DOGZILLA has no wheel odometry, so local motion is estimated directly from
-- the MS200 scans. A small rolling set of live submaps is retained while all
-- map-building state loaded from test1.pbstream remains frozen.

local options = include "dogzilla_2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- Keep localization responsive without starving Nav2 on the four-core Pi.
options.pose_publish_period_sec = 0.02
options.trajectory_publish_period_sec = 0.10
options.submap_publish_period_sec = 1.0
MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.constraint_builder.sampling_ratio = 0.20
POSE_GRAPH.global_sampling_ratio = 0.05
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.58
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.max_num_final_iterations = 25
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4

return options
