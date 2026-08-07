-- Pure localization against a frozen DOGZILLA Cartographer PBStream.
--
-- DOGZILLA has no wheel odometry, so local motion is estimated directly from
-- the MS200 scans. A small rolling set of live submaps is retained while all
-- map-building state loaded from test1.pbstream remains frozen.

local options = include "dogzilla_2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- Localize promptly against the frozen trajectory without saturating the Pi.
POSE_GRAPH.optimize_every_n_nodes = 5
POSE_GRAPH.constraint_builder.sampling_ratio = 0.30
POSE_GRAPH.global_sampling_ratio = 0.10
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.58
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.max_num_final_iterations = 50

return options
