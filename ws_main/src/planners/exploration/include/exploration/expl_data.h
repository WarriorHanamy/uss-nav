#ifndef _EXPLORATION_EXPL_DATA_H_
#define _EXPLORATION_EXPL_DATA_H_

#include <Eigen/Eigen>
#include <exploration/ftr_data_structure.h>
#include <map>
#include <vector>
#include <memory>

namespace ego_planner {

/**
 * Per-cycle exploration data including frontiers, viewpoints, and tour results.
 */
struct ExplorationData {
  Frontier                          frontier_to_goal, frontier_to_explore_;
  std::vector<std::vector<Eigen::Vector3d>> frontiers_;
  std::vector<Frontier>                  frontiers_with_info_;
  std::vector<std::vector<Eigen::Vector3d>> dead_frontiers_;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> frontier_boxes_;
  std::unordered_map<int, int>           topo_blacklist_;
  bool                                   flag_first_plangoal_;
  std::vector<Eigen::Vector3d>           points_;
  std::vector<Eigen::Vector3d>           averages_;
  std::vector<Eigen::Vector3d>           views_;
  std::vector<double>                    yaws_;
  std::vector<Eigen::Vector3d>           global_tour_;
  std::map<int, std::vector<Eigen::Vector3d>> global_tour_map_;
  bool                                   force_plangoal_by_frontier_;

  std::vector<Frontier>                  last_frontiers_with_info_;
  std::vector<int>                       last_indices_;
  bool                                   is_gohome = false;
  bool                                   is_stick_to_last = false;

  std::vector<int>                       refined_ids_;
  std::vector<std::vector<Eigen::Vector3d>> n_points_;
  std::vector<Eigen::Vector3d>           unrefined_points_;
  std::vector<Eigen::Vector3d>           refined_points_;
  std::vector<Eigen::Vector3d>           refined_views_;
  std::vector<Eigen::Vector3d>           refined_views1_, refined_views2_;
  std::vector<Eigen::Vector3d>           refined_tour_;

  std::vector<Eigen::Vector3d>           path_next_goal_;
  std::vector<int>                       last_grid_ids_;

  std::vector<Eigen::Vector3d>           views_vis1_, views_vis2_;
  std::vector<Eigen::Vector3d>           centers_, scales_;
  typedef std::shared_ptr<ExplorationData> Ptr;
};

/**
 * Exploration planner parameters.
 */
struct ExplorationParam
{
  bool         refine_local_;
  int          refined_num_;
  double       refined_radius_;
  int          top_view_num_;
  double       max_decay_;
  std::string  tsp_dir_;
  double       relax_time_;
  double       radius_close_;
  double       radius_far_;
  int          frontier_tsp_mode_{0};
  double       track_dist_;
  double       track_dist_thr_;
  double       track_replan_dist_;
  double       track_turn_yaw_dist_;
  double       track_fly_yaw_thr_;
  double       track_yaw_thr_;
  double       track_detect_error_;

  typedef std::shared_ptr<ExplorationParam> Ptr;
};

}  // namespace ego_planner

#endif
