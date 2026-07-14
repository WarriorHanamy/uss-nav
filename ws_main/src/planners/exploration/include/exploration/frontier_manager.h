#ifndef _FRONTIER_MANAGER_H_
#define _FRONTIER_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <utility>
#include <exploration/graph_node.h>
#include <exploration/graph_search.h>
#include <exploration/frontier_finder.h>
#include <exploration/expl_data.h>
#include <nav_msgs/Odometry.h>
#include <exploration/hgrid.h>
#include <exploration/visualization.hpp>
#include <map_interface/map_interface.hpp>
#include <scene_graph/scene_graph.h>


using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using std::list;
using std::pair;

namespace ego_planner {

enum EXPL_RESULT { NO_FRONTIER, FAIL, SUCCEED};

enum PLAN_TO_WHAT {
    FRONTIER_TO_GOAL = 1,
    PATH_TO_GOAL = 2,
    OTHER = 3,
};

class FrontierManager{
 public:
  /**
   * Construct the frontier manager with map and scene graph.
   *
   * @param[in] nh            ROS node handle
   * @param[in] map           Map interface pointer
   * @param[in] scene_graph   Scene graph pointer
   */
  FrontierManager(ros::NodeHandle& nh, const MapInterface::Ptr& map, SceneGraph::Ptr scene_graph);
  ~FrontierManager() = default;

  typedef std::shared_ptr<FrontierManager> Ptr;

  /**
   * Plan exploration using the rapid frontier method.
   *
   * @param[in]  pos      Current position [m]
   * @param[in]  vel      Current velocity [m/s]
   * @param[out] aim_pos  Target aim position [m]
   * @param[out] aim_vel  Target aim velocity [m/s]
   * @param[out] path_res Planned path points [m]
   * @return EXPL_RESULT enum
   */
  int planExploreRapid(const Vector3d& pos, const Vector3d& vel, 
                       Vector3d& aim_pos, Vector3d& aim_vel, vector<Eigen::Vector3d>& path_res);
  /**
   * Plan exploration using the TSP-based global tour method.
   *
   * @param[in]  pos      Current position [m]
   * @param[in]  vel      Current velocity [m/s]
   * @param[in]  yaw      Current yaw [rad]
   * @param[out] aim_pos  Target aim position [m]
   * @param[out] aim_vel  Target aim velocity [m/s]
   * @param[out] aim_yaw  Target aim yaw [rad]
   * @param[out] path_res Planned path points [m]
   * @return EXPL_RESULT enum
   */
  int planExploreTSP(const Vector3d& pos, const Vector3d& vel, const double& yaw,
                     Vector3d& aim_pos, Vector3d& aim_vel, double& aim_yaw, vector<Eigen::Vector3d>& path_res);
  /**
   * Update frontier set for the next planning cycle.
   *
   * @param[in] pos  Current position [m]
   * @param[in] yaw  Current yaw [rad]
   */
  void updateFrontiersForPlanning(const Vector3d& pos, const double& yaw);
  /**
   * Plan a path to a tracking goal for target following.
   *
   * @param[in]  pos      Current position [m]
   * @param[in]  vel      Current velocity [m/s]
   * @param[in]  far_goal Distant goal position [m]
   * @param[out] path_res Planned path points [m]
   * @return EXPL_RESULT enum
   */
  int planTrackGoal(const Vector3d& pos, const Vector3d& vel,
                    const Vector3d& far_goal, vector<Eigen::Vector3d>& path_res);
  /**
   * Find the optimal global tour through selected viewpoints.
   *
   * @param[in]  vps      Viewpoint list
   * @param[in]  cost_mat Cost matrix between viewpoints [--]
   * @param[in]  cur_pos  Current position [m]
   * @param[in]  cur_yaw  Current yaw [rad]
   * @param[in]  cur_vel  Current velocity [m/s]
   */
  void findVPGlobalTour(std::vector<Viewpoint::Ptr> &vps, const Eigen::MatrixXd& cost_mat,
                        const Eigen::Vector3d &cur_pos, const double & cur_yaw, const Eigen::Vector3d &cur_vel);
  /**
   * Plan exploration within a specific area using LLM-guided scene graph.
   *
   * @param[in]  area_id  Area ID
   * @param[in]  cur_pos  Current position [m]
   * @param[in]  cur_vel  Current velocity [m/s]
   * @param[in]  cur_yaw  Current yaw [rad]
   * @param[in]  cur_poly Current topological polygon
   * @param[out] aim_pos  Target aim position [m]
   * @param[out] aim_yaw  Target aim yaw [rad]
   * @param[out] aim_vel  Target aim velocity [m/s]
   * @param[out] path_res Planned path points [m]
   * @return EXPL_RESULT enum
   */
  int planLLMExploration(const int &area_id, const Eigen::Vector3d &cur_pos,
                         const Eigen::Vector3d cur_vel, const double &cur_yaw,
                         const PolyHedronPtr &cur_poly, Eigen::Vector3d &aim_pos, double &aim_yaw, Eigen::Vector3d &aim_vel, std::vector<Eigen::
                         Vector3d> &path_res);
  /**
   * Find global tour for a subset of selected frontiers.
   *
   * @param[in]  ftr_select Selected frontiers
   * @param[in]  cur_pos    Current position [m]
   * @param[in]  cur_vel    Current velocity [m/s]
   * @param[in]  cur_yaw    Current yaw [rad]
   * @param[in]  cur_poly   Current topological polygon
   * @param[out] indices    Frontier visitation order
   */
  void findGlobalTour_SomeFtrs(std::vector<Frontier>& ftr_select, const Eigen::Vector3d& cur_pos,
                               const Eigen::Vector3d cur_vel, const double& cur_yaw,
                               const PolyHedronPtr& cur_poly, vector<int>& indices);
  
  /**
   * Force-delete a specific frontier from the set.
   *
   * @param[in] ftr  Frontier to delete
   */
  void forceDeleteFrontier(Frontier ftr);
  /**
   * Set the exploration region polygon constraint.
   *
   * @param[in] polygon  Region polygon vertices [m], XY-plane
   * @param[in] enabled  Whether to enable region constraint
   */
  void setExplorationRegion(const std::vector<Eigen::Vector3d>& polygon, bool enabled);
  /**
   * Check whether an exploration region constraint is active.
   *
   * @return True if region constraint is enabled
   */
  bool hasExplorationRegion() const;
  /**
   * Test whether a point lies inside the exploration region polygon (ray casting, XY-plane).
   *
   * @param[in] point  Query point [m]
   * @return True if point is inside the region
   */
  bool isPointInExplorationRegion(const Eigen::Vector3d& point) const;
  /**
   * Check whether a frontier is allowed by the region constraint.
   *
   * @param[in] frontier  Frontier to check
   * @return True if frontier is within the allowed region
   */
  bool frontierAllowedByRegion(const Frontier& frontier) const;
  /**
   * Filter a list of frontiers by the exploration region.
   *
   * @param[in,out] frontiers  Frontier list to filter in-place
   */
  void filterFrontiersByExplorationRegion(std::vector<Frontier>& frontiers) const;
  /**
   * Update the topological blacklist around visited goals.
   *
   * @param[in] goal    Blacklisted goal position [m]
   * @param[in] cur     Current position [m]
   * @param[in] range   Blacklist range [m]
   * @param[in,out] blacklist  Topo node blacklist map
   */
  void updateTopoBlacklist(Eigen::Vector3d goal, Eigen::Vector3d cur, double range,
                           std::unordered_map<int, int> &blacklist);
  /**
   * Get the position of a blacklisted topology node.
   *
   * @param[in] idx  Topology node index
   * @return Position of the blacklisted node [m]
   */
  Eigen::Vector3d getBlacklistTopoPos(int idx);

  /**
   * Update the HGrid occupancy structure.
   */
  void updateHgrid();

  // skeleton update
  PolyHedronPtr cur_mount_topo_{nullptr}, last_mount_topo_{nullptr};
  /**
   * Set the current topological node (polyhedron).
   *
   * @param[in] topo_node  Topology polyhedron pointer
   */
  void setCurrentTopoNode(PolyHedronPtr topo_node);

  /**
   * Publish all visualization markers.
   *
   * @param[in] pos  Current position [m]
   */
  void visualize(const Eigen::Vector3d &pos);
  /**
   * Visualize frontier indices as text markers.
   */
  void visFrontierInx();
  /**
   * Visualize HGrid occupancy grid.
   *
   * @param[in] pos  Current position [m]
   */
  void visHgrid(const Eigen::Vector3d &pos);
  /**
   * Visualize blacklisted topology nodes.
   */
  void visBlacklist();

 private:
  /**
   * Find the optimal global TSP tour for coarse viewpoints of all frontiers.
   *
   * @param[in]  cur_pos   Current position [m]
   * @param[in]  cur_vel   Current velocity [m/s]
   * @param[in]  cur_yaw   Current yaw [rad]
   * @param[out] indices   Frontier visitation order
   * @param[out] cost_mat  Computed cost matrix [--]
   */
  void findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const double& cur_yaw,
                      vector<int>& indices, Eigen::MatrixXd& cost_mat);
  /**
   * Formulate the TSP from the cost matrix.
   *
   * @param[in] cost_mat  Cost matrix [--]
   */
  void TSPFormulate(const Eigen::MatrixXd& cost_mat);
  /**
   * Get raw TSP solution (node IDs in tour order).
   *
   * @param[out] ids  Node ID sequence
   */
  void TSPGetRawRes(vector<int>& ids);
  /**
   * Convert raw TSP IDs to frontier indices.
   *
   * @param[in]  ids      Raw TSP node IDs
   * @param[out] indices  Frontier indices in tour order
   */
  void TSPGetRes(const vector<int>& ids, vector<int>& indices);
  /**
   * Convert raw TSP IDs to indices for a subset of frontiers.
   *
   * @param[in]  ids              Raw TSP node IDs
   * @param[in]  frontier_indices Selected frontier indices
   * @param[out] indices          Reordered frontier indices
   */
  void TSPGetPartialRes(const vector<int>& ids, const vector<int>& frontier_indices, vector<int>& indices);
  /**
   * Solve the TSP for the given cost matrix.
   *
   * @param[in]  cost_mat  Cost matrix [--]
   * @param[out] indices   Frontier visitation order
   */
  void solveTSP(const Eigen::MatrixXd& cost_mat, vector<int>& indices);
  /**
   * Shorten a path by removing redundant colinear points.
   *
   * @param[in,out] path  Path to shorten in-place [m]
   */
  void shortenPath(vector<Vector3d>& path);
  /**
   * Refine local tour by building a viewpose graph and finding the shortest path via Dijkstra.
   *
   * @param[in]  cur_pos     Current position [m]
   * @param[in]  cur_vel     Current velocity [m/s]
   * @param[in]  cur_yaw     Current yaw [rad]
   * @param[in]  n_points    Candidate viewpoints per frontier [m]
   * @param[in]  n_yaws      Candidate yaws per frontier [rad]
   * @param[out] refined_pts Refined path points [m]
   * @param[out] refined_yaws Refined yaws [rad]
   */
  void refineLocalTour(
      const Vector3d& cur_pos, const Vector3d& cur_vel, const double& cur_yaw,
      const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws,
      vector<Vector3d>& refined_pts, vector<double>& refined_yaws);
  /**
   * Plan a path to the next frontier target with multi-layer fallback.
   *
   * @param[in]  pos                Current position [m]
   * @param[in]  next_ftr           Target frontier
   * @param[out] aim_pos            Aim position [m]
   * @param[out] path_res           Planned path [m]
   * @param[in]  force_direct_egoplan Force direct straight-line path
   * @return True if a valid path was found
   */
  bool planNextFtr(const Vector3d& pos, const Frontier& next_ftr, Vector3d& aim_pos, vector<Eigen::Vector3d>& path_res, bool force_direct_egoplan);



 public:
  //! Sub-Class
  FrontierFinder::Ptr               frontier_finder_;
  SceneGraph::Ptr                   scene_graph_;
  ExplorationData::Ptr              ed_;
  ExplorationParam::Ptr             ep_;
  HGrid::Ptr                        hgrid_;
  PLAN_TO_WHAT                      local_aim_type_;
  expl_vis::Visualization::Ptr vis_ptr_;

 private:
  MapInterface::Ptr                 map_;
  ros::Timer                        frontier_timer, goal_timer;
  bool                              has_exploration_region_{false};
  std::vector<Eigen::Vector3d>      exploration_region_polygon_;

};

}  // namespace ego_planner
#endif
