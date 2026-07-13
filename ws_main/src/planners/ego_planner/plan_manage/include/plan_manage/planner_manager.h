#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/DataDisp.h>
#include <plan_env/grid_map.h>
#include <traj_utils/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>
#include <optimizer/poly_traj_utils.hpp>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called
  enum PLAN_RET
  {
    SUCCESS = 0,
    LOCAL_TGT_FAIL,
    INIT_FAIL,
    DEFAULT_FAIL
  };

  /**
   * Top-level planner orchestrator.
   *
   * Manages the planning pipeline: state initialization, trajectory optimization
   * via MINCO + L-BFGS, collision checking, density evaluation, and emergency stop.
   * Owns MapManager (environment), TrajContainer (trajectory storage), and
   * PolyTrajOptimizer (optimization core).
   */
  class EGOPlannerManager
  {
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PlanParameters pp_;
    MapManager::Ptr map_;
    TrajContainer traj_;
    std::list<DensityEvalRayData> his_dendat_; // historical density data

    /**
     * Initialize all planning modules (map, optimizer, A*, visualization).
     *
     * @param[inout] nh  ROS node handle
     * @param[in] vis  Visualization instance
     */
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);
    /**
     * Compute the initial state for the trajectory optimizer.
     *
     * @param[in]  start_pt           Start position [m]
     * @param[in]  start_vel          Start velocity [m/s]
     * @param[in]  start_acc          Start acceleration [m/s^2]
     * @param[in]  glb_start_pt       Global start position [m]
     * @param[in]  final_goal         Final target position [m]
     * @param[in]  flag_use_last_optimial  Use last optimal trajectory as initial guess
     * @param[in]  flag_random_init   Randomize initial control points
     * @param[out] pathes             Density evaluation ray data
     * @param[out] initMJO            Initial MINCO optimizer state
     * @param[out] touch_goal         Whether the trajectory reaches the goal
     * @return True if initialization succeeds
     */
    bool computeInitState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &glb_start_pt,
        const Eigen::Vector3d &final_goal, const bool flag_use_last_optimial,
        const bool flag_random_init, vector<DensityEvalRayData> *pathes,
        poly_traj::MinJerkOpt &initMJO, bool &touch_goal);
    /**
     * Compute initial trajectory duration from start to local target.
     *
     * @param[in] start_pt         Start position [m]
     * @param[in] start_vel        Start velocity [m/s]
     * @param[in] local_target_pt  Local target position [m]
     * @param[in] local_target_vel Local target velocity [m/s]
     * @return Initial duration guess [s]
     */
    double computeInitDuration(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel);
    /**
     * Compute planning parameters (piece length, clearance, weights) for the given velocity.
     *
     * @param[in] vel  Current velocity magnitude [m/s]
     * @return True if parameters computed successfully
     */
    bool computePlanningParams(const double vel);
    /**
     * Compute planning horizon distance for the given velocity.
     *
     * @param[in] vel  Current velocity magnitude [m/s]
     * @return True if horizon computed successfully
     */
    bool computePlanningHorizon(const double vel);
    /**
     * Compute MINCO optimizer parameters for the given horizon and velocity.
     *
     * @param[in] planning_horizon  Planning horizon distance [m]
     * @param[in] vel               Current velocity magnitude [m/s]
     * @return True if parameters computed successfully
     */
    bool computeMINCOParams(const double planning_horizon, const double vel);
    /**
     * Main receding-horizon trajectory optimization entry point.
     *
     * Runs the full rebound planning loop: initial path search, trajectory
     * optimization, collision checking, and constraint point refinement.
     *
     * @param[in]  start_pt                Start position [m]
     * @param[in]  start_vel               Start velocity [m/s]
     * @param[in]  start_acc               Start acceleration [m/s^2]
     * @param[in]  start_jerk              Start jerk [m/s^3]
     * @param[in]  glb_start_pt            Global start position [m]
     * @param[in]  final_goal              Final target position [m]
     * @param[in]  flag_use_last_optimial  Use last optimal trajectory as initial guess
     * @param[in]  flag_random_init        Randomize initial control points
     * @param[out] pathes                  Density evaluation ray data
     * @param[out] touch_goal              Whether the trajectory reaches the goal
     * @return Plan result (SUCCESS / LOCAL_TGT_FAIL / INIT_FAIL / DEFAULT_FAIL)
     */
    PLAN_RET reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &start_jerk,
        const Eigen::Vector3d &glb_start_pt, const Eigen::Vector3d &final_goal,
        const bool flag_use_last_optimial, const bool flag_random_init,
        vector<DensityEvalRayData> *pathes, bool &touch_goal);
    /**
     * Evaluate environment density along a ray from start to end.
     *
     * @param[in]  start_pt  Ray start position [m]
     * @param[in]  end_pt    Ray end position [m]
     * @param[out] best_ray  Best density ray result
     * @param[out] all_rays  All density ray results
     * @return True if evaluation succeeded
     */
    bool densityEval(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt,
                     DensityEvalRayData *best_ray = NULL, vector<DensityEvalRayData> *all_rays = NULL) const;
    /**
     * Determine cruise velocity from density evaluation results.
     *
     * @param[inout] best_ray  Best density ray result, updated with preferred speed
     * @return True if velocity determined successfully
     */
    bool DetVelByDensity(DensityEvalRayData &best_ray);
    /**
     * Execute emergency stop at the given position.
     *
     * @param[in] stop_pos  Braking target position [m]
     * @return True if stop trajectory generated successfully
     */
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    /**
     * Check for collision with another drone in the swarm.
     *
     * @param[in] drone_id  Swarm drone ID of the other agent
     * @return True if collision detected
     */
    bool checkCollision(int drone_id);
    /**
     * Generate a single-piece trajectory between two positions (zero velocity at both ends).
     *
     * @param[in] start_pos  Start position [m]
     * @param[in] end_pos    End position [m]
     * @return True if generation succeeded
     */
    bool OnePieceTrajGen(Eigen::Vector3d start_pos, Eigen::Vector3d end_pos);
    /**
     * Generate a single-piece trajectory with full boundary conditions.
     *
     * @param[in] start_pos  Start position [m]
     * @param[in] start_vel  Start velocity [m/s]
     * @param[in] start_acc  Start acceleration [m/s^2]
     * @param[in] end_pos    End position [m]
     * @param[in] end_vel    End velocity [m/s]
     * @param[in] end_acc    End acceleration [m/s^2]
     * @param[in] duration   Trajectory duration [s]
     * @return Generated multi-piece trajectory
     */
    poly_traj::Trajectory OnePieceTrajGen(
        Eigen::Vector3d start_pos, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
        Eigen::Vector3d end_pos, Eigen::Vector3d end_vel, Eigen::Vector3d end_acc, double duration);
    /**
     * Generate a random midpoint between start and end for multi-topology initialization.
     *
     * @param[in] start_pt  Start position [m]
     * @param[in] end_pt    End position [m]
     * @return Random midpoint position [m]
     */
    Eigen::Vector3d GenRandomMidPt(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt);
    /**
     * Find a nearby safe (collision-free) position from an unsafe point.
     *
     * @param[in]  unsafe_pt  Unsafe position [m]
     * @param[in]  max_grid   Maximum search radius [voxel]
     * @param[out] safe_pt    Safe position [m]
     * @return True if a safe point is found
     */
    bool getNearbySafePt(const Eigen::Vector3d unsafe_pt, const int max_grid, Eigen::Vector3d &safe_pt);
    /**
     * Set local trajectory from MINCO optimizer result.
     *
     * @param[in] opt          MINCO optimizer result
     * @param[in] touch_goal   Whether the trajectory reaches the goal
     * @param[in] set_uk_info  Whether to set unknown region info
     * @return True if trajectory set successfully
     */
    bool setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal, const bool set_uk_info = false);
    inline double getSwarmClearance(void) { return poly_traj_opt_->get_swarm_clearance_(); }
    inline int getCpsNumPrePiece(void) { return poly_traj_opt_->get_cps_num_prePiece_(); }
    inline int getContinousFailureCount(void) { return continous_failures_count_; }

  private:
    PlanningVisualization::Ptr visualization_;

    PolyTrajOptimizer::Ptr poly_traj_opt_;

    int continous_failures_count_{0}, success_cnt_{0}, failure_cnt_{0};
    double sum_success_time_{0.0};

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif