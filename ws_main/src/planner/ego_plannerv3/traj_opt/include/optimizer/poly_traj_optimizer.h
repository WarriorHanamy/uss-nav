#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "optimizer/lbfgs.hpp"
#include <traj_utils/plan_container.hpp>
#include "poly_traj_utils.hpp"
#include <fstream>

namespace ego_planner
{

#define UNKNOWN_COUND_THRES 20

  /**
   * Collision constraint points container.
   *
   * Stores deformation control points with associated obstacle avoidance
   * direction vectors, velocity/acceleration limits, and timing information.
   * Used by PolyTrajOptimizer for collision-free trajectory optimization.
   */
  class ConstraintPoints
  {
  public:
    int cp_size; // number of deformation control points
    Eigen::MatrixXd points;                                                      // control point positions [m]
    std::vector<std::vector<Eigen::Vector3d>> base_point;                       // collision point on obstacle surface [m]
    std::vector<std::vector<Eigen::Vector3d>> direction;                        // normalized repulsion direction [--]
    std::vector<bool> flag_got_pvpair;                                          // whether position-velocity pair is computed
    std::vector<double> vel_limit;                                              // velocity limit per point [m/s]
    std::vector<double> acc_limit;                                              // acceleration limit per point [m/s^2]
    std::vector<double> t;                                                      // time at each constraint point [s]
    std::vector<std::pair<std::pair<Eigen::Vector3d, double>, Eigen::Vector3d>> curve_fitting;
    bool dyn_limit_valid;
    EnterUnknownRegionInfo ent_uk;

    void resize_cp(const int size_set)
    {
      cp_size = size_set;
      dyn_limit_valid = false;

      base_point.clear();
      direction.clear();
      flag_got_pvpair.clear();
      vel_limit.clear();
      acc_limit.clear();
      t.clear();
      curve_fitting.clear();

      points.resize(3, size_set);
      base_point.resize(cp_size);
      direction.resize(cp_size);
      flag_got_pvpair.resize(cp_size);
      vel_limit.resize(cp_size);
      acc_limit.resize(cp_size);
      t.resize(cp_size);
      curve_fitting.resize(cp_size);

      ent_uk.enable = false;
    }

    void segment(ConstraintPoints &buf, const int start, const int end)
    {
      if (start < 0 || end >= cp_size || points.rows() != 3)
      {
        ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
        return;
      }

      buf.resize_cp(end - start + 1);
      buf.points = points.block(0, start, 3, end - start + 1);
      buf.cp_size = end - start + 1;
      for (int i = start; i <= end; i++)
      {
        buf.base_point[i - start] = base_point[i];
        buf.direction[i - start] = direction[i];
      }
    }

    static inline int one_thirds_id(const int cps_nums, const bool use_all)
    {
      return use_all ? cps_nums - 1 : (cps_nums - 2) / 3 + 1;
    }

    static inline int two_thirds_id(const int cps_nums, const bool use_all)
    {
      return use_all ? cps_nums - 1 : cps_nums - 1 - (cps_nums - 2) / 3;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  struct OptFsm
  {
    enum ACTION
    {
      ROUGH_REBOUND,
      FINE_REBOUND,
      SUCCESS_RET,
      FAILED_RET,
      ADJUST_SPEED,
      ADJUST_WEI
    };

    ACTION act = ACTION::FAILED_RET;

    std::string show(void)
    {
      switch (act)
      {
      case ROUGH_REBOUND:
        return std::string("ROUGH_REBOUND");
      case FINE_REBOUND:
        return std::string("FINE_REBOUND");
      case SUCCESS_RET:
        return std::string("SUCCESS_RET");
      case FAILED_RET:
        return std::string("FAILED_RET");
      case ADJUST_SPEED:
        return std::string("ADJUST_SPEED");
      case ADJUST_WEI:
        return std::string("ADJUST_WEI");
      default:
        return std::string("UNKNOWN");
      }
    }
  };

  /**
   * MINCO + L-BFGS trajectory optimizer.
   *
   * Optimizes a multi-piece polynomial trajectory in position flat-output space
   * using the Minimum Control eNergy (MINCO) formulation. Supports collision
   * avoidance (ESDF + obstacle gradient), swarm deconfliction, feasibility
   * enforcement (velocity/acceleration/jerk/snap limits), and multi-topology
   * trajectory generation. Uses L-BFGS solver with bounded step sizes.
   */
  class PolyTrajOptimizer
  {

  private:
    MapManager::Ptr map_;
    dyn_a_star::AStar::Ptr a_star_;
    poly_traj::MinJerkOpt jerkOpt_;
    SwarmTrajData *swarm_trajs_{NULL}; // Can not use shared_ptr and no need to free
    ConstraintPoints cps_;
    std::vector<std::pair<Eigen::Vector3d, double>> restrict_plane_;
    PlanParameters pp_cpy_;

    int drone_id_;
    int cps_num_prePiece_, cps_num_prePiece_Long_; // number of distinctive constraint points each piece
    int variable_num_;                             // number of optimization variables
    int piece_num_;                                // number of polynomial trajectory pieces
    int iter_num_, total_iter_num_;                // L-BFGS solver iteration counters
    std::vector<double> min_ellip_dist2_;          // minimum trajectory distance to swarm agents [m]
    bool touch_goal_;
    struct MultitopologyData_t
    {
      bool use_multitopology_trajs{false};
      bool initial_obstacles_avoided{false};
    } multitopology_data_;
    double maxv_measure_, maxa_measure_, maxj_measure_;
    Eigen::Vector3d uk_vel_measure_, uk_pos_measure_;
    ros::Time opt_start_time_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_obs_, wei_obs_soft_;       // obstacle avoidance weight [--]
    double wei_trust_region_;             // trust region penalty weight [--]
    double wei_curve_fitting_;            // curve fitting weight [--]
    double wei_plane_;                    // restrict plane penalty weight [--]
    double wei_swarm_, wei_swarm_mod_;    // swarm deconfliction weight [--]
    double wei_feas_, wei_feas_mod_;      // dynamic feasibility weight [--]
    double wei_sqrvar_;                   // squared variance weight [--]
    double wei_time_;                     // time regularization weight [--]
    double obs_clearance_, obs_clearance4_, obs_clearance_soft_, swarm_clearance_; // safe clearance distances [m]
    double max_vel_, max_acc_, max_jer_, max_sna_;                                 // dynamic limits [m/s], [m/s^2], [m/s^3], [m/s^4]
    Eigen::Vector3d start_jerk_;

    double t_now_;

  public:
    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    enum CHK_RET
    {
      OBS_FREE,
      ERR,
      FINISH,
      TIME_LIM
    };

    /**
     * Set ROS parameters from the node handle.
     *
     * @param[in] nh  ROS node handle
     */
    void setParam(ros::NodeHandle &nh);
    /**
     * Set the environment map pointer.
     *
     * @param[in] map  Map manager instance
     */
    void setEnvironment(const MapManager::Ptr map);
    /**
     * Set the initial control point positions.
     *
     * @param[in] points  Control point positions matrix (3 x N) [m]
     */
    void setControlPoints(const Eigen::MatrixXd &points);
    /**
     * Set swarm trajectory data for deconfliction.
     *
     * @param[in] swarm_trajs_ptr  Pointer to swarm trajectory data
     */
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    /**
     * Set the drone ID for this optimizer instance.
     *
     * @param[in] drone_id  Drone identifier (>= 0 for swarm, < 0 for single)
     */
    void setDroneId(const int drone_id);
    /**
     * Set whether the trajectory should reach the goal.
     *
     * @param[in] touch_goal  True if trajectory must reach the final goal
     */
    void setIfTouchGoal(const bool touch_goal);
    /**
     * Set constraint points directly.
     *
     * @param[in] cps  Constraint points container
     */
    void setConstraintPoints(ConstraintPoints cps);
    /**
     * Enable or disable multi-topology trajectory support.
     *
     * @param[in] use_multitopology_trajs  True to enable multi-topology
     */
    void setUseMultitopologyTrajs(bool use_multitopology_trajs);
    /**
     * Set maximum velocity and acceleration limits.
     *
     * @param[in] max_vel  Maximum velocity [m/s]
     * @param[in] max_acc  Maximum acceleration [m/s^2]
     */
    void setMaxVelAcc(double max_vel, double max_acc);
    /**
     * Set number of constraint points per polynomial piece.
     *
     * @param[in] N  Constraint points per piece
     */
    void setCPsNumPerPiece(const int N);
    /**
     * Set a copy of the planning parameters.
     *
     * @param[in] pp_cpy  Planning parameters copy
     */
    void setPlanParametersCopy(const PlanParameters &pp_cpy);

    /**
     * Get the current constraint points (control points).
     *
     * @return Reference to constraint points container
     */
    inline const ConstraintPoints &getControlPoints(void) { return cps_; }
    /**
     * Get the current MINCO optimizer instance.
     *
     * @return Reference to MinJerkOpt instance
     */
    inline const poly_traj::MinJerkOpt &getMinJerkOpt(void) { return jerkOpt_; }
    /**
     * Get the number of constraint points per piece.
     *
     * @return Constraint points per piece [--]
     */
    inline int get_cps_num_prePiece_(void) { return cps_num_prePiece_; }
    /**
     * Get the swarm deconfliction clearance distance.
     *
     * @return Clearance distance [m]
     */
    inline double get_swarm_clearance_(void) { return swarm_clearance_; }

    /**
     * Optimize the trajectory (shape + time) using MINCO + L-BFGS.
     *
     * @param[in]  iniState     Initial state (3x3: pos/vel/acc) [m, m/s, m/s^2]
     * @param[in]  finState     Final state (3x3: pos/vel/acc) [m, m/s, m/s^2]
     * @param[in]  initInnerPts Initial inner control points (3 x N) [m]
     * @param[in]  initT        Initial piece durations (N-1) [s]
     * @param[out] final_cost   Final optimization cost [--]
     * @return True if optimization succeeded
     */
    bool optimizeTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                            double &final_cost);

    /**
     * Optimize trajectory shape only (keep piece durations fixed).
     *
     * @param[in]  iniState     Initial state (3x3: pos/vel/acc) [m, m/s, m/s^2]
     * @param[in]  finState     Final state (3x3: pos/vel/acc) [m, m/s, m/s^2]
     * @param[in]  initInnerPts Initial inner control points (3 x N) [m]
     * @param[in]  initT        Fixed piece durations (N-1) [s]
     * @param[in]  CPsNumPerPiece  Constraint points per piece [--]
     * @param[out] final_cost   Final optimization cost [--]
     * @return True if optimization succeeded
     */
    bool optimizeTrajectoryShapeOnly(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                     const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                                     const int CPsNumPerPiece, double &final_cost);

    /**
     * Optimize trajectory time only (keep shape fixed).
     *
     * @param[in]  iniState     Initial state (3x3: pos/vel/acc) [m, m/s, m/s^2]
     * @param[in]  finState     Final state (3x3: pos/vel/acc) [m, m/s, m/s^2]
     * @param[in]  initInnerPts Fixed inner control points (3 x N) [m]
     * @param[in]  initT        Initial piece durations (N-1) [s]
     * @param[out] final_cost   Final optimization cost [--]
     * @return True if optimization succeeded
     */
    bool optimizeTrajectoryTimeOnly(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                    const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                                    double &final_cost);

    /**
     * Compute points along the trajectory for collision checking.
     *
     * @param[in]  traj       Trajectory to check
     * @param[in]  id_end     End piece index [--]
     * @param[out] pts_check  Output check points container
     * @return True if computation succeeded
     */
    bool computePointsToCheck(poly_traj::Trajectory &traj, int id_end, PtsChk_t &pts_check);

    /**
     * Check the trajectory for numerical normality (no NaN/inf).
     *
     * @return True if trajectory is normal
     */
    bool normalityCheck();

    // std::vector<std::pair<int, int>> finelyCheckConstraintPointsOnly(Eigen::MatrixXd &init_points);

    /**
     * Finely check collision for constraint points and set position-velocity pairs.
     *
     * For each constraint point in collision, searches a collision-free path
     * via A* and sets the repulsion direction {base_point, direction}.
     *
     * @param[in]  segments        Piece segments to check
     * @param[out] a_star_pathes   A* collision-free paths per segment
     * @param[in]  pt_data         MINCO trajectory data
     * @param[in]  cps_num_prePiece  Constraint points per piece [--]
     * @param[in]  flag_first_init  True on first initialization
     * @return Check result (OBS_FREE / ERR / FINISH / TIME_LIM)
     */
    CHK_RET finelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments,
                                              vector<vector<Eigen::Vector3d>> &a_star_pathes,
                                              const poly_traj::MinJerkOpt &pt_data,
                                              const int cps_num_prePiece,
                                              const bool flag_first_init /*= true*/);

    /**
     * Roughly check constraint points for collision (without A* search).
     *
     * @return True if all constraint points are collision-free
     */
    bool roughlyCheckConstraintPoints(void);

    /**
     * Try to extend a point in the repulsion direction and check for collision.
     *
     * @param[in] p  Point to extend [m]
     * @param[in] v  Repulsion direction (normalized) [--]
     * @param[in] q  Target point for fallback [m]
     * @return Extended collision-free point [m]
     */
    Eigen::Vector3d tryExtendAndChkP(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d q);

    /**
     * Check whether the optimizer should allow a rebound (re-initialization).
     *
     * @return True if rebound is allowed
     */
    bool allowRebound(void);

    /**
     * Compute per-piece velocity limits from the initial and final states.
     *
     * @param[in] iniState  Initial state (3x3: pos/vel/acc) [m, m/s, m/s^2]
     * @param[in] finState  Final state (3x3: pos/vel/acc) [m, m/s, m/s^2]
     */
    void computeVelLim(const Eigen::MatrixXd &iniState, Eigen::MatrixXd finState);

    /**
     * Prepare fitted curve data for constraint point regularization.
     */
    void prepareFittedCurve();

    /**
     * Generate distinctive trajectory candidates for multi-topology planning.
     *
     * Samples multiple collision-free topologies by perturbing control points
     * in different directions around obstacles.
     *
     * @param[in] segments  Piece segments to generate topologies for
     * @return Vector of distinctive constraint point sets
     */
    std::vector<ConstraintPoints> distinctiveTrajs(vector<std::pair<int, int>> segments);

  private:
    /* callbacks by the L-BFGS optimizer */
    static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);
    static double ShapeOnlyCostFunctionCallback(void *func_data, const double *x, double *grad, const int n);

    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    static double stepSizeBound(void *func_data, const double *xp, const double *d, const int n);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    /* gradient and cost evaluation functions */
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

    void initAndGetSmoothnessGradCost2P(double &cost);

    template <typename EIGENVEC>
    void addPVAJGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);

    void addPGradCost2C(Eigen::VectorXd &costs, const int &N, const int &K);

    /**
     * Obstacle avoidance gradient and cost via ESDF.
     *
     * @param[in]  i_dp   Constraint point index [--]
     * @param[in]  p      Point position [m]
     * @param[out] gradp  Gradient w.r.t. p [--/m]
     * @param[out] costp  Cost contribution [--]
     * @return True if gradient/cost computed successfully
     */
    bool obstacleGradCostP(const int i_dp,
                           const Eigen::Vector3d &p,
                           Eigen::Vector3d &gradp,
                           double &costp);

    /**
     * ESDF-based gradient and cost for obstacle proximity.
     *
     * @param[in]  i_dp   Constraint point index [--]
     * @param[in]  p      Point position [m]
     * @param[out] gradp  Gradient w.r.t. p [--/m]
     * @param[out] costp  Cost contribution [--]
     * @return True if ESDF evaluation succeeded
     */
    bool ESDFGradCostP(const int i_dp,
                       const Eigen::Vector3d &p,
                       Eigen::Vector3d &gradp,
                       double &costp);

    bool restrictplaneGradCostP(const int i_piece,
                                const Eigen::Vector3d &p,
                                Eigen::Vector3d &gradp,
                                double &costp);

    bool cpsDistGradCostP(const int cps_id,
                          const Eigen::Vector3d &p,
                          Eigen::Vector3d &gradp,
                          double &costp);

    bool FixUnknwonPosGradCostP(const int cps_id,
                                const Eigen::Vector3d &p,
                                const Eigen::Vector3d &v,
                                Eigen::Vector3d &gradp,
                                double &costp,
                                Eigen::Vector3d &gradv,
                                double &costv);

    bool CurveFittingGradCostP(const int cps_id,
                               const Eigen::Vector3d &p,
                               Eigen::Vector3d &gradp,
                               double &costp);

    bool swarmGradCostP(const int i_dp,
                        const double t,
                        const Eigen::Vector3d &p,
                        const Eigen::Vector3d &v,
                        Eigen::Vector3d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);

    /**
     * Velocity feasibility gradient and cost.
     *
     * @param[in]  i_dp   Constraint point index [--]
     * @param[in]  v      Velocity [m/s]
     * @param[out] gradv  Gradient w.r.t. v [--/(m/s)]
     * @param[out] costv  Cost contribution [--]
     * @return True if gradient/cost computed
     */
    bool feasibilityGradCostV(const int i_dp,
                              const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);

    /**
     * Acceleration feasibility gradient and cost.
     *
     * @param[in]  i_dp   Constraint point index [--]
     * @param[in]  a      Acceleration [m/s^2]
     * @param[out] grada  Gradient w.r.t. a [--/(m/s^2)]
     * @param[out] costa  Cost contribution [--]
     * @return True if gradient/cost computed
     */
    bool feasibilityGradCostA(const int i_dp,
                              const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);

    /**
     * Jerk feasibility gradient and cost.
     *
     * @param[in]  i_dp   Constraint point index [--]
     * @param[in]  j      Jerk [m/s^3]
     * @param[out] gradj  Gradient w.r.t. j [--/(m/s^3)]
     * @param[out] costj  Cost contribution [--]
     * @return True if gradient/cost computed
     */
    bool feasibilityGradCostJ(const int i_dp,
                              const Eigen::Vector3d &j,
                              Eigen::Vector3d &gradj,
                              double &costj);

    /**
     * Snap feasibility gradient and cost.
     *
     * @param[in]  i_dp   Constraint point index [--]
     * @param[in]  s      Snap [m/s^4]
     * @param[out] grads  Gradient w.r.t. s [--/(m/s^4)]
     * @param[out] costs  Cost contribution [--]
     * @return True if gradient/cost computed
     */
    bool feasibilityGradCostS(const int i_dp,
                              const Eigen::Vector3d &s,
                              Eigen::Vector3d &grads,
                              double &costs);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);

    void lengthVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                      const int n,
                                      Eigen::MatrixXd &gdp,
                                      double &var);

  public:
    typedef unique_ptr<PolyTrajOptimizer> Ptr;
  };

} // namespace ego_planner
#endif