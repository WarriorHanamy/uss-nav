#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <optimizer/poly_traj_utils.hpp>

using std::vector;

namespace ego_planner
{

  typedef std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> PtsChk_t;

  /**
   * Unknown region entry information.
   *
   * Tracks whether the trajectory enters unmapped (unknown) space
   * and the associated constraint point index.
   */
  struct EnterUnknownRegionInfo
  {
    bool enable{false};         // whether unknown region entry is active
    int cps_id;                 // constraint point ID entering unknown region
    int K;                      // piece index at entry
    Eigen::Vector3d fixed_pos;  // fixed position at unknown region boundary [m]
  };

  /**
   * Global reference trajectory data.
   *
   * Stores the global planning trajectory and synchronization
   * timestamps for local target selection.
   */
  struct GlobalTrajData
  {
    poly_traj::Trajectory traj;
    double global_start_time; // world time at trajectory start [s]
    double duration;          // total trajectory duration [s]

    /* Global traj time.
       The corresponding global trajectory time of the current local target.
       Used in local target selection process */
    double glb_t_of_lc_tgt;            // [s]
    /* Global traj time.
       The corresponding global trajectory time of the last local target.
       Used in initial-path-from-last-optimal-trajectory generation process */
    double last_glb_t_of_lc_tgt;       // [s]
  };

  /**
   * Local trajectory data for a single drone.
   */
  struct LocalTrajData
  {
    poly_traj::Trajectory traj;
    int drone_id; // negative value indicates no received trajectories
    int traj_id;
    double duration;           // trajectory duration [s]
    double start_time{0.0};   // world time at trajectory start [s]
    double end_time;          // world time at trajectory end [s]
    double last_opt_cp_time;  // last optimization constraint point time [s]
    Eigen::Vector3d start_pos; // start position [m]
    double des_clearance;      // desired clearance [m]
    EnterUnknownRegionInfo uk_info;
  };

  typedef std::vector<LocalTrajData> SwarmTrajData;

  /**
   * Container for local and swarm trajectory data.
   */
  class TrajContainer
  {
  public:
    LocalTrajData local_traj;
    SwarmTrajData swarm_traj;

    TrajContainer()
    {
      local_traj.traj_id = 0;
    }
    ~TrajContainer() {}

    void setLocalTraj(const poly_traj::Trajectory &trajectory, const double last_opt_cp_time_in, const double &world_time,
                      const int drone_id = -1, const EnterUnknownRegionInfo *ent_uk_in = NULL)
    {
      local_traj.drone_id = drone_id;
      local_traj.traj_id++;
      local_traj.duration = trajectory.getTotalDuration();
      local_traj.start_pos = trajectory.getJuncPos(0);
      local_traj.start_time = world_time;
      local_traj.traj = trajectory;
      local_traj.last_opt_cp_time = last_opt_cp_time_in;
      if (ent_uk_in != NULL)
        local_traj.uk_info = *ent_uk_in;
      else
        local_traj.uk_info.enable = false;
    }
  };

  /**
   * Planning algorithm parameters.
   *
   * Stores physical limits, speed mode, and timing statistics
   * for the trajectory planning pipeline.
   */
  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_{1.0}, max_acc_{2.0};                               // physical limits [m/s], [m/s^2]
    double max_vel_user_ = max_vel_, max_acc_user_ = max_acc_;         // user-specified limits [m/s], [m/s^2]
    double max_vel_prevplan_ = max_vel_, max_acc_prevplan_ = max_acc_; // previous planning limits [m/s], [m/s^2]
    bool desvel_changed_toomuch_{false};
    double polyTraj_piece_length; // distance between adjacent polynomial pieces [m]
    double planning_horizen_;     // planning horizon [m]
    bool use_multitopology_trajs;
    bool touch_goal;
    bool emergency_{false}; // ignore acc/jerk/snap limits in emergency [--]
    int drone_id;           // single: drone_id <= -1, swarm: drone_id >= 0 [--]
    enum MODE
    {
      SLOW,
      FAST
    } speed_mode{SLOW};

    /* processing time */
    double time_search_ = 0.0;    // A* search time [s]
    double time_optimize_ = 0.0;  // optimization time [s]
    double time_adjust_ = 0.0;    // adjustment time [s]
  };

  /**
   * Density evaluation ray data.
   *
   * Stores the result of evaluating environment density along a ray
   * from start_p toward end_p, used for adaptive velocity selection.
   */
  struct DensityEvalRayData
  {
    Eigen::Vector3d start_p{Eigen::Vector3d::Zero()}; // ray start position [m]
    Eigen::Vector3d mid_p{Eigen::Vector3d::Zero()};   // mid-point (may extend beyond end_p) [m]
    Eigen::Vector3d end_p{Eigen::Vector3d::Zero()};   // ray end or collision point [m]
    double safe_l{0.0};          // safe length along ray [m]
    double safe_margin{-std::numeric_limits<double>::max()}; // safety margin [m]
    bool safe{false};             // whether the ray is fully safe [--]
    double score{-std::numeric_limits<double>::max()}; // density score [--]
    double norm_devi{std::numeric_limits<double>::max()}; // normalized deviation [--]
    bool full_speed{false};       // full speed allowed [--]
    double preferred_speed{0.0};  // preferred cruise velocity [m/s]
    double time{0};               // evaluation timestamp [s]

    bool operator<(DensityEvalRayData b)
    {
      return this->preferred_speed < b.preferred_speed;
    }

    bool operator>(DensityEvalRayData b)
    {
      return this->preferred_speed > b.preferred_speed;
    }
  };

  class TicToc
  {
  public:
    ros::Time t0;

    TicToc() { t0 = ros::Time::now(); }
    void tic() { t0 = ros::Time::now(); }
    double toc(bool print = true)
    {
      double t_passed = (ros::Time::now() - t0).toSec() * 1000;
      if (print)
        ROS_INFO("passed time = %f ms", t_passed);
      return t_passed;
    }
  };

} // namespace ego_planner

#endif