#ifndef _SITUATION_AWARENESS_H_
#define _SITUATION_AWARENESS_H_

#include <Eigen/Eigen>
#include <ros/ros.h>

namespace ego_planner {

/**
 * Quality assessment of incoming odometry.
 */
struct OdomQuality {
  bool   valid{false};
  bool   stale{false};
  double last_update_age{0.0};
  double max_age{5.0};
  double velocity_norm{0.0};
  double yaw_rate{0.0};
};

/**
 * Quality metrics for the current occupancy map.
 */
struct MapQuality {
  bool   initialized{false};
  int    frontier_count{0};
  double frontier_density{0.0};
  bool   degraded{false};
};

/**
 * Battery state summary for safety decisions.
 */
struct BatteryLevel {
  double voltage{0.0};
  double percentage{0.0};
  bool   low{false};
};

/**
 * Progress tracking for the current mission/planner.
 */
struct MissionProgress {
  int    replan_count{0};
  double time_in_state{0.0};
  double distance_to_goal{0.0};
  bool   made_progress{true};
};

/**
 * Aggregated situational context for the decision engine.
 *
 * Built from sensor callbacks and planner health reports,
 * this is the single source of truth the MissionFSM consults
 * to decide which planner to activate and whether to fallback.
 */
struct SituationAwareness {
  OdomQuality      odom;
  MapQuality       map;
  BatteryLevel     battery;
  MissionProgress  progress;
  PlannerHealth    active_planner;

  void updateOdom(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                  double yaw, double yaw_rate, const ros::Time& stamp) {
    odom.velocity_norm = vel.norm();
    odom.yaw_rate  = yaw_rate;
    odom.last_update_age  = (ros::Time::now() - stamp).toSec();
    odom.stale     = (odom.last_update_age > odom.max_age);
    odom.valid     = (!odom.stale && odom.velocity_norm < 100.0);
  }

  void updateMap(bool init, int frontier_cnt) {
    map.initialized    = init;
    map.frontier_count = frontier_cnt;
    map.degraded       = (init && frontier_cnt < 3);
  }

  void updateBattery(double voltage, double percentage) {
    battery.voltage    = voltage;
    battery.percentage = percentage;
    battery.low        = (voltage < 19.0);
  }
};

}  // namespace ego_planner

#endif
