#ifndef _PLANNER_INTERFACE_H_
#define _PLANNER_INTERFACE_H_

#include <Eigen/Eigen>
#include <memory>
#include <string>
#include <vector>

namespace ego_planner {

/**
 * Planner self-reported health and confidence.
 */
struct PlannerHealth {
  enum State { IDLE, RUNNING, DEGRADED, STUCK, FAILED, COMPLETED };

  State   state{IDLE};
  float   confidence{1.0f};
  int     replan_count{0};
  double  time_in_state{0.0};
  std::string message;
};

/**
 * Mission goal specification passed from mission executive to a planner.
 */
struct MissionGoal {
  enum GoalType {
    EXPLORE,
    TRACK,
    GO_TO_OBJECT,
    GO_TO_WAYPOINT,
    YAW_SCAN,
    VLA_SEARCH,
    PANORAMA,
    DF_DEMO,
    FIND_TERMINATE,
  };

  GoalType type;

  Eigen::Vector3d target_position{Eigen::Vector3d::Zero()};
  double          target_yaw{0.0};
  int             target_id{-1};
  int             area_id{-1};
  std::string     text_command;

  std::vector<Eigen::Vector3d> region_polygon;
  bool                         region_enabled{false};

  std::vector<Eigen::Vector3d> waypoints;
  std::vector<double>          waypoint_yaws;
};

/**
 * Abstract interface for all planner backends.
 *
 * Each planner implements this to plug into the mission executive
 * as a swappable strategy. The mission executive uses `getHealth()`
 * and `canHandle()` to decide whether to activate, keep, or fallback
 * away from a given planner.
 */
class PlannerInterface {
 public:
  using Ptr = std::shared_ptr<PlannerInterface>;

  virtual ~PlannerInterface() = default;

  virtual bool   start(const MissionGoal& goal) = 0;
  virtual void   abort() = 0;
  virtual bool   isActive() const = 0;

  virtual bool   canHandle(const MissionGoal& goal) const = 0;
  virtual float  suitabilityScore(const MissionGoal& goal) const = 0;

  virtual PlannerHealth getHealth() const = 0;
};

}  // namespace ego_planner

#endif
