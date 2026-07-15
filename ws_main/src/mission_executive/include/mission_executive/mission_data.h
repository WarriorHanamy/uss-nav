#ifndef _MISSION_DATA_H_
#define _MISSION_DATA_H_

#include <Eigen/Eigen>
#include <unordered_map>
#include <vector>

using std::unordered_map;
using std::vector;
using Eigen::Vector3d;

namespace mission_executive {

/**
 * FSM states for the exploration mission executive.
 */
enum MISSION_FSM_STATE
{
  INIT,
  WAIT_TRIGGER,
  WARM_UP,
  PLAN_EXPLORE,
  LLM_PLAN_EXPLORE,
  APPROACH_EXPLORE,
  PLAN_TRACK,
  APPROACH_TRACK,
  THINKING,
  YAW_HANDLE,
  FIND_TERMINATE_TARGET,
  FINISH,
  STOP,
  UNKONWN,
  GO_TARGET_OBJECT,
  GO_TARGET_WITH_WAYPOINT,
  DF_DEMO,
  VLA_SEARCH_PLAN_LOCAL,
  VLA_SEARCH_WAIT_LLM,
  VLA_SEARCH_WAIT_TARGET,
  VLA_SEARCH_APPROACH,
  VLA_SEARCH_YAW_HANDLE,
  VLA_SEARCH_RECOVERY,
  VLA_SEARCH_FINISH
};

/**
 * Mission data containing swarm identity and current mission state.
 */
struct MissionData 
{
  int                     drone_id_;
  int                     swarm_id_;
  vector<int>             swarm_mate_;
  bool                    is_leader_;
  bool                    is_follower_;
  bool                    is_initialized_;

  MISSION_FSM_STATE       mission_state_;
  int                     instruction_;
  unordered_map<MISSION_FSM_STATE, std::string> state_str_;
};


}  // namespace mission_executive

#endif
