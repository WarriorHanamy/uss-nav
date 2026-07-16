#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <global_belief/grid_map.h>
#include <global_belief/map_interface.hpp>
#include <plan_manage/ego_replan_fsm.h>
#include <mission_executive/mission_fsm.h>

#include <memory>

using namespace mission_executive;

/**
 * ROS node entry point for the mission executive FSM.
 *
 * Initializes GlobalBelief for high-level scene understanding,
 * EGO replan FSM for trajectory execution, and mission FSM.
 *
 * @param[in] argc  Argument count
 * @param[in] argv  Argument vector
 * @return Exit code
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mission_executive_node");
  ros::NodeHandle nh("~");

  global_belief::MapManager::Ptr global_mm = std::make_shared<global_belief::MapManager>();
  global_mm->initMapManager(nh);

  global_belief::MapInterface::Ptr map_;
  map_ = std::make_shared<global_belief::MapInterface>(nh, global_mm);

  bool enable_ego_replan = true;
  nh.param("fsm/enable_ego_replan", enable_ego_replan, true);

  std::unique_ptr<ego_planner::EGOReplanFSM> rebo_replan;
  if (enable_ego_replan) {
    rebo_replan = std::make_unique<ego_planner::EGOReplanFSM>();
    rebo_replan->init(nh);
  } else {
    ROS_WARN("[MissionFSM] EGOReplanFSM disabled; external local planner must consume local_goal and publish execution feedback.");
  }

  MissionFSM fsm;
  fsm.init(nh, map_);

  ros::spin();
  return 0;
}
