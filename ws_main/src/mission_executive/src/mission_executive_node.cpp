#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <plan_manage/ego_replan_fsm.h>
#include <mission_executive/mission_fsm.h>

#include <memory>

using namespace mission_executive;

/**
 * ROS node entry point for the exploration FSM.
 *
 * Initializes the EGO replan FSM, map interface, and exploration FSM.
 *
 * @param[in] argc  Argument count
 * @param[in] argv  Argument vector
 * @return Exit code
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mission_executive_node");
  ros::NodeHandle nh("~");

  ego_planner::EGOReplanFSM rebo_replan;
  rebo_replan.init(nh);

  ego_planner::MapInterface::Ptr map_;
  map_ = std::make_shared<ego_planner::MapInterface>(nh, rebo_replan.getMapPtr());

  MissionFSM expl_fsm;
  expl_fsm.init(nh, map_);

  ros::spin();
  return 0;
}
