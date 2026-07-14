#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <plan_manage/ego_replan_fsm.h>
#include <mission_executive/fast_exploration_fsm.h>

#include <memory>

using namespace ego_planner;

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
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");

  EGOReplanFSM rebo_replan;
  rebo_replan.init(nh);

  MapInterface::Ptr map_;
  map_ = std::make_shared<MapInterface>(nh, rebo_replan.getMapPtr());

  FastExplorationFSM expl_fsm;
  expl_fsm.init(nh, map_);

  ros::spin();
  return 0;
}
