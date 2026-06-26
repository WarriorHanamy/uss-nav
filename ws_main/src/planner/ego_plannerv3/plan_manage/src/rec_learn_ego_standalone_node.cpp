#define REC_LEARN

/**
 * REC_LEARN: minimal ego-planner node stripping FastExplorationFSM,
 * scene_graph, YOLOE, and active_perception dependencies.
 *
 * Replaces exploration_manager/exploration_node for Docker test
 * environments where the full dependency chain is unavailable.
 */

#include <ros/ros.h>
#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rec_learn_ego_planner_node");
    ros::NodeHandle nh("~");

    EGOReplanFSM replan_fsm;
    replan_fsm.init(nh);

    ROS_INFO("[REC_LEARN] Ego standalone node ready.");
    ros::spin();
    return 0;
}
