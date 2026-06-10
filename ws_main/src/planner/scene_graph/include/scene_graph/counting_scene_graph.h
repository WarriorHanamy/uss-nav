#ifndef COUNTING_SCENE_GRAPH_H
#define COUNTING_SCENE_GRAPH_H

#include <cstdint>
#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <scene_graph/object_factory.h>

class CountingSceneGraph {
public:
    typedef std::shared_ptr<CountingSceneGraph> Ptr;

    explicit CountingSceneGraph(ros::NodeHandle& nh);

    void startSession(uint32_t session_id, const Eigen::Vector3d& start_pos);
    void cancelSession();
    bool finishSessionAndPublish();
    bool active() const { return active_; }
    uint32_t sessionId() const { return session_id_; }

private:
    ros::NodeHandle nh_;
    ros::Publisher json_pub_;
    ros::Subscriber emergency_stop_sub_;
    ObjectFactory::UPtr object_factory_;
    bool active_{false};
    uint32_t session_id_{0};
    Eigen::Vector3d start_pos_{Eigen::Vector3d::Zero()};

    void emergencyStopCallback(const std_msgs::Empty::ConstPtr& msg);
};

#endif
