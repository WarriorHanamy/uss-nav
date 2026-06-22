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

/**
 * Counting / demo scene graph session manager.
 *
 * Manages counting/demo object detection sessions: starting a session
 * with a given position, collecting object data via ObjectFactory,
 * and publishing results on session completion.
 */
class CountingSceneGraph {
public:
    typedef std::shared_ptr<CountingSceneGraph> Ptr;

    explicit CountingSceneGraph(ros::NodeHandle& nh);

    /**
     * Start a new counting session.
     *
     * @param[in] session_id  Session identifier [--]
     * @param[in] start_pos   Session start position [m]
     */
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
    bool active_{false};                ///< Whether a session is active [--]
    uint32_t session_id_{0};            ///< Current session ID [--]
    Eigen::Vector3d start_pos_{Eigen::Vector3d::Zero()}; ///< Session start position [m]

    void emergencyStopCallback(const std_msgs::Empty::ConstPtr& msg);
};

#endif
