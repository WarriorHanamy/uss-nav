#include <scene_graph/counting_scene_graph.h>

#include <algorithm>

#include <nlohmann/json.hpp>

CountingSceneGraph::CountingSceneGraph(ros::NodeHandle& nh): nh_(nh) {
    std::string json_topic;
    nh_.param<std::string>(
        "counting_scene_graph/json_topic",
        json_topic,
        "/counting_scene_graph/json_text"
    );
    json_pub_ = nh_.advertise<std_msgs::String>(json_topic, 2);
    emergency_stop_sub_ = nh_.subscribe(
        "/command/emergency_stop",
        2,
        &CountingSceneGraph::emergencyStopCallback,
        this
    );

    // Counting 图只复用对象检测与跨帧融合，不创建 skeleton、area 或 topology。
    object_factory_ = std::make_unique<ObjectFactory>(
        nh_,
        "counting_obj",
        "/counting_scene_graph"
    );
    object_factory_->stopThisModule();
}

void CountingSceneGraph::emergencyStopCallback(const std_msgs::Empty::ConstPtr&) {
    cancelSession();
}

void CountingSceneGraph::startSession(
    uint32_t session_id,
    const Eigen::Vector3d& start_pos
) {
    session_id_ = session_id;
    start_pos_ = start_pos;
    object_factory_->startFreshSession();
    active_ = true;
    ROS_INFO(
        "[CountingSceneGraph] Session %u started at [%.3f, %.3f, %.3f].",
        session_id_,
        start_pos_.x(),
        start_pos_.y(),
        start_pos_.z()
    );
}

void CountingSceneGraph::cancelSession() {
    if (!active_) return;
    object_factory_->cancelSession();
    active_ = false;
    ROS_WARN("[CountingSceneGraph] Session %u cancelled.", session_id_);
}

bool CountingSceneGraph::finishSessionAndPublish() {
    if (!active_) return false;

    const auto objects = object_factory_->stopAndSnapshot();
    nlohmann::json data;
    data["session_id"] = session_id_;
    data["start_pos"] = {start_pos_.x(), start_pos_.y(), start_pos_.z()};
    data["objects"] = nlohmann::json::array();

    for (const auto& object : objects) {
        if (object == nullptr) continue;
        nlohmann::json object_json;
        object_json["id"] = object->id;
        object_json["label"] = object->label;
        object_json["pos"] = {
            object->pos.x(),
            object->pos.y(),
            object->pos.z()
        };
        object_json["confidence"] = object->conf;
        object_json["detection_count"] = object->detection_count;
        data["objects"].push_back(object_json);
    }

    std_msgs::String msg;
    msg.data = data.dump();
    json_pub_.publish(msg);
    active_ = false;
    ROS_INFO(
        "[CountingSceneGraph] Session %u published with %zu object(s).",
        session_id_,
        objects.size()
    );
    return true;
}
