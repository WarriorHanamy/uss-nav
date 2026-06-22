//
// Created by gwq on 8/14/25.
//

#ifndef SCENE_GRAPH_H
#define SCENE_GRAPH_H

#include <ros/ros.h>
#include <ros/package.h>
#include "../include/scene_graph/data_structure.h"
#include "../include/scene_graph/skeleton_generation.h"
#include "../include/scene_graph/object_factory.h"
#include "../include/scene_graph/skeleton_cluster.h"
#include "nlohmann/json.hpp"
#include <fstream>

#include <visualization_msgs/MarkerArray.h>
#include "scene_graph/PromptMsg.h"

class SceneGraphMapIO;

/**
 * SceneGraph runtime data: list of clustered areas.
 */
struct SceneGraph_Data {
std::vector<AreaHandler::Ptr> area;
};

/**
 * VLA + swarm LLM prompt result container.
 *
 * Stores the parsed result of a visual-language-action (VLA) swarm prompt,
 * including success status, error details, and the raw JSON payload.
 */
struct VLASwarmPromptResult {
    bool valid{false};       ///< Whether the result is valid [--]
    bool success{false};     ///< Whether the action succeeded [--]
    std::string error;       ///< Error description
    std::string detail;      ///< Detailed result description
    nlohmann::json payload;  ///< Raw JSON payload from LLM
};

/**
 * Top-level scene graph orchestrator.
 *
 * Manages skeleton generation, object detection/fusion, area clustering,
 * and LLM-based reasoning for autonomous exploration and search.
 * Coordinates SkeletonGenerator, ObjectFactory, and AreaHandler.
 */
class SceneGraph {
public:
    typedef std::shared_ptr<SceneGraph> Ptr;
    SceneGraph(ros::NodeHandle& nh, ego_planner::MapInterface::Ptr& map_interface) {
        nh_ = nh;
        scene_graph_pub_    = nh_.advertise<visualization_msgs::MarkerArray>("/scene_graph/vis", 2);
        prompt_pub_         = nh_.advertise<scene_graph::PromptMsg>("/scene_graph/prompt", 2);
        llm_ans_sub_        = nh_.subscribe("/scene_graph/llm_ans", 2, &SceneGraph::llmAnsCallback, this, ros::TransportHints().tcpNoDelay());
        skeleton_gen_       = std::make_shared<SkeletonGenerator>(nh, map_interface);
        object_factory_     = std::make_unique<ObjectFactory>(nh, skeleton_gen_);
        this_package_path_  = ros::package::getPath("scene_graph");
        INFO_MSG("SceneGraph initialized, package path: " << this_package_path_);
    };
    ~SceneGraph() = default;
    // submodules //
    SkeletonGenerator::Ptr  skeleton_gen_;     ///< Skeleton (free-space decomposition) generator
    ObjectFactory::UPtr     object_factory_;    ///< Object detection and tracking module
    PolyHedronPtr           cur_poly_;          ///< Current polyhedron the robot is in
    std::vector<int>        history_visited_area_ids_; ///< IDs of areas visited in this session

    std::string target_cmd_string_;       ///< Natural language target command
    std::string prior_knowledge_string_;   ///< Prior knowledge for LLM context

    /**
     * Set the target command and prior knowledge string for LLM prompts.
     *
     * @param[in] target_cmd_str       Natural-language target command
     * @param[in] prior_knowledge_str  Prior knowledge / context string
     */
    void setTargetAndPriorKnowledge(const std::string& target_cmd_str, const std::string& prior_knowledge_str);

    /**
     * Mount the current polyhedron at the robot's position and yaw.
     *
     * @param[in] pos  Robot position [m]
     * @param[in] yaw  Robot yaw [rad]
     */
    void mountCurPoly(const Eigen::Vector3d pos, const double yaw);
    PolyHedronPtr getCurPoly() {return cur_poly_;};

    /**
     * Initialize the scene graph from the current position.
     *
     * @param[in] cur_pos  Current robot position [m]
     * @param[in] yaw      Current robot yaw [rad]
     * @return True if initialization succeeded
     */
    bool initSceneGraph(const Eigen::Vector3d &cur_pos, double yaw);
    /**
     * Update the scene graph with the current robot state.
     *
     * @param[in]  cur_pos  Current robot position [m]
     * @param[in]  yaw      Current robot yaw [rad]
     * @param[out] new_topo  Whether new topology (polyhedron/frontier) was discovered
     */
    void updateSceneGraph(const Eigen::Vector3d &cur_pos, const double &yaw, bool &new_topo);
    /**
     * Update object positions and associations in the scene graph.
     */
    void updateObjectToSceneGraph();
    /**
     * Get a path to an object by its ID.
     *
     * @param[in]  id       Object ID [--]
     * @param[out] path     Path waypoints to the object [m]
     * @param[out] aim_pos  Target position for yaw aiming [m]
     * @param[out] aim_yaw  Target yaw [rad]
     * @return True if a path was found
     */
    bool getPathToObjectWithId(const int &id, std::vector<Eigen::Vector3d> &path, Eigen::Vector3d & aim_pos, double &aim_yaw);

    /**
     * Send a prompt to the LLM and return a future for the response.
     *
     * @param[in] prompt_id    Unique prompt identifier [--]
     * @param[in] prompt_type  Prompt type code [--]
     * @param[in] prompt_str   Prompt text string
     * @param[in] timeout      Maximum wait time for response [s]
     * @param[in] max_retries  Maximum retry count on failure [--]
     * @return Future containing the LLM response string
     */
    std::future<std::string> sendPrompt(unsigned int prompt_id, unsigned char prompt_type, std::string prompt_str,
                                        const std::chrono::seconds &timeout, int max_retries);
    int wait_recv_id_;
    /**
     * Check whether a prompt answer has been received.
     *
     * @param[in] prompt_id  Prompt identifier [--]
     * @return True if an answer exists
     */
    bool hasPromptAnswer(unsigned int prompt_id);
    void clearPromptData(unsigned int prompt_id);

    template<typename T>
    bool waitForFutureWithSpinOnce(std::future<T>& future, const ros::Duration& timeout);

    // prompt generation //
    bool allRoomPredictionPromptGen(std::string &prompt_str);
    bool singleRoomPredictionPromptGen(const int room_id, nlohmann::json &prompt_json);
    bool newAreaPredictionPromptGen(std::string &prompt_str);
    bool chooseAreaToGoPromptGen(std::string &prompt_str);
    bool chooseTerminateObjIdPromptGen(std::string &prompt_str);
    bool DFDemoPromptGen(std::string &prompt_str);
    void sendSceneGraphJson(std::string &scene_graph_json_str);
    bool vlaSwarmPromptGen(unsigned char prompt_type, const std::string &command,
                           uint32_t task_session_id, uint32_t observation_batch_id,
                           std::string &prompt_str) const;

    // result handle //
    void handleRoomPredictionResult(unsigned int prompt_id);
    int handelExplorationResult(unsigned int prompt_id);
    int handelTerminateObjIdResult(unsigned int prompt_id);
    int handelDFDemoResult(unsigned int prompt_id);
    VLASwarmPromptResult parseVlaSwarmPromptResult(unsigned int prompt_id,
                                                   unsigned char expected_prompt_type);

    // data operations //
    unsigned int getCurPromptIdAndPlusOne(){std::lock_guard<std::mutex> lock(mutex_); return cur_prompt_id_++; }
    unsigned int getCurPromptId(){return cur_prompt_id_;}
    int getAreaFromPoly(const PolyHedronPtr& poly){return poly->area_id_;}
    bool needAreaPrediction(){ return !skeleton_gen_->area_handler_->areas_need_predict_.empty();}
    /**
     * Save the current scene graph map to disk.
     *
     * @param[in] save_name  File name (empty = auto-generated timestamp)
     * @return True if save succeeded
     */
    bool saveMap(const std::string& save_name = "");
    /**
     * Load a scene graph map from disk.
     *
     * @param[in] save_name  File name
     * @return True if load succeeded
     */
    bool loadMap(const std::string& save_name);

    void refreshLoadedMapVisualization();
    void visualizeSceneGraph();

private:
    friend class SceneGraphMapIO;
    ros::NodeHandle        nh_;
    ros::Publisher         scene_graph_pub_;
    std::mutex             mutex_;

    // LLM interface //
    std::string            this_package_path_;
    ros::Publisher         prompt_pub_;
    ros::Subscriber        llm_ans_sub_;
    unsigned int           cur_prompt_id_ = 0;
    bool                   need_area_prediction_ = false;

    std::map<unsigned int, std::promise<std::string>> llm_ans_promises_;
    void llmAnsCallback(const scene_graph::PromptMsg::ConstPtr& msg);
};

/**
 * @brief 等待一个 std::future，在等待期间通过调用 ros::spinOnce() 来处理回调。
 * @tparam T future 的返回类型。
 * @param future 要等待的 future 对象。
 * @param timeout 等待的超时时间。
 * @return 如果在超时时间内成功收到结果，则返回 true；否则返回 false。
 */
template<typename T>
bool SceneGraph::waitForFutureWithSpinOnce(std::future<T>& future, const ros::Duration& timeout)
{
    ros::Time start_time = ros::Time::now();
    while (ros::ok())
    {
        if (ros::Time::now() - start_time > timeout)
        {
            return false; // 超时失败
        }
        // 2. 检查 future 是否就绪 (使用0秒等待实现非阻塞检查)
        auto status = future.wait_for(std::chrono::seconds(0));
        if (status == std::future_status::ready)
        {
            return true; // 成功
        }
        ros::spinOnce();
        ros::WallDuration(0.01).sleep(); // 休眠10毫秒
    }

    return false;
}

#endif //SCENE_GRAPH_H
