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
struct VLASearchPromptResult {
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
    SceneGraph(ros::NodeHandle& nh, global_belief::MapInterface::Ptr& map_interface) {
        nh_ = nh;
        map_interface_      = map_interface;
        scene_graph_pub_    = nh_.advertise<visualization_msgs::MarkerArray>("/scene_graph/vis", 2, true);
        prompt_pub_         = nh_.advertise<scene_graph::PromptMsg>("/scene_graph/prompt", 2);
        llm_ans_sub_        = nh_.subscribe("/scene_graph/llm_ans", 2, &SceneGraph::llmAnsCallback, this, ros::TransportHints().tcpNoDelay());
        skeleton_gen_       = std::make_shared<SkeletonGenerator>(nh, map_interface);
        object_factory_     = std::make_unique<ObjectFactory>(nh, skeleton_gen_);
        this_package_path_  = ros::package::getPath("scene_graph");
        // topo-block unreachable detection/repair/marking params (overridable by YAML/launch)
        nh_.param("topo_block/enable",                  topo_block_enable_,             true);
        nh_.param("topo_block/repair_radius",           topo_repair_radius_,            0.5);
        nh_.param("topo_block/repair_vis_mode",         topo_repair_vis_mode_,           0);
        nh_.param("topo_block/repair_vis_sphere_radius", topo_repair_vis_sphere_radius_, 2.0);
        nh_.param("topo_block/hits_thresh",             topo_block_hits_thresh_,        2);
        nh_.param("topo_block/ttl",                     topo_block_ttl_,                8.0);
        nh_.param("topo_block/revalidate_on_fail",      topo_block_revalidate_on_fail_, true);
        nh_.param("topo_block/max_iter",                topo_block_max_iter_,            4);
        nh_.param("topo_block/repair_insert_node",     topo_repair_insert_node_,        false);
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

    /**
     * Get the current polyhedron the robot occupies.
     *
     * @return Pointer to the current polyhedron
     */
    PolyHedronPtr getCurPoly() {return cur_poly_;};

    /**
     * Get the repair visualization mode.
     *
     * @return Visualization mode: 0=isVisible check, 1=skip isVisible, 2=isVisible+sphere-intersect midpoint
     */
    int getRepairVisMode() const { return topo_repair_vis_mode_; }

    /**
     * Get the repair visualization sphere radius.
     *
     * @return Sphere radius for mode-2 repair visualization [m]
     */
    double getRepairVisSphereRadius() const { return topo_repair_vis_sphere_radius_; }

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
     * Freeze incremental scene graph topology updates.
     *
     * Loading a full map is still allowed while frozen; this only blocks
     * updateSceneGraph from expanding topology online.
     */
    void freezeUpdate()   { scene_graph_update_frozen_ = true; }
    /**
     * Unfreeze incremental scene graph topology updates.
     */
    void unfreezeUpdate() { scene_graph_update_frozen_ = false; }
    /**
     * Check whether scene graph topology updates are frozen.
     *
     * @return True if updates are frozen
     */
    bool isUpdateFrozen() const { return scene_graph_update_frozen_; }

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
     * Project a blocked topological point to a nearby inflate-free point.
     *
     * @param[in]  p       Candidate point to repair [m]
     * @param[in]  toward  Forward reference point used to avoid repairing backward [m]
     * @param[out] p_out   Repaired inflate-free point [m]
     * @return True if a valid repair point was found
     */
    bool projectToInflateFree(const Eigen::Vector3d &p, const Eigen::Vector3d &toward, Eigen::Vector3d &p_out);
    /**
     * Search an inflate-free midpoint in the intersection of two visibility spheres.
     *
     * @param[in]  probe          First visibility sphere center [m]
     * @param[in]  toward         Second visibility sphere center [m]
     * @param[in]  sphere_radius  Radius of both visibility spheres [m]
     * @param[out] mid_out        Repaired midpoint [m]
     * @return True if a valid midpoint was found
     */
    bool findIntersectionMidpoint(const Eigen::Vector3d &probe, const Eigen::Vector3d &toward,
                                  double sphere_radius, Eigen::Vector3d &mid_out);
    /**
     * Mark the polyhedron matching a path center as navigation-blocked.
     *
     * @param[in] center  Center of the polyhedron to mark [m]
     * @param[in] force   If true, bypass debounce and mark immediately
     */
    void markPolyhedronBlocked(const Eigen::Vector3d &center, bool force = false);
    /**
     * Revalidate blocked polyhedra after their TTL and clear recovered nodes.
     */
    void revalidateBlocked();
    /**
     * Clear all runtime navigation-block marks.
     */
    void clearAllBlocked();
    /**
     * Insert a replacement topology node and connect it to visible neighbors.
     *
     * @param[in] old_center  Center of the blocked node [m]
     * @param[in] new_center  Center of the replacement node [m]
     */
    void insertReplacementNode(const Eigen::Vector3d &old_center, const Eigen::Vector3d &new_center);
    /**
     * Check whether a point is inside the local map and occupied in the inflated map.
     *
     * Out-of-map points are not treated as blocked by this predicate.
     *
     * @param[in] p  Query point [m]
     * @return True if the point is locally in-map and inflate-occupied
     */
    bool isInflateBlocked(const Eigen::Vector3d &p);

    // LLM prompt/answer polling state used by VLA swarm routing.
    std::map<unsigned int, std::string> llm_ans_str_poll_;
    std::map<unsigned int, scene_graph::PromptMsg> llm_prompts_;

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

    /**
     * Clear all stored data for a given prompt ID.
     *
     * @param[in] prompt_id  Prompt identifier to clear [--]
     */
    void clearPromptData(unsigned int prompt_id);

    template<typename T>
    bool waitForFutureWithSpinOnce(std::future<T>& future, const ros::Duration& timeout);

    // prompt generation //
    /**
     * Generate a prompt for predicting all room types.
     *
     * @param[out] prompt_str  Generated prompt string
     * @return True if generation succeeded
     */
    bool allRoomPredictionPromptGen(std::string &prompt_str);

    /**
     * Generate a prompt for predicting a single room type.
     *
     * @param[in]  room_id     Room ID [--]
     * @param[out] prompt_json JSON structure to populate
     * @return True if generation succeeded
     */
    bool singleRoomPredictionPromptGen(const int room_id, nlohmann::json &prompt_json);

    /**
     * Generate a prompt for predicting newly detected areas.
     *
     * @param[out] prompt_str  Generated prompt string
     * @return True if generation succeeded
     */
    bool newAreaPredictionPromptGen(std::string &prompt_str);

    /**
     * Generate a prompt for choosing the next area to explore.
     *
     * @param[out] prompt_str  Generated prompt string
     * @return True if generation succeeded
     */
    bool chooseAreaToGoPromptGen(std::string &prompt_str);

    /**
     * Generate a prompt for choosing a terminate object ID.
     *
     * @param[out] prompt_str  Generated prompt string
     * @return True if generation succeeded
     */
    bool chooseTerminateObjIdPromptGen(std::string &prompt_str);

    /**
     * Generate a DF (digital-fingerprint) demo prompt.
     *
     * @param[out] prompt_str  Generated prompt string
     * @return True if generation succeeded
     */
    bool DFDemoPromptGen(std::string &prompt_str);

    /**
     * Publish the scene graph JSON to the CoPaw topic.
     *
     * @param[in] scene_graph_json_str  Scene graph JSON string
     */
    void sendSceneGraphJson(std::string &scene_graph_json_str);

    /**
     * Generate a VLA search prompt for swarm task routing.
     *
     * @param[in]  prompt_type           Prompt type code [--]
     * @param[in]  command               Overall task command
     * @param[in]  task_session_id       Task session identifier [--]
     * @param[in]  observation_batch_id  Observation batch identifier [--]
     * @param[in]  semantic_context       Semantic context JSON
     * @param[out] prompt_str            Generated prompt string
     * @return True if generation succeeded
     */
    bool vlaSearchPromptGen(unsigned char prompt_type, const std::string &command,
                           uint32_t task_session_id, uint32_t observation_batch_id,
                           const nlohmann::json &semantic_context,
                           std::string &prompt_str) const;

    // result handle //
    /**
     * Handle the result of a room prediction LLM query.
     *
     * @param[in] prompt_id  Prompt identifier for the result [--]
     */
    void handleRoomPredictionResult(unsigned int prompt_id);

    /**
     * Handle the result of an exploration choice LLM query.
     *
     * @param[in]  prompt_id  Prompt identifier for the result [--]
     * @return Selected area ID on success, -1 on failure
     */
    int handelExplorationResult(unsigned int prompt_id);

    /**
     * Handle the result of a terminate object ID LLM query.
     *
     * @param[in]  prompt_id  Prompt identifier for the result [--]
     * @return Selected object ID on success, -1 on failure
     */
    int handelTerminateObjIdResult(unsigned int prompt_id);

    /**
     * Handle the result of a DF demo LLM query.
     *
     * @param[in]  prompt_id  Prompt identifier for the result [--]
     * @return Selected DF demo object ID on success, -1 on failure
     */
    int handelDFDemoResult(unsigned int prompt_id);

    /**
     * Parse a VLA search prompt result from the LLM answer.
     *
     * @param[in] prompt_id             Prompt identifier for the result [--]
     * @param[in] expected_prompt_type  Expected prompt type code [--]
     * @return Parsed result structure
     */
    VLASearchPromptResult parseVlaSearchPromptResult(unsigned int prompt_id,
                                                   unsigned char expected_prompt_type);

    // data operations //
    /**
     * Get the current prompt ID and atomically increment it.
     *
     * @return Current prompt ID before increment [--]
     */
    unsigned int getCurPromptIdAndPlusOne(){std::lock_guard<std::mutex> lock(mutex_); return cur_prompt_id_++; }

    /**
     * Get the current prompt ID without incrementing.
     *
     * @return Current prompt ID [--]
     */
    unsigned int getCurPromptId(){return cur_prompt_id_;}

    /**
     * Get the area ID from a polyhedron.
     *
     * @param[in] poly  Polyhedron pointer
     * @return Area ID of the polyhedron [--]
     */
    int getAreaFromPoly(const PolyHedronPtr& poly){return poly->area_id_;}

    /**
     * Check whether any areas need re-prediction.
     *
     * @return True if there are areas pending prediction
     */
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
    bool loadMap(const std::string& save_name, const std::string& data_path);

    /**
     * Refresh visualization after loading a saved map.
     */
    void refreshLoadedMapVisualization();

    /**
     * Publish the full scene graph visualization marker array.
     */
    void visualizeSceneGraph();

private:
    friend class SceneGraphMapIO;
    ros::NodeHandle        nh_;
    ros::Publisher         scene_graph_pub_;
    std::mutex             mutex_;

    // topo-block unreachable detection //
    global_belief::MapInterface::Ptr map_interface_;       // occupancy/inflate query interface
    std::vector<PolyHedronPtr>     last_poly_path_;      // polyhedron sequence from last getPathToObjectWithId (for center-based marking)
    std::vector<PolyHedronPtr>     blocked_list_;        // currently blocked polyhedra (for TTL revalidation/clear)
    bool   topo_block_enable_               = true;
    double topo_repair_radius_              = 0.5;
    int    topo_repair_vis_mode_            = 0;    // 0=isVisible intercept, 1=skip isVisible, 2=isVisible+sphere-intersect midpoint
    double topo_repair_vis_sphere_radius_   = 2.0;  // sphere radius for mode 2
    int    topo_block_hits_thresh_          = 2;
    double topo_block_ttl_                = 8.0;
    bool   topo_block_revalidate_on_fail_ = true;
    int    topo_block_max_iter_            = 4;
    bool   topo_repair_insert_node_       = false;   // insert repair node into topo graph: true=discard old+generate new+connect; false=mark+TTL recovery

    // LLM interface //
    std::string            this_package_path_;
    ros::Publisher         prompt_pub_;
    ros::Subscriber        llm_ans_sub_;
    unsigned int           cur_prompt_id_ = 0;
    bool                   need_area_prediction_ = false;
    bool                   scene_graph_update_frozen_ = false;

    std::map<unsigned int, std::promise<std::string>> llm_ans_promises_;
    void llmAnsCallback(const scene_graph::PromptMsg::ConstPtr& msg);
};

/**
 * Wait for a std::future while calling ros::spinOnce() to process callbacks.
 *
 * @tparam T Return type of the future
 * @param[in] future  Future object to wait on
 * @param[in] timeout  Maximum wait duration [s]
 * @return True if the future became ready within the timeout
 */
template<typename T>
bool SceneGraph::waitForFutureWithSpinOnce(std::future<T>& future, const ros::Duration& timeout)
{
    ros::Time start_time = ros::Time::now();
    while (ros::ok())
    {
        if (ros::Time::now() - start_time > timeout)
        {
            return false; // timeout failure
        }
        // non-blocking check of future readiness (0-second wait)
        auto status = future.wait_for(std::chrono::seconds(0));
        if (status == std::future_status::ready)
        {
            return true; // success
        }
        ros::spinOnce();
        ros::WallDuration(0.01).sleep(); // sleep 10 ms
    }

    return false;
}

#endif //SCENE_GRAPH_H
