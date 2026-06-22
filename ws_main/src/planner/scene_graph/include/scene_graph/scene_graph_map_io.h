//
// Created by Codex on 26-03-29.
//

#ifndef SCENE_GRAPH_MAP_IO_H
#define SCENE_GRAPH_MAP_IO_H

#include <string>

class SceneGraph;

/**
 * Scene graph map save/load utility.
 *
 * Serializes the entire scene graph (skeleton polyhedra, objects,
 * areas, edges) to JSON for persistence between missions.
 */
class SceneGraphMapIO {
public:
    explicit SceneGraphMapIO(SceneGraph& scene_graph);

    /**
     * Save the scene graph to disk.
     *
     * @param[in] save_name  File name (empty = auto-generated timestamp)
     * @return True if save succeeded
     */
    bool save(const std::string& save_name = "");
    /**
     * Load a scene graph from disk.
     *
     * @param[in] save_name  File name
     * @return True if load succeeded
     */
    bool load(const std::string& save_name);

private:
    SceneGraph& scene_graph_;
};

#endif // SCENE_GRAPH_MAP_IO_H
