#ifndef _COVISIBILITY_GRAPH_H_
#define _COVISIBILITY_GRAPH_H_

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <map>
#include <mutex>
#include <shared_mutex>
#include <algorithm>
#include <memory>

#include <opencv2/core.hpp>

namespace Orbis {

/**
 * @brief CovisibilityGraph manages the covisibility relationships between keyframes
 *
 * This class maintains a graph where nodes are keyframes and edges represent
 * covisibility (number of shared map points). It uses only STL containers for
 * cross-platform compatibility and computational efficiency.
 *
 * Thread-safe implementation using reader-writer locks for concurrent access.
 */
class CovisibilityGraph {
public:
    struct Edge {
        uint64_t from_id;
        uint64_t to_id;
        int weight;  // Number of shared map points

        Edge(uint64_t from, uint64_t to, int w)
            : from_id(from), to_id(to), weight(w) {}
    };

private:
    // Adjacency list: keyframe_id -> map of (connected_keyframe_id -> weight)
    std::unordered_map<uint64_t, std::map<uint64_t, int>> adjacency_list_;

    // Set of all keyframe IDs in the graph
    std::unordered_set<uint64_t> keyframe_ids_;

    // Mutex for thread-safe operations
    mutable std::shared_mutex mutex_;

    // Minimum covisibility weight threshold
    int min_covisibility_weight_;

public:
    /**
     * @brief Construct a new Covisibility Graph
     *
     * @param min_weight Minimum number of shared map points to create an edge (default: 15)
     */
    explicit CovisibilityGraph(int min_weight = 15);

    ~CovisibilityGraph() = default;

    // Non-copyable but movable
    CovisibilityGraph(const CovisibilityGraph&) = delete;
    CovisibilityGraph& operator=(const CovisibilityGraph&) = delete;
    CovisibilityGraph(CovisibilityGraph&&) = default;
    CovisibilityGraph& operator=(CovisibilityGraph&&) = default;

    /**
     * @brief Add a keyframe to the graph
     *
     * @param keyframe_id ID of the keyframe to add
     */
    void addKeyframe(uint64_t keyframe_id);

    /**
     * @brief Remove a keyframe from the graph
     *
     * @param keyframe_id ID of the keyframe to remove
     */
    void removeKeyframe(uint64_t keyframe_id);

    /**
     * @brief Add or update a covisibility edge between two keyframes
     *
     * @param keyframe_id1 First keyframe ID
     * @param keyframe_id2 Second keyframe ID
     * @param weight Number of shared map points
     */
    void addEdge(uint64_t keyframe_id1, uint64_t keyframe_id2, int weight);

    /**
     * @brief Remove an edge between two keyframes
     *
     * @param keyframe_id1 First keyframe ID
     * @param keyframe_id2 Second keyframe ID
     */
    void removeEdge(uint64_t keyframe_id1, uint64_t keyframe_id2);

    /**
     * @brief Get covisibility weight between two keyframes
     *
     * @param keyframe_id1 First keyframe ID
     * @param keyframe_id2 Second keyframe ID
     * @return int Weight (0 if no edge exists)
     */
    int getWeight(uint64_t keyframe_id1, uint64_t keyframe_id2) const;

    /**
     * @brief Get all connected keyframes for a given keyframe
     *
     * @param keyframe_id The keyframe ID
     * @return std::vector<uint64_t> List of connected keyframe IDs
     */
    std::vector<uint64_t> getConnectedKeyframes(uint64_t keyframe_id) const;

    /**
     * @brief Get connected keyframes sorted by covisibility weight (descending)
     *
     * @param keyframe_id The keyframe ID
     * @param min_weight Minimum weight threshold (default: use class minimum)
     * @return std::vector<std::pair<uint64_t, int>> Pairs of (keyframe_id, weight)
     */
    std::vector<std::pair<uint64_t, int>> getCovisibleKeyframes(
        uint64_t keyframe_id,
        int min_weight = -1
    ) const;

    /**
     * @brief Get the top N most covisible keyframes
     *
     * @param keyframe_id The keyframe ID
     * @param n Number of top keyframes to return
     * @return std::vector<std::pair<uint64_t, int>> Pairs of (keyframe_id, weight)
     */
    std::vector<std::pair<uint64_t, int>> getTopNCovisibleKeyframes(
        uint64_t keyframe_id,
        size_t n
    ) const;

    /**
     * @brief Check if a keyframe exists in the graph
     *
     * @param keyframe_id The keyframe ID
     * @return true if keyframe exists
     */
    bool hasKeyframe(uint64_t keyframe_id) const;

    /**
     * @brief Get total number of keyframes in the graph
     *
     * @return size_t Number of keyframes
     */
    size_t size() const;

    /**
     * @brief Get all keyframe IDs in the graph
     *
     * @return std::vector<uint64_t> List of all keyframe IDs
     */
    std::vector<uint64_t> getAllKeyframes() const;

    /**
     * @brief Get all edges in the graph
     *
     * @return std::vector<Edge> List of all edges
     */
    std::vector<Edge> getAllEdges() const;

    /**
     * @brief Clear the entire graph
     */
    void clear();

    /**
     * @brief Update covisibility based on shared map points
     *
     * This is a helper function that computes covisibility between two keyframes
     * based on their map points and updates the graph accordingly.
     *
     * @param keyframe_id1 First keyframe ID
     * @param keyframe_id2 Second keyframe ID
     * @param map_points1 Map points from first keyframe
     * @param map_points2 Map points from second keyframe
     * @param valid1 Validity flags for first keyframe's map points
     * @param valid2 Validity flags for second keyframe's map points
     */
    void updateCovisibility(
        uint64_t keyframe_id1,
        uint64_t keyframe_id2,
        const std::vector<cv::Point3f>& map_points1,
        const std::vector<cv::Point3f>& map_points2,
        const std::vector<bool>& valid1,
        const std::vector<bool>& valid2
    );

    typedef std::shared_ptr<CovisibilityGraph> Ptr;

    static CovisibilityGraph::Ptr create(int min_weight = 15) {
        return std::make_shared<CovisibilityGraph>(min_weight);
    }
};

} /* namespace Orbis */

#endif /* _COVISIBILITY_GRAPH_H_ */
