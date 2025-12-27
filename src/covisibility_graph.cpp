#include "orbis_slam/covisibility_graph.h"
#include <opencv2/core.hpp>
#include <cmath>

namespace Orbis {

CovisibilityGraph::CovisibilityGraph(int min_weight)
    : min_covisibility_weight_(min_weight)
{
}

void CovisibilityGraph::addKeyframe(uint64_t keyframe_id) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    keyframe_ids_.insert(keyframe_id);
    // Initialize empty adjacency list entry if not exists
    if (adjacency_list_.find(keyframe_id) == adjacency_list_.end()) {
        adjacency_list_[keyframe_id] = std::map<uint64_t, int>();
    }
}

void CovisibilityGraph::removeKeyframe(uint64_t keyframe_id) {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    // Remove from keyframe set
    keyframe_ids_.erase(keyframe_id);

    // Remove all edges from this keyframe
    adjacency_list_.erase(keyframe_id);

    // Remove all edges to this keyframe from other keyframes
    for (auto& [other_id, connections] : adjacency_list_) {
        connections.erase(keyframe_id);
    }
}

void CovisibilityGraph::addEdge(uint64_t keyframe_id1, uint64_t keyframe_id2, int weight) {
    if (keyframe_id1 == keyframe_id2) {
        return;  // No self-loops
    }

    if (weight < min_covisibility_weight_) {
        return;  // Weight below threshold
    }

    std::unique_lock<std::shared_mutex> lock(mutex_);

    // Ensure both keyframes exist
    keyframe_ids_.insert(keyframe_id1);
    keyframe_ids_.insert(keyframe_id2);

    // Add bidirectional edge
    adjacency_list_[keyframe_id1][keyframe_id2] = weight;
    adjacency_list_[keyframe_id2][keyframe_id1] = weight;
}

void CovisibilityGraph::removeEdge(uint64_t keyframe_id1, uint64_t keyframe_id2) {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    auto it1 = adjacency_list_.find(keyframe_id1);
    if (it1 != adjacency_list_.end()) {
        it1->second.erase(keyframe_id2);
    }

    auto it2 = adjacency_list_.find(keyframe_id2);
    if (it2 != adjacency_list_.end()) {
        it2->second.erase(keyframe_id1);
    }
}

int CovisibilityGraph::getWeight(uint64_t keyframe_id1, uint64_t keyframe_id2) const {
    std::shared_lock<std::shared_mutex> lock(mutex_);

    auto it = adjacency_list_.find(keyframe_id1);
    if (it == adjacency_list_.end()) {
        return 0;
    }

    auto edge_it = it->second.find(keyframe_id2);
    if (edge_it == it->second.end()) {
        return 0;
    }

    return edge_it->second;
}

std::vector<uint64_t> CovisibilityGraph::getConnectedKeyframes(uint64_t keyframe_id) const {
    std::shared_lock<std::shared_mutex> lock(mutex_);

    std::vector<uint64_t> connected;

    auto it = adjacency_list_.find(keyframe_id);
    if (it == adjacency_list_.end()) {
        return connected;
    }

    connected.reserve(it->second.size());
    for (const auto& [neighbor_id, weight] : it->second) {
        connected.push_back(neighbor_id);
    }

    return connected;
}

std::vector<std::pair<uint64_t, int>> CovisibilityGraph::getCovisibleKeyframes(
    uint64_t keyframe_id,
    int min_weight
) const {
    std::shared_lock<std::shared_mutex> lock(mutex_);

    if (min_weight < 0) {
        min_weight = min_covisibility_weight_;
    }

    std::vector<std::pair<uint64_t, int>> covisible;

    auto it = adjacency_list_.find(keyframe_id);
    if (it == adjacency_list_.end()) {
        return covisible;
    }

    // Filter by weight and collect
    for (const auto& [neighbor_id, weight] : it->second) {
        if (weight >= min_weight) {
            covisible.emplace_back(neighbor_id, weight);
        }
    }

    // Sort by weight in descending order
    std::sort(covisible.begin(), covisible.end(),
        [](const auto& a, const auto& b) {
            return a.second > b.second;
        }
    );

    return covisible;
}

std::vector<std::pair<uint64_t, int>> CovisibilityGraph::getTopNCovisibleKeyframes(
    uint64_t keyframe_id,
    size_t n
) const {
    auto all_covisible = getCovisibleKeyframes(keyframe_id, 0);

    // Already sorted by weight, just take top N
    if (all_covisible.size() > n) {
        all_covisible.resize(n);
    }

    return all_covisible;
}

bool CovisibilityGraph::hasKeyframe(uint64_t keyframe_id) const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return keyframe_ids_.find(keyframe_id) != keyframe_ids_.end();
}

size_t CovisibilityGraph::size() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return keyframe_ids_.size();
}

std::vector<uint64_t> CovisibilityGraph::getAllKeyframes() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return std::vector<uint64_t>(keyframe_ids_.begin(), keyframe_ids_.end());
}

std::vector<CovisibilityGraph::Edge> CovisibilityGraph::getAllEdges() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);

    std::vector<Edge> edges;

    for (const auto& [from_id, connections] : adjacency_list_) {
        for (const auto& [to_id, weight] : connections) {
            // Only add each edge once (avoid duplicates in undirected graph)
            if (from_id < to_id) {
                edges.emplace_back(from_id, to_id, weight);
            }
        }
    }

    return edges;
}

void CovisibilityGraph::clear() {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    adjacency_list_.clear();
    keyframe_ids_.clear();
}

void CovisibilityGraph::updateCovisibility(
    uint64_t keyframe_id1,
    uint64_t keyframe_id2,
    const std::vector<cv::Point3f>& map_points1,
    const std::vector<cv::Point3f>& map_points2,
    const std::vector<bool>& valid1,
    const std::vector<bool>& valid2
) {
    if (keyframe_id1 == keyframe_id2) {
        return;  // No self-loops
    }

    // Count shared map points
    int shared_count = 0;
    constexpr float distance_threshold = 0.01f;  // 1 cm threshold for considering points as same

    for (size_t i = 0; i < map_points1.size() && i < valid1.size(); ++i) {
        if (!valid1[i]) continue;

        const auto& p1 = map_points1[i];

        for (size_t j = 0; j < map_points2.size() && j < valid2.size(); ++j) {
            if (!valid2[j]) continue;

            const auto& p2 = map_points2[j];

            // Compute Euclidean distance
            float dx = p1.x - p2.x;
            float dy = p1.y - p2.y;
            float dz = p1.z - p2.z;
            float dist = std::sqrt(dx*dx + dy*dy + dz*dz);

            if (dist < distance_threshold) {
                shared_count++;
                break;  // Found a match, move to next point in map_points1
            }
        }
    }

    // Add or update edge if shared count is above threshold
    if (shared_count >= min_covisibility_weight_) {
        addEdge(keyframe_id1, keyframe_id2, shared_count);
    } else {
        // Remove edge if it exists but no longer meets threshold
        removeEdge(keyframe_id1, keyframe_id2);
    }
}

} /* namespace Orbis */
