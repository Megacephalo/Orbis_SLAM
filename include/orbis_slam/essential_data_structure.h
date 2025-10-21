#ifndef _ESSENTIAL_DATA_STRUCTURE_H_
#define _ESSENTIAL_DATA_STRUCTURE_H_

#include <cstdint>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <memory>
#include <optional>
#include <functional>
#include <algorithm>
#include <execution>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace Orbis{

struct Frame {
    uint64_t id;  // guaranteed to be 64 bits, portable across platforms
    Sophus::SE3d pose;
    double timestamp;
    std::atomic<bool> is_keyframe;

    explicit Frame(const uint64_t& id)
    : id(id), pose(Sophus::SE3d()), timestamp(0.0), is_keyframe(false) {}
    
    Frame(const uint64_t& id_ = 0, const Sophus::SE3d& pose_ = Sophus::SE3d(), const bool isKeyFrame = false, const double& ts = 0.0)
    : id(id_), pose(pose_), timestamp(ts), is_keyframe(isKeyFrame) {}

    ~Frame() = default;

    Frame(const Frame& other)
    : id(other.id), pose(other.pose), timestamp(other.timestamp), is_keyframe(other.is_keyframe.load()) {}

    Frame& operator = (const Frame& other) {
        if (this != &other) {
            id = other.id;
            pose = other.pose;
            is_keyframe = other.is_keyframe.load();
            timestamp = other.timestamp;
        }
        return *this;
    }

    Frame(Frame&& other) noexcept
    : id(other.id), pose(other.pose), timestamp(other.timestamp), is_keyframe(other.is_keyframe.load()) {
        other.id = 0;
        other.pose = Sophus::SE3d();
        other.timestamp = 0.0;
        other.is_keyframe = false;
    }

    Frame& operator = (Frame&& other) noexcept {
        if (this != &other) {
            id = other.id;
            pose = other.pose;
            is_keyframe = other.is_keyframe.load();
            timestamp = other.timestamp;
            other.id = 0;
            other.pose = Sophus::SE3d();
            other.is_keyframe = false;
            other.timestamp = 0.0;
        }
        return *this;
    }

    bool operator == (const Frame& other) const {
        constexpr double eps = 1e-9;
        return (id == other.id) || 
            ( (pose.translation() - other.pose.translation()).norm() <= eps      ) || 
            ( (pose.so3().log() - other.pose.so3().log()).norm() <= eps          );
    }
    
    bool operator != (const Frame& other) const {
        return ! (*this == other);
    }

    bool isKeyFrame() const { return is_keyframe.load(); }

    void setAsKeyFrame(const bool& isKeyFrame = true) { is_keyframe.store(isKeyFrame); }

    struct FrameHash {
        std::size_t operator() (const Frame& f) const {
            return std::hash<uint64_t>()(f.id) ^ (std::hash<double>()(f.pose.matrix().sum()) << 1);
        }
    };

    typedef std::shared_ptr<Frame> Ptr;

    static Frame::Ptr create(const unsigned long long& id = 0, const Sophus::SE3d& pose = Sophus::SE3d(), const bool isKeyFrame = false, const double& timestamp = 0.0) {
        return std::make_shared<Frame>(id, pose, isKeyFrame, timestamp);
    }
};



class Trajectory {
private:
    mutable std::shared_mutex mutex_;
    std::vector<Frame::Ptr> frames_;

public:
    Trajectory() = default;
    ~Trajectory() = default;
    
    // Non-copyable but movable
    Trajectory(const Trajectory&) = delete;
    Trajectory& operator=(const Trajectory&) = delete;
    Trajectory(Trajectory&&) = default;
    Trajectory& operator=(Trajectory&&) = default;

    // Add a frame to the trajectory
    void pushBack(Frame::Ptr frame) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        frames_.push_back(frame);
    }

    // Add a frame by constructing it in-place
    template<typename... Args>
    void emplaceBack(Args&&... args) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        frames_.emplace_back(std::make_shared<Frame>(std::forward<Args>(args)...));
    }

    // Get frame at specific index (returns copy for thread safety)
    std::optional<Frame> at(size_t index) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        if (index >= frames_.size()) {
            return std::nullopt;
        }
        return *frames_[index];  // Return copy
    }

    // Get frame pointer at specific index (shared ownership)
    Frame::Ptr getFramePtr(size_t index) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        if (index >= frames_.size()) {
            return nullptr;
        }
        return frames_[index];
    }

    // Get the latest frame
    std::optional<Frame> getLatest() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        if (frames_.empty()) {
            return std::nullopt;
        }
        return *frames_.back();
    }

    // Get latest frame pointer
    Frame::Ptr getLatestPtr() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        if (frames_.empty()) {
            return nullptr;
        }
        return frames_.back();
    }

    // Find frame by ID
    std::optional<Frame> findById(uint64_t id) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        auto it = std::find_if(frames_.begin(), frames_.end(),
            [id](const Frame::Ptr& frame) { return frame->id == id; });
        
        if (it != frames_.end()) {
            return **it;
        }
        return std::nullopt;
    }

    // Find frame pointer by ID
    Frame::Ptr findPtrById(uint64_t id) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        auto it = std::find_if(frames_.begin(), frames_.end(),
            [id](const Frame::Ptr& frame) { return frame->id == id; });
        
        return (it != frames_.end()) ? *it : nullptr;
    }

    // Get all keyframes
    std::vector<Frame> getKeyframes() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        std::vector<Frame> keyframes;
        for (const auto& frame : frames_) {
            if (frame->is_keyframe) {
                keyframes.push_back(*frame);
            }
        }
        return keyframes;
    }

    // Get all keyframe pointers
    std::vector<Frame::Ptr> getKeyframePtrs() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        std::vector<Frame::Ptr> keyframes;
        for (const auto& frame : frames_) {
            if (frame->is_keyframe) {
                keyframes.push_back(frame);
            }
        }
        return keyframes;
    }

    // Get trajectory size
    size_t size() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return frames_.size();
    }

    // Check if trajectory is empty
    bool empty() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return frames_.empty();
    }

    // Clear all frames
    void clear() {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        frames_.clear();
    }

    // Reserve capacity for better performance
    void reserve(size_t capacity) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        frames_.reserve(capacity);
    }

    // Update frame at specific index
    bool updateFrame(size_t index, const Frame& frame) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        if (index >= frames_.size()) {
            return false;
        }
        *frames_[index] = frame;
        return true;
    }

    // Apply function to all frames (read-only)
    template<typename Func>
    void forEach(Func&& func) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        for (const auto& frame : frames_) {
            func(*frame);
        }
    }

    // Apply function to all frames (with modification capability)
    template<typename Func>
    void forEachMutable(Func&& func) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        for (Frame::Ptr& frame : frames_) {
            func(*frame);
        }
    }

    // Get frames in a time range
    std::vector<Frame> getFramesInTimeRange(double start_time, double end_time) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        std::vector<Frame> result;
        for (const auto& frame : frames_) {
            if (frame->timestamp >= start_time && frame->timestamp <= end_time) {
                result.push_back(*frame);
            }
        }
        return result;
    }

    // Get the last N frames
    std::vector<Frame> getLastNFrames(size_t n) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        std::vector<Frame> result;
        size_t start_idx = (n >= frames_.size()) ? 0 : frames_.size() - n;
        
        for (size_t i = start_idx; i < frames_.size(); ++i) {
            result.push_back(*frames_[i]);
        }
        return result;
    }

    // Thread-safe iterator access with callback
    template<typename Func>
    void safeIterate(Func&& func) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        for (auto it = frames_.begin(); it != frames_.end(); ++it) {
            if (!func(*it, std::distance(frames_.begin(), it))) {
                break;  // Allow early termination
            }
        }
    }

    // Iterator support for range-based for loops
    class const_iterator {
    private:
        mutable std::shared_lock<std::shared_mutex> lock_;
        std::vector<Frame::Ptr>::const_iterator iter_;
        std::vector<Frame::Ptr>::const_iterator end_iter_;
        
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = Frame::Ptr;
        using difference_type = std::ptrdiff_t;
        using pointer = const Frame::Ptr*;
        using reference = const Frame::Ptr&;

        const_iterator(std::shared_lock<std::shared_mutex>&& lock, 
                      std::vector<Frame::Ptr>::const_iterator iter,
                      std::vector<Frame::Ptr>::const_iterator end_iter)
            : lock_(std::move(lock)), iter_(iter), end_iter_(end_iter) {}

        // Move-only iterator
        const_iterator(const const_iterator&) = delete;
        const_iterator& operator=(const const_iterator&) = delete;
        const_iterator(const_iterator&&) = default;
        const_iterator& operator=(const_iterator&&) = default;

        reference operator*() const { return *iter_; }
        pointer operator->() const { return &(*iter_); }
        
        const_iterator& operator++() { ++iter_; return *this; }
        
        // Remove problematic post-increment operator
        // const_iterator operator++(int) is not supported for move-only iterators
        
        bool operator==(const const_iterator& other) const { return iter_ == other.iter_; }
        bool operator!=(const const_iterator& other) const { return iter_ != other.iter_; }
        
        // Helper method to check if we've reached the end
        bool is_end() const { return iter_ == end_iter_; }
    };

    // Begin iterator
    const_iterator begin() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return const_iterator(std::move(lock), frames_.begin(), frames_.end());
    }

    // End iterator  
    const_iterator end() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return const_iterator(std::move(lock), frames_.end(), frames_.end());
    }

    // Safer alternative: Get a snapshot for iteration
    std::vector<Frame::Ptr> getSnapshot() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return frames_;  // Returns a copy
    }

    typedef std::shared_ptr<Trajectory> Ptr;
    static Trajectory::Ptr create() {
        return std::make_shared<Trajectory>();
    }
}; /* class Trajectory */


} /* namespace Orbis */

#endif /* _ESSENTIAL_DATA_STRUCTURE_H_ */