#ifndef _KEYFRAME_SELECTOR_H_
#define _KEYFRAME_SELECTOR_H_

#include <cstdint>
#include <memory>
#include <unordered_set>
#include <iostream>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include "orbis_slam/common_utils.h"
#include "orbis_slam/essential_data_structure.h"

namespace Orbis {

class KeyFrameSelector {
  private:
    Frame::Ptr last_keyframe_;
    double translation_threshold_;  // meters
    double rotation_threshold_;     // radians
    double sample_time_threshold_;  // seconds
    uint64_t pick_every_n_frames_;  

    std::unordered_set<uint64_t> keyframe_ids_;
    bool pickedAsKeyFrame(const Frame::Ptr& curr_frame) const;

  public:
    KeyFrameSelector();
    ~KeyFrameSelector() = default;
    void processFrame(const Frame::Ptr& curr_frame);
    bool isKeyFrame(const Frame::Ptr& frame) const;
    bool isKeyFrame(const uint64_t& frame_id) const;

    size_t getTotalKeyFrames() const { return keyframe_ids_.size(); }
    Frame::Ptr getLastKeyFrame() const { return last_keyframe_; }
}; /* class KeyFrameSelector */

} /* namespace Orbis */

#endif /* _KEYFRAME_SELECTOR_H_ */