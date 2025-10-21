#include "orbis_slam/keyframe_selector.h"

namespace Orbis {

KeyFrameSelector::KeyFrameSelector()
: last_keyframe_(nullptr)
, translation_threshold_(0.1)           // 10 cm
, rotation_threshold_(M_PI / 12.0)      // radians, i.e. 15 degrees
, sample_time_threshold_(3.0)          // seconds
, pick_every_n_frames_(10)
{} /* ctor */

void
KeyFrameSelector::processFrame(const Frame::Ptr& curr_frame) {
    if (pickedAsKeyFrame(curr_frame)) {
        curr_frame->setAsKeyFrame();
        keyframe_ids_.insert(curr_frame->id);
        last_keyframe_ = curr_frame;
    }
}

bool
KeyFrameSelector::pickedAsKeyFrame(const Frame::Ptr& curr_frame) const {
    if (last_keyframe_ == nullptr || keyframe_ids_.empty()) {
        return true;
    }

    // sample-interval criterion
    bool pick_by_interval = (pick_every_n_frames_ > 0) && (curr_frame->id % pick_every_n_frames_ == 0);

    // time-based criterion
    bool pick_by_time = (curr_frame->timestamp - last_keyframe_->timestamp) > sample_time_threshold_; // seconds

    // pose-based criterion
    Sophus::SE3d curr_pose = curr_frame->pose;
    double translation_dist = calculateTranslationDistance(curr_pose, last_keyframe_->pose);
    double rotation_dist = calculateRotationDistance(curr_pose, last_keyframe_->pose);
    bool pick_by_pose = (translation_dist > translation_threshold_) || (rotation_dist > rotation_threshold_);

    return pick_by_pose || pick_by_time || pick_by_interval;
}

bool
KeyFrameSelector::isKeyFrame(const Frame::Ptr& frame) const {
    return isKeyFrame(frame->id);
}

bool
KeyFrameSelector::isKeyFrame(const uint64_t& frame_id) const {
    return keyframe_ids_.find(frame_id) != keyframe_ids_.end();
}



} /* namespace Orbis */