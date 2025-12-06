#ifndef _ZED_RECORDER_H_
#define _ZED_RECORDER_H_

#include <string>
#include <optional>
#include <limits>
#include <atomic>
#include <sstream>

#include <sl/Camera.hpp>

#include "utils.h"

namespace Orbis {

struct Record_Parameters{
    sl::InitParameters init_parameters;
    sl::PositionalTrackingParameters pose_tracking_parameters;
    sl::RecordingParameters recording_parameters;
    unsigned long long max_frames;

    Record_Parameters()
    : max_frames(std::numeric_limits<unsigned long long>::max())
    {
        init_parameters.camera_resolution           = sl::RESOLUTION::AUTO;
        init_parameters.depth_mode                  = sl::DEPTH_MODE::NEURAL;
        init_parameters.coordinate_system           = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
        init_parameters.coordinate_units            = sl::UNIT::METER;
        init_parameters.sensors_required            = true;

        pose_tracking_parameters.mode               = sl::POSITIONAL_TRACKING_MODE::GEN_3;
        pose_tracking_parameters.enable_imu_fusion  = true;

        recording_parameters.compression_mode     = sl::SVO_COMPRESSION_MODE::H264;
        recording_parameters.video_filename.set("zed_recording.svo");
    }

    Record_Parameters(const Record_Parameters& other)
    : init_parameters(other.init_parameters)
    , pose_tracking_parameters(other.pose_tracking_parameters)
    {}

    Record_Parameters& operator=(const Record_Parameters& other) {
        if (this != &other) {
            init_parameters = other.init_parameters;
            pose_tracking_parameters = other.pose_tracking_parameters;
        }
        return *this;
    }
};

enum Record_Status {
    STOPPED,
    RECORDING,
    PAUSED,
    IDLE
};

std::string to_string(const Record_Status& status);
std::string to_string(const sl::ERROR_CODE& error_code);

class ZED_Recorder {
    Record_Parameters parameters_;
    std::atomic<Record_Status> recorder_status_;
    sl::Camera zed_;
    sl::ERROR_CODE zed_status_;

  public:
    ZED_Recorder(const std::optional<Record_Parameters>& params);
    ~ZED_Recorder() = default;

    void record();
#if 0
    void pause();

    void resume();

    void stop();
#endif
}; /* class ZED_Recorder */

} /* namespace Orbis */

#endif /* _ZED_RECORDER_H_ */