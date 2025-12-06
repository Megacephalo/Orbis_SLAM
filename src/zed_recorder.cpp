#include "orbis_slam/recorder_and_player/zed_recorder.h"

namespace Orbis {

std::string to_string(const Record_Status& status) {
    switch (status) {
        case Record_Status::STOPPED:
            return "STOPPED";
        case Record_Status::RECORDING:
            return "RECORDING";
        case Record_Status::PAUSED:
            return "PAUSED";
        case Record_Status::IDLE:
            return "IDLE";
        default:
            return "UNKNOWN";
    }
}

std::string to_string(const sl::ERROR_CODE& error_code) {
    std::stringstream sstr;
    sstr << error_code;
    return sstr.str();
}

ZED_Recorder::ZED_Recorder(const std::optional<Record_Parameters>& params) {
    recorder_status_.store(Record_Status::IDLE);

    if (params.has_value()) {
        parameters_ = params.value();
    }

    zed_status_ = zed_.open(parameters_.init_parameters);
    if (zed_status_ != sl::ERROR_CODE::SUCCESS) {
        recorder_status_.store(Record_Status::STOPPED);
        zed_.close();
        throw std::runtime_error("ZED_Recorder::record: Unable to open ZED camera: " + to_string(zed_status_));
    }

    zed_status_ = zed_.enablePositionalTracking(parameters_.pose_tracking_parameters);
    if (zed_status_ != sl::ERROR_CODE::SUCCESS) {
        recorder_status_.store(Record_Status::STOPPED);
        zed_.close();
        throw std::runtime_error("ZED_Recorder::record: Unable to enable positional tracking: " + to_string(zed_status_));
    }

    zed_status_ = zed_.enableRecording(parameters_.recording_parameters);
    if (zed_status_ != sl::ERROR_CODE::SUCCESS) {
        recorder_status_.store(Record_Status::STOPPED);
        zed_.close();
        throw std::runtime_error("ZED_Recorder::record: Unable to enable recording: " + to_string(zed_status_));
    }   
}

void
ZED_Recorder::record() {
    if (parameters_.recording_parameters.video_filename.empty()) {
        std::cerr << "ZED_Recorder::record: save_to_file parameter is empty." << std::endl;
        return;
    }

    if ( zed_status_ != sl::ERROR_CODE::SUCCESS ) {
        std::cerr << "ZED_Recorder::record: ZED camera is not opened properly." << std::endl;
        return;
    }
    
    if (recorder_status_.load() == Record_Status::IDLE || recorder_status_.load() == Record_Status::PAUSED) {
        recorder_status_.store(Record_Status::RECORDING);
    }

    // Main recording loop
    // Start recording SVO, stop with Ctrl-C command
    unsigned long long frame_count = 0;
    SetCtrlHandler();
    while ( recorder_status_.load() == Record_Status::RECORDING && 
            frame_count < parameters_.max_frames) {
        
        if (exit_app) {
            recorder_status_.store(Record_Status::STOPPED);
            break;
        }
#if 0
        sl::ERROR_CODE grab_status = zed_.grab();

        if (grab_status != sl::ERROR_CODE::SUCCESS) {
            std::cerr << "ZED_Recorder::record: Failed to grab frame: " << std::to_string(int(grab_status)) << std::endl;
            continue;   
        }
        
        // get the camera intrinsics

        // get timestamp
        sl::Timestamp timestamp = zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE);

        // pose information
        sl::Pose zed_pose;
        zed_.getPosition(zed_pose, sl::REFERENCE_FRAME::WORLD);
        
        // rotation matrix
        sl::Matrix3f rotation_matrix = zed_pose.getRotationMatrix();

        // get the stereo image

        // get the (registered) depth map
#else
        // DEBUG
        std::cout << "Recording frame " << frame_count << "\r";
#endif
        frame_count++;
    } /* while */

    zed_.disableRecording();
    zed_.disablePositionalTracking();
    zed_.close();
}
#if 0
void
ZED_Recorder::pause() {

}

void
ZED_Recorder::resume() {

}

void
ZED_Recorder::stop() {

}

#endif

} /* namespace */