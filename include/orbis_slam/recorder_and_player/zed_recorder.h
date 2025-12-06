#ifndef _ZED_RECORDER_H_
#define _ZED_RECORDER_H_

#include <string>
#include <optional>
#include <limits>
#include <atomic>
#include <sstream>
#include <fstream>
#include <memory>

#include <opencv2/opencv.hpp>

#include <sl/Camera.hpp>

#include "utils.h"
#include "zed_recording.pb.h"

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
cv::Mat toCvMat(sl::Mat& zed_mat);

class ZED_Recorder {
    Record_Parameters parameters_;
    std::atomic<Record_Status> recorder_status_;
    sl::Camera zed_;
    sl::ERROR_CODE zed_status_;

    // Protobuf recording data
    std::unique_ptr<orbis_slam::Recording> recording_;

    // Frame counter for recording
    uint64_t recorded_frame_count_;
    uint64_t start_timestamp_ns_;
    uint64_t last_timestamp_ns_;

  private:
    // Helper methods for protobuf conversion
    void compressImage(const cv::Mat& image, orbis_slam::CompressedImage* compressed_image,
                      orbis_slam::CompressedImage::Format format = orbis_slam::CompressedImage::JPEG);
    void addFrame(uint64_t frame_index, const sl::Timestamp& timestamp,
                  const cv::Mat& left_image, const cv::Mat& right_image,
                  const cv::Mat& depth_map, const sl::Pose& pose,
                  const sl::Matrix3f& rotation_matrix);
    void saveRecording(const std::string& filepath, uint64_t end_timestamp_ns);

  public:
    // Constructor for standalone recording (opens its own camera)
    ZED_Recorder(const std::optional<Record_Parameters>& params);

    // Constructor for incremental recording (uses existing camera)
    ZED_Recorder();

    ~ZED_Recorder() = default;

    // Original blocking recording method
    void record();

    // New API for incremental recording
    /**
     * @brief Start a new recording session
     * @param camera Reference to an already-opened ZED camera
     */
    void startRecording(sl::Camera& camera);

    /**
     * @brief Add a frame to the current recording
     * @param camera Reference to the ZED camera
     * @param timestamp Frame timestamp
     */
    void addFrameToRecording(sl::Camera& camera, const sl::Timestamp& timestamp);

    /**
     * @brief Finish recording and save to file
     * @param filepath Path where to save the recording
     */
    void finishRecording(const std::string& filepath);

    /**
     * @brief Discard the current recording without saving
     */
    void discardRecording();

    /**
     * @brief Get the number of frames recorded
     */
    uint64_t getFrameCount() const { return recorded_frame_count_; }

    /**
     * @brief Check if currently recording
     */
    bool isRecording() const { return recorder_status_.load() == Record_Status::RECORDING; }

#if 0
    void pause();

    void resume();

    void stop();
#endif
}; /* class ZED_Recorder */

} /* namespace Orbis */

#endif /* _ZED_RECORDER_H_ */