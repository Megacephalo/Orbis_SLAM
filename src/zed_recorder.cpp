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

cv::Mat toCvMat(sl::Mat& zed_mat) {
    return cv::Mat(zed_mat.getHeight(), zed_mat.getWidth(), (zed_mat.getChannels() == 4) ? CV_8UC4 : CV_8UC3, zed_mat.getPtr<sl::uchar1>(sl::MEM::CPU));
}

// Constructor for incremental recording (uses existing camera)
ZED_Recorder::ZED_Recorder()
    : recorded_frame_count_(0)
    , start_timestamp_ns_(0)
    , last_timestamp_ns_(0)
{
    recorder_status_.store(Record_Status::IDLE);
    zed_status_ = sl::ERROR_CODE::SUCCESS;
}

// Constructor for standalone recording (opens its own camera)
ZED_Recorder::ZED_Recorder(const std::optional<Record_Parameters>& params)
    : recorded_frame_count_(0)
    , start_timestamp_ns_(0)
    , last_timestamp_ns_(0)
{
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
ZED_Recorder::compressImage(const cv::Mat& image, orbis_slam::CompressedImage* compressed_image,
                            orbis_slam::CompressedImage::Format format) {
    compressed_image->set_width(image.cols);
    compressed_image->set_height(image.rows);
    compressed_image->set_channels(image.channels());
    compressed_image->set_format(format);

    std::vector<uint8_t> buffer;

    switch (format) {
        case orbis_slam::CompressedImage::JPEG: {
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 95};
            cv::imencode(".jpg", image, buffer, params);
            break;
        }
        case orbis_slam::CompressedImage::PNG: {
            std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 3};
            cv::imencode(".png", image, buffer, params);
            break;
        }
        case orbis_slam::CompressedImage::RAW: {
            size_t size = image.total() * image.elemSize();
            buffer.resize(size);
            std::memcpy(buffer.data(), image.data, size);
            break;
        }
        default:
            throw std::runtime_error("Unsupported image compression format");
    }

    compressed_image->set_data(buffer.data(), buffer.size());
}

void
ZED_Recorder::addFrame(uint64_t frame_index, const sl::Timestamp& timestamp,
                      const cv::Mat& left_image, const cv::Mat& right_image,
                      const cv::Mat& depth_map, const sl::Pose& pose,
                      const sl::Matrix3f& rotation_matrix) {
    orbis_slam::Frame* frame = recording_->add_frames();

    // Frame metadata
    frame->set_frame_index(frame_index);
    frame->set_timestamp_ns(timestamp.getNanoseconds());

    // Compress images
    compressImage(left_image, frame->mutable_left_image(), orbis_slam::CompressedImage::JPEG);
    compressImage(right_image, frame->mutable_right_image(), orbis_slam::CompressedImage::JPEG);

    // For depth, use PNG for better lossless compression or RAW for exact values
    compressImage(depth_map, frame->mutable_depth_image(), orbis_slam::CompressedImage::PNG);

    // Camera pose
    orbis_slam::CameraPose* camera_pose = frame->mutable_pose();
    camera_pose->set_tx(pose.getTranslation().tx);
    camera_pose->set_ty(pose.getTranslation().ty);
    camera_pose->set_tz(pose.getTranslation().tz);
    camera_pose->set_qw(pose.getOrientation().ow);
    camera_pose->set_qx(pose.getOrientation().ox);
    camera_pose->set_qy(pose.getOrientation().oy);
    camera_pose->set_qz(pose.getOrientation().oz);

    // Rotation matrix (row-major 3x3)
    // Matrix3f is stored as a flat array in row-major order
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            camera_pose->add_rotation_matrix(rotation_matrix.r[i * 3 + j]);
        }
    }
}

void
ZED_Recorder::saveRecording(const std::string& filepath, uint64_t end_timestamp_ns) {
    // Set end timestamp from last frame
    recording_->set_end_timestamp_ns(end_timestamp_ns);

    // Open file for binary writing
    std::ofstream output(filepath, std::ios::binary);
    if (!output.is_open()) {
        throw std::runtime_error("Failed to open file for writing: " + filepath);
    }

    // Serialize to binary
    if (!recording_->SerializeToOstream(&output)) {
        throw std::runtime_error("Failed to serialize recording to file: " + filepath);
    }

    output.close();
    std::cout << "Recording saved to: " << filepath << std::endl;
    std::cout << "Total frames: " << recording_->frames_size() << std::endl;
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

    // Get initial timestamp from camera for recording start time
    sl::Timestamp start_timestamp = zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE);

    // Get the camera information and initialize protobuf recording
    sl::CameraInformation cam_info = zed_.getCameraInformation();
    recording_ = std::make_unique<orbis_slam::Recording>();

    // Set recording metadata using camera timestamp
    recording_->set_recording_id("zed_recording_" + std::to_string(start_timestamp.getNanoseconds()));
    recording_->set_start_timestamp_ns(start_timestamp.getNanoseconds());
    recording_->set_camera_model("ZED2i");

    // Set calibration parameters
    sl::CalibrationParameters calib_params = cam_info.camera_configuration.calibration_parameters;

    // Left camera intrinsics
    orbis_slam::CameraIntrinsics* left_intrinsics = recording_->mutable_left_intrinsics();
    left_intrinsics->set_fx(calib_params.left_cam.fx);
    left_intrinsics->set_fy(calib_params.left_cam.fy);
    left_intrinsics->set_cx(calib_params.left_cam.cx);
    left_intrinsics->set_cy(calib_params.left_cam.cy);
    left_intrinsics->set_width(cam_info.camera_configuration.resolution.width);
    left_intrinsics->set_height(cam_info.camera_configuration.resolution.height);

    // Right camera intrinsics
    orbis_slam::CameraIntrinsics* right_intrinsics = recording_->mutable_right_intrinsics();
    right_intrinsics->set_fx(calib_params.right_cam.fx);
    right_intrinsics->set_fy(calib_params.right_cam.fy);
    right_intrinsics->set_cx(calib_params.right_cam.cx);
    right_intrinsics->set_cy(calib_params.right_cam.cy);
    right_intrinsics->set_width(cam_info.camera_configuration.resolution.width);
    right_intrinsics->set_height(cam_info.camera_configuration.resolution.height);

    // Stereo calibration
    orbis_slam::StereoCalibration* stereo_calib = recording_->mutable_stereo_calibration();
    stereo_calib->set_baseline(calib_params.stereo_transform.getTranslation()[0]);

    // Rotation matrix (row-major 3x3)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            stereo_calib->add_rotation(calib_params.stereo_transform.m[i * 4 + j]);
        }
    }

    // Translation vector
    sl::float3 T = calib_params.stereo_transform.getTranslation();
    stereo_calib->add_translation(T.x);
    stereo_calib->add_translation(T.y);
    stereo_calib->add_translation(T.z);

    // Generate output filename with .orbis extension
    std::string svo_filename = parameters_.recording_parameters.video_filename.get();
    std::string orbis_filename = svo_filename;

    // Replace .svo extension with .orbis
    size_t ext_pos = orbis_filename.rfind(".svo");
    if (ext_pos != std::string::npos) {
        orbis_filename.replace(ext_pos, 4, ".orbis");
    } else {
        orbis_filename += ".orbis";
    }

    std::cout << "Recording to SVO: " << svo_filename << std::endl;
    std::cout << "Recording to Protobuf: " << orbis_filename << std::endl;

    // Main recording loop
    unsigned long long frame_count = 0;
    uint64_t last_timestamp_ns = start_timestamp.getNanoseconds();
    SetCtrlHandler();

    while ( recorder_status_.load() == Record_Status::RECORDING &&
            frame_count < parameters_.max_frames) {

        if (exit_app) {
            recorder_status_.store(Record_Status::STOPPED);
            break;
        }

        sl::ERROR_CODE grab_status = zed_.grab();

        if (grab_status != sl::ERROR_CODE::SUCCESS) {
            std::cerr << "ZED_Recorder::record: Failed to grab frame: " << std::to_string(int(grab_status)) << std::endl;
            continue;
        }

        // Get timestamp from camera
        sl::Timestamp timestamp = zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE);
        last_timestamp_ns = timestamp.getNanoseconds();

        // Get pose information
        sl::Pose zed_pose;
        zed_.getPosition(zed_pose, sl::REFERENCE_FRAME::WORLD);

        // Get rotation matrix
        sl::Matrix3f rotation_matrix = zed_pose.getRotationMatrix();

        // Get the stereo images
        sl::Mat sl_left_image, sl_right_image;
        cv::Mat left_image, right_image;
        zed_.retrieveImage(sl_left_image, sl::VIEW::LEFT);
        zed_.retrieveImage(sl_right_image, sl::VIEW::RIGHT);
        left_image = toCvMat(sl_left_image);
        right_image = toCvMat(sl_right_image);

        // Get the (registered) depth map
        sl::Mat sl_depth_map;
        cv::Mat depth_map;
        zed_.retrieveMeasure(sl_depth_map, sl::MEASURE::DEPTH);
        depth_map = toCvMat(sl_depth_map);

        // Add frame to protobuf recording
        addFrame(frame_count, timestamp, left_image, right_image, depth_map, zed_pose, rotation_matrix);

        frame_count++;

        // Print progress every 30 frames
        if (frame_count % 30 == 0) {
            std::cout << "Recorded " << frame_count << " frames..." << std::endl;
        }
    } /* while */

    std::cout << "Recording complete. Total frames: " << frame_count << std::endl;

    // Save the protobuf recording to .orbis file with camera timestamp
    try {
        saveRecording(orbis_filename, last_timestamp_ns);
    } catch (const std::exception& e) {
        std::cerr << "Failed to save protobuf recording: " << e.what() << std::endl;
    }

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

void
ZED_Recorder::startRecording(sl::Camera& camera) {
    if (recorder_status_.load() == Record_Status::RECORDING) {
        std::cerr << "ZED_Recorder::startRecording: Already recording." << std::endl;
        return;
    }

    // Get initial timestamp from camera for recording start time
    sl::Timestamp start_timestamp = camera.getTimestamp(sl::TIME_REFERENCE::IMAGE);
    start_timestamp_ns_ = start_timestamp.getNanoseconds();
    last_timestamp_ns_ = start_timestamp_ns_;
    recorded_frame_count_ = 0;

    // Get the camera information and initialize protobuf recording
    sl::CameraInformation cam_info = camera.getCameraInformation();
    recording_ = std::make_unique<orbis_slam::Recording>();

    // Set recording metadata using camera timestamp
    recording_->set_recording_id("zed_recording_" + std::to_string(start_timestamp_ns_));
    recording_->set_start_timestamp_ns(start_timestamp_ns_);
    recording_->set_camera_model("ZED2i");

    // Set calibration parameters
    sl::CalibrationParameters calib_params = cam_info.camera_configuration.calibration_parameters;

    // Left camera intrinsics
    orbis_slam::CameraIntrinsics* left_intrinsics = recording_->mutable_left_intrinsics();
    left_intrinsics->set_fx(calib_params.left_cam.fx);
    left_intrinsics->set_fy(calib_params.left_cam.fy);
    left_intrinsics->set_cx(calib_params.left_cam.cx);
    left_intrinsics->set_cy(calib_params.left_cam.cy);
    left_intrinsics->set_width(cam_info.camera_configuration.resolution.width);
    left_intrinsics->set_height(cam_info.camera_configuration.resolution.height);

    // Right camera intrinsics
    orbis_slam::CameraIntrinsics* right_intrinsics = recording_->mutable_right_intrinsics();
    right_intrinsics->set_fx(calib_params.right_cam.fx);
    right_intrinsics->set_fy(calib_params.right_cam.fy);
    right_intrinsics->set_cx(calib_params.right_cam.cx);
    right_intrinsics->set_cy(calib_params.right_cam.cy);
    right_intrinsics->set_width(cam_info.camera_configuration.resolution.width);
    right_intrinsics->set_height(cam_info.camera_configuration.resolution.height);

    // Stereo calibration
    orbis_slam::StereoCalibration* stereo_calib = recording_->mutable_stereo_calibration();
    stereo_calib->set_baseline(calib_params.stereo_transform.getTranslation()[0]);

    // Rotation matrix (row-major 3x3)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            stereo_calib->add_rotation(calib_params.stereo_transform.m[i * 4 + j]);
        }
    }

    // Translation vector
    sl::float3 T = calib_params.stereo_transform.getTranslation();
    stereo_calib->add_translation(T.x);
    stereo_calib->add_translation(T.y);
    stereo_calib->add_translation(T.z);

    recorder_status_.store(Record_Status::RECORDING);
    std::cout << "Recording started at timestamp: " << start_timestamp_ns_ << std::endl;
}

void
ZED_Recorder::addFrameToRecording(sl::Camera& camera, const sl::Timestamp& timestamp) {
    if (recorder_status_.load() != Record_Status::RECORDING) {
        std::cerr << "ZED_Recorder::addFrameToRecording: Not currently recording." << std::endl;
        return;
    }

    if (!recording_) {
        std::cerr << "ZED_Recorder::addFrameToRecording: Recording not initialized." << std::endl;
        return;
    }

    // Update last timestamp
    last_timestamp_ns_ = timestamp.getNanoseconds();

    // Get pose information
    sl::Pose zed_pose;
    camera.getPosition(zed_pose, sl::REFERENCE_FRAME::WORLD);

    // Get rotation matrix
    sl::Matrix3f rotation_matrix = zed_pose.getRotationMatrix();

    // Get the stereo images
    sl::Mat sl_left_image, sl_right_image;
    cv::Mat left_image, right_image;
    camera.retrieveImage(sl_left_image, sl::VIEW::LEFT);
    camera.retrieveImage(sl_right_image, sl::VIEW::RIGHT);
    left_image = toCvMat(sl_left_image);
    right_image = toCvMat(sl_right_image);

    // Get the (registered) depth map
    sl::Mat sl_depth_map;
    cv::Mat depth_map;
    camera.retrieveMeasure(sl_depth_map, sl::MEASURE::DEPTH);
    depth_map = toCvMat(sl_depth_map);

    // Add frame to protobuf recording
    addFrame(recorded_frame_count_, timestamp, left_image, right_image, depth_map, zed_pose, rotation_matrix);

    recorded_frame_count_++;

    // Print progress every 30 frames
    if (recorded_frame_count_ % 30 == 0) {
        std::cout << "Recorded " << recorded_frame_count_ << " frames..." << std::endl;
    }
}

void
ZED_Recorder::finishRecording(const std::string& filepath) {
    if (recorder_status_.load() != Record_Status::RECORDING) {
        std::cerr << "ZED_Recorder::finishRecording: Not currently recording." << std::endl;
        return;
    }

    if (!recording_) {
        std::cerr << "ZED_Recorder::finishRecording: No recording data to save." << std::endl;
        return;
    }

    std::cout << "Finishing recording. Total frames: " << recorded_frame_count_ << std::endl;

    // Save the protobuf recording to file
    try {
        saveRecording(filepath, last_timestamp_ns_);
    } catch (const std::exception& e) {
        std::cerr << "Failed to save protobuf recording: " << e.what() << std::endl;
        recorder_status_.store(Record_Status::IDLE);
        throw;
    }

    // Clean up
    recording_.reset();
    recorder_status_.store(Record_Status::IDLE);
    recorded_frame_count_ = 0;
    start_timestamp_ns_ = 0;
    last_timestamp_ns_ = 0;
}

void
ZED_Recorder::discardRecording() {
    if (recorder_status_.load() != Record_Status::RECORDING) {
        std::cerr << "ZED_Recorder::discardRecording: Not currently recording." << std::endl;
        return;
    }

    std::cout << "Discarding recording with " << recorded_frame_count_ << " frames." << std::endl;

    // Clean up without saving
    recording_.reset();
    recorder_status_.store(Record_Status::IDLE);
    recorded_frame_count_ = 0;
    start_timestamp_ns_ = 0;
    last_timestamp_ns_ = 0;
}

} /* namespace */