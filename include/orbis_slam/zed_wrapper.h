#ifndef _ZED_SETUP_H_
#define _ZED_SETUP_H_

#include <iostream>
#include <chrono>
#include <ctime>
#include <string>

#include <sl/Camera.hpp>

#include "orbis_slam/zed_setup_help_utils.h"

namespace Orbis {

class ZEDWrapper {
  private:
    sl::Camera zed_;
    sl::Mat frame_left_image_;
    sl::Mat frame_right_image_;
    sl::Mat stereo_image_; // only for visualization purpose
    sl::Mat frame_point_cloud_;
    sl::Pose T_w_c_;
    sl::Pose T_prev_curr_;
    sl::CalibrationParameters calibration_parameters_;

  public:
    ZEDWrapper() = default;
    ~ZEDWrapper() {
        frame_point_cloud_.free();
        zed_.disablePositionalTracking();
        zed_.close();
    }

    bool setup(int argc = 0, char** argv = nullptr) {
        // Set configuration parameters for the ZED
        InitParameters init_parameters;
        init_parameters.camera_resolution = sl::RESOLUTION::AUTO;
        init_parameters.depth_mode = DEPTH_MODE::NEURAL;
        // init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD; // ROS standards
        init_parameters.coordinate_system = COORDINATE_SYSTEM::IMAGE; // ZED default
        init_parameters.coordinate_units = UNIT::METER;
        init_parameters.sensors_required = true; // enable IMU
        init_parameters.sdk_verbose = 1;
        int res_arg = parseArgs(argc, argv, init_parameters);
        (void)res_arg; // mark as intentionally unused to suppress warning

        // Open the camera
        auto returned_state = zed_.open(init_parameters);
        if (returned_state != ERROR_CODE::SUCCESS) {
            print("Camera Open", returned_state, "Exit program.");
            return false;
        }

        // extract the calibration parameters
        calibration_parameters_ = zed_.getCameraInformation().camera_configuration.calibration_parameters;

        // position tracking
        sl::PositionalTrackingParameters positional_tracking_parameters;
        positional_tracking_parameters.mode = POSITIONAL_TRACKING_MODE::GEN_3; // latest tracking mode
        positional_tracking_parameters.enable_imu_fusion = true; // fuse IMU data
        auto tracking_state = zed_.enablePositionalTracking(positional_tracking_parameters);
        if (tracking_state != ERROR_CODE::SUCCESS) {
            std::cerr << "Position tracking error: " << tracking_state << std::endl;
            return false;
        }

        return true;
    } /* setup() */

    bool hasNewFrame() {
        sl::ERROR_CODE err = zed_.grab();
        return (err == ERROR_CODE::SUCCESS);
    } /* hasNewFrame() */

    void grabFrame() {
        zed_.retrieveImage(frame_left_image_, VIEW::LEFT);
        zed_.retrieveImage(frame_right_image_, VIEW::RIGHT);
        zed_.retrieveImage(stereo_image_, VIEW::SIDE_BY_SIDE);
        zed_.retrieveMeasure(frame_point_cloud_, MEASURE::XYZRGBA, MEM::CPU); // retrieve to CPU for memory access on ROS2
        zed_.getPosition(T_w_c_, REFERENCE_FRAME::WORLD);
        zed_.getPosition(T_prev_curr_, REFERENCE_FRAME::CAMERA);
    } /* grabFrame() */

    bool get3DCorrespondences(const std::vector<cv::Point2f>& features_2d, 
                              std::vector<cv::Point3f>& points_3d) {
        points_3d.clear();
        points_3d.reserve(features_2d.size());

        sl::float4 landmark_coords;

        for (const auto& feat : features_2d) {
            int x = static_cast<int>(feat.x);
            int y = static_cast<int>(feat.y);

            // get 3D point coordinates at pixel location
            frame_point_cloud_.getValue(x, y, &landmark_coords);

            // check if point is valid (not NaN / infinite)
            if ( ! std::isfinite(landmark_coords.x) ||
                 ! std::isfinite(landmark_coords.y) ||
                 ! std::isfinite(landmark_coords.z) ) {
                continue;
            }

            points_3d.emplace_back(
                landmark_coords.x, 
                landmark_coords.y, 
                landmark_coords.z
            );
        } /* for */
        return ( ! points_3d.empty() );
    }

    sl::Camera& getZED() { return zed_; }
    sl::Mat& sl_getLeftImage() { return frame_left_image_; }
    sl::Mat& sl_getRightImage() { return frame_right_image_; }
    sl::Mat& sl_getStereoImage() { return stereo_image_; }
    sl::Mat& sl_getPointCloud() { return frame_point_cloud_; }
    sl::Pose& sl_getPose() { return T_w_c_; }
    sl::Pose& sl_getIncrementalMotion() { return T_prev_curr_; }

    cv::Mat getLeftImage() { return toCvMat(frame_left_image_); }
    cv::Mat getRightImage() { return toCvMat(frame_right_image_); }
    cv::Mat getStereoImage() { return toCvMat(stereo_image_); }

    int getCameraFPS() const {
        return static_cast<int>(zed_.getCameraInformation().camera_configuration.fps);
    }

    double fx_left() const { return calibration_parameters_.left_cam.fx; }
    double fy_left() const { return calibration_parameters_.left_cam.fy; }
    double cx_left() const { return calibration_parameters_.left_cam.cx; }
    double cy_left() const { return calibration_parameters_.left_cam.cy; }
    double fx_right() const { return calibration_parameters_.right_cam.fx; }
    double fy_right() const { return calibration_parameters_.right_cam.fy; }
    double cx_right() const { return calibration_parameters_.right_cam.cx; }
    double cy_right() const { return calibration_parameters_.right_cam.cy; }
    double baseline() const { return calibration_parameters_.stereo_transform.getTranslation()[0]; }
    bool hasIMU() const { return zed_.getCameraInformation().sensors_configuration.isSensorAvailable(sl::SENSOR_TYPE::GYROSCOPE); }
};

} /* namespace Orbis */


#endif /* _ZED_SETUP_H_ */