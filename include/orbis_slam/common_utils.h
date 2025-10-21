#ifndef _ORBIS_SLAM_COMMON_UTILS_H_
#define _ORBIS_SLAM_COMMON_UTILS_H_

#include <Eigen/Dense>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <cmath>

namespace Orbis {

inline
double calculateTranslationDistance(const Sophus::SE3d& pose1, const Sophus::SE3d& pose2) {
    return (pose1.translation() - pose2.translation()).norm();
}

inline
double calculateRotationDistance(const Sophus::SE3d& pose1, const Sophus::SE3d& pose2) {
    // Extract the rotation parts (SO3)
    Sophus::SO3d R1 = pose1.so3();
    Sophus::SO3d R2 = pose2.so3();

    // Compute relative rotation
    Sophus::SO3d R_rel = R1.inverse() * R2;

    // Get the angle of rotation (in radians)
    double angle = R_rel.log().norm();
    return angle;
}

} /* namespace Orbis */

#endif /* _ORBIS_SLAM_COMMON_UTILS_H_ */