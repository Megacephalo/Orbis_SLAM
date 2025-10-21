#ifndef _POSE_OPTIMIZER_H_
#define _POSE_OPTIMIZER_H_

#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <sophus/se3.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sl/Camera.hpp>

#include "orbis_slam/zed_setup_help_utils.h"
#include "orbis_slam/essential_data_structure.h"

namespace Orbis {

class PoseOptimizer {
  private:
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;

    std::unique_ptr<LinearSolverType> linear_solver_;
    std::unique_ptr<BlockSolverType> block_solver_;
    g2o::OptimizationAlgorithmLevenberg* solver_;

    g2o::SparseOptimizer optimizer_;

    std::thread optimization_thread_;
    std::condition_variable cv_;
    std::mutex mutex_;
    std::atomic<bool> should_stop_;
    std::atomic<bool> optimization_requested_;
    std::atomic<bool> is_optimizing_;

    uint64_t keyframe_count_;
    uint64_t last_optimized_keyframe_count_;
    uint64_t min_keyframes_between_optimizations_;
    uint64_t optimization_iterations_;
    double last_optimization_time_; // seconds
    double optimization_interval_; // seconds
    Trajectory::Ptr trajectory_;

  public:
    PoseOptimizer(Trajectory::Ptr trajectory = nullptr);
    ~PoseOptimizer();

    void setTrajectory(Trajectory::Ptr trajectory);
    
    bool shouldOptimize(const double& current_time);
    bool shouldOptimizeByCount();
    void requestOptimization(const double& current_time);

    int addPoseVertex(const Frame::Ptr& curr_frame, bool fixed = false);

    void addRelativeMotionEdgeFromPoses(
        const Frame::Ptr& from_frame, const Frame::Ptr& to_frame,
        const Eigen::Matrix<double, 6, 6>& information
    );

    void addRelativeMotionEdge(
        const int& from_id, const int& to_id, 
        const Sophus::SE3d& relative_motion, 
        const Eigen::Matrix<double, 6, 6>& infoormation
    );

    void optimizeLoop();

    Sophus::SE3d getPose(const int& vertex_id);

    void save(const std::string& save_to_file);

    static Eigen::Matrix<double, 6, 6> createInformationMatrix(const sl::Pose& zed_pose);
};

} /* namespace Orbis */

#endif /* _POSE_OPTIMIZER_H_ */