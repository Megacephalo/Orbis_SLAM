#include "orbis_slam/pose_optimizer.h"

namespace Orbis {

PoseOptimizer::PoseOptimizer(Trajectory::Ptr trajectory) 
: should_stop_(false)
, is_optimizing_(false)
, keyframe_count_(0)
, last_optimized_keyframe_count_(0)
, min_keyframes_between_optimizations_(3)
, optimization_iterations_(10)
, last_optimization_time_(0.0)
, optimization_interval_(2.0) // seconds
, trajectory_(trajectory)
{
    linear_solver_ = std::make_unique<LinearSolverType>();
    block_solver_ = std::make_unique<BlockSolverType>(std::move(linear_solver_));
    solver_ = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver_));

    optimizer_.setAlgorithm(solver_);
    optimizer_.setVerbose(false);

    // start running the optimization thread
    optimization_thread_ = std::thread(&PoseOptimizer::optimizeLoop, this);
} /* ctor */

PoseOptimizer::~PoseOptimizer() {
    should_stop_ = true;
    cv_.notify_all();
    if ( optimization_thread_.joinable() ) {
        optimization_thread_.join();
    }
} /* dtor */

void
PoseOptimizer::setTrajectory(Trajectory::Ptr trajectory) {
    std::lock_guard<std::mutex> lock(mutex_);
    trajectory_ = trajectory;
}

bool
PoseOptimizer::shouldOptimize(const double& current_time){
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (trajectory_->size() - last_optimized_keyframe_count_ < min_keyframes_between_optimizations_)  return false;

    if (is_optimizing_)  return false;

    if ( (current_time - last_optimization_time_) < optimization_interval_ )  return false;

    return true;
}

bool
PoseOptimizer::shouldOptimizeByCount() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (is_optimizing_)  return false;
    
    if (trajectory_->size() - last_optimized_keyframe_count_ < min_keyframes_between_optimizations_)  return false;

    return true;
}

void
PoseOptimizer::requestOptimization(const double& current_time) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (is_optimizing_)  return;
        optimization_requested_ = true;
        last_optimization_time_ = current_time;
    }
    cv_.notify_one();
}

int
PoseOptimizer::addPoseVertex(const Frame::Ptr& curr_frame, bool fixed) {
    g2o::VertexSE3* v_se3 = new g2o::VertexSE3();
    v_se3->setId(curr_frame->id);
    Sophus::SE3d curr_pose = curr_frame->pose;
    v_se3->setEstimate(g2o::SE3Quat(curr_pose.rotationMatrix(), curr_pose.translation()));
    v_se3->setFixed(fixed);
    optimizer_.addVertex(v_se3);

    return v_se3->id();
} /* addPoseVertex */

void
PoseOptimizer::addRelativeMotionEdgeFromPoses(
        const Frame::Ptr& from_frame, const Frame::Ptr& to_frame,
        const Eigen::Matrix<double, 6, 6>& information) {

    Sophus::SE3d relative_motion = from_frame->pose.inverse() * to_frame->pose;
    addRelativeMotionEdge(from_frame->id, to_frame->id, relative_motion, information);
}

void
PoseOptimizer::addRelativeMotionEdge(
    const int& from_id, const int& to_id, 
    const Sophus::SE3d& relative_motion, 
    const Eigen::Matrix<double, 6, 6>& infoormation) {

    g2o::EdgeSE3* edge_se3 = new g2o::EdgeSE3();

    // set the vertices that this edge connects
    g2o::OptimizableGraph::Vertex* v_from = optimizer_.vertex(from_id);
    g2o::OptimizableGraph::Vertex* v_to = optimizer_.vertex(to_id);
    edge_se3->setVertex(0, v_from);
    edge_se3->setVertex(1, v_to);

    // set the measurement (relative motion from previous to ccurrent)
    edge_se3->setMeasurement(g2o::SE3Quat(relative_motion.rotationMatrix(), relative_motion.translation()));

    // set the information matrix (inverse of covariance)
    edge_se3->setInformation(infoormation);

    optimizer_.addEdge(edge_se3);
}

void
PoseOptimizer::optimizeLoop() {

    while ( ! should_stop_ ) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return optimization_requested_ || should_stop_; });

        if ( should_stop_ )  break;

        if (optimization_requested_) {
            is_optimizing_ = true;
            optimization_requested_ = false;
            last_optimized_keyframe_count_ = trajectory_->size();
            lock.unlock();

            optimizer_.initializeOptimization();
            optimizer_.optimize(optimization_iterations_);

            lock.lock();

            trajectory_->forEachMutable([this](Frame& frame) {
                g2o::OptimizableGraph::Vertex* v = optimizer_.vertex(frame.id);
                g2o::VertexSE3* v_se3 = dynamic_cast<g2o::VertexSE3*>(v);
                if (v_se3) {
                    frame.pose = Sophus::SE3d(v_se3->estimate().rotation(), v_se3->estimate().translation());
                }
            });

            is_optimizing_ = false;
            lock.unlock();
        } /* if */
    } /* while */
}

Sophus::SE3d
PoseOptimizer::getPose(const int& vertex_id) {
    g2o::OptimizableGraph::Vertex* v = optimizer_.vertex(vertex_id);
    g2o::VertexSE3* v_se3 = dynamic_cast<g2o::VertexSE3*>(v);
    if (v_se3) {
        return Sophus::SE3d(v_se3->estimate().rotation(), v_se3->estimate().translation());
    }
    return Sophus::SE3d();
}

void
PoseOptimizer::save(const std::string& save_to_file) {
    optimizer_.save(save_to_file.c_str());
}

Eigen::Matrix<double, 6, 6>
PoseOptimizer::createInformationMatrix(const sl::Pose& zed_pose) {
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();

    int confidence = zed_pose.pose_confidence;
    double weight = confidence / 100.0;

    // translation uncertainty
    information.block<3, 3>(0, 0) *= (weight * 100.0);

    // rotation uncertainty
    information.block<3, 3>(3, 3) *= (weight * 100.0);

    return information;
}

} /* namespace Orbis */