#include "orbis_slam/recorder_and_player/zed_backend_worker.h"
#include <iostream>
#include <chrono>
#include <thread>

namespace Orbis {

ZEDBackendWorker::ZEDBackendWorker(QObject* parent)
    : QObject(parent)
    , mode_(STOPPED)
    , shouldStop_(false)
    , cameraReady_(false)
    , frameCounter_(0)
{
}

ZEDBackendWorker::~ZEDBackendWorker() {
    stop();
}

void ZEDBackendWorker::initializeCamera() {
    try {
        zedWrapper_ = std::make_unique<ZEDWrapper>();

        // Initialize the camera
        if (!zedWrapper_->setup()) {
            cameraReady_.store(false);
            emit error("Failed to initialize ZED camera");
            return;
        }

        // Mark camera as ready before emitting signal
        cameraReady_.store(true);
        emit cameraInitialized();
    } catch (const std::exception& e) {
        cameraReady_.store(false);
        emit error(QString("Camera initialization error: %1").arg(e.what()));
    }
}

void ZEDBackendWorker::startLive() {
    if (!zedWrapper_ || !cameraReady_.load()) {
        emit error("Camera not initialized. Call initializeCamera() first.");
        return;
    }

    if (mode_.load() != STOPPED) {
        // Silently ignore if already running to avoid duplicate error messages
        return;
    }

    mode_.store(LIVE);
    shouldStop_.store(false);
    frameCounter_ = 0;

    emit liveStarted();

    // Start capture loop
    captureLoop();
}

void ZEDBackendWorker::startRecording(const QString& filename) {
    if (!zedWrapper_ || !cameraReady_.load()) {
        emit error("Camera not initialized. Call initializeCamera() first.");
        return;
    }

    WorkerMode currentMode = mode_.load();

    // If already recording, ignore
    if (currentMode == RECORDING) {
        return;
    }

    // If currently in LIVE mode, transition to RECORDING mode seamlessly
    if (currentMode == LIVE) {
        // Just change the mode - the capture loop is already running
        mode_.store(RECORDING);
        emit recordingStarted();
        return;
    }

    // If stopped, start fresh capture loop in recording mode
    if (currentMode == STOPPED) {
        // TODO: Integrate with ZED_Recorder for actual recording
        // For now, just treat as live mode with recording indicator
        mode_.store(RECORDING);
        shouldStop_.store(false);
        frameCounter_ = 0;

        emit recordingStarted();

        // Start capture loop
        captureLoop();
    }
}

void ZEDBackendWorker::stopRecording() {
    WorkerMode currentMode = mode_.load();

    // If currently recording, transition back to live mode
    if (currentMode == RECORDING) {
        // Just change the mode - keep the capture loop running
        mode_.store(LIVE);
        emit liveStarted();
    }
    // If not recording, do nothing
}

void ZEDBackendWorker::stop() {
    shouldStop_.store(true);
    mode_.store(STOPPED);

    // Wait a bit for the loop to exit
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    emit stopped();
}

void ZEDBackendWorker::captureLoop() {
    while (!shouldStop_.load() && mode_.load() != STOPPED) {
        // Check if new frame is available
        if (zedWrapper_->hasNewFrame()) {
            // Grab the frame
            zedWrapper_->grabFrame();

            // Get left and right images
            // IMPORTANT: Clone the images to make them thread-safe for cross-thread Qt signals
            // OpenCV Mat uses reference counting that is not thread-safe by default
            cv::Mat leftImage = zedWrapper_->getLeftImage().clone();
            cv::Mat rightImage = zedWrapper_->getRightImage().clone();

            // Get timestamp (use ZED timestamp)
            auto now = std::chrono::system_clock::now();
            auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count();

            // Emit frame for display
            emit frameReady(leftImage, rightImage, timestamp, frameCounter_);

            frameCounter_++;
        } else {
            // Small sleep to avoid busy-waiting if grab fails
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    emit stopped();
}

} // namespace Orbis
