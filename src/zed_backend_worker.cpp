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
    , recordedFrameCount_(0)
    , maxRecordingDurationSeconds_(0) // 0 means no timeout
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

    // Start capture loop in a separate thread to avoid blocking Qt event loop
    std::thread([this]() {
        captureLoop();
    }).detach();
}

void ZEDBackendWorker::startRecording(const QString& filename) {
    std::cout << "[Worker] startRecording called with filename: " << filename.toStdString() << std::endl;

    if (!zedWrapper_ || !cameraReady_.load()) {
        std::cout << "[Worker] Error: Camera not initialized!" << std::endl;
        emit error("Camera not initialized. Call initializeCamera() first.");
        return;
    }

    WorkerMode currentMode = mode_.load();
    std::cout << "[Worker] Current mode: " << currentMode << std::endl;

    // If already recording, ignore
    if (currentMode == RECORDING) {
        std::cout << "[Worker] Already recording, ignoring." << std::endl;
        return;
    }

    // Store the pending filepath for later save
    pendingRecordingFilepath_ = filename;

    // Create a new recorder if needed (using the parameterless constructor)
    if (!zedRecorder_) {
        try {
            zedRecorder_ = std::make_unique<ZED_Recorder>();
        } catch (const std::exception& e) {
            emit error(QString("Failed to create recorder: %1").arg(e.what()));
            return;
        }
    }

    // Start the recording session
    try {
        zedRecorder_->startRecording(zedWrapper_->getCamera());
        recordedFrameCount_ = 0;
        recordingStartTime_ = std::chrono::steady_clock::now();
    } catch (const std::exception& e) {
        emit error(QString("Failed to start recording: %1").arg(e.what()));
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
        mode_.store(RECORDING);
        shouldStop_.store(false);
        frameCounter_ = 0;

        emit recordingStarted();

        // Start capture loop in a separate thread to avoid blocking Qt event loop
        std::thread([this]() {
            captureLoop();
        }).detach();
    }
}

void ZEDBackendWorker::stopRecording() {
    std::cout << "[Worker] stopRecording called" << std::endl;

    WorkerMode currentMode = mode_.load();
    std::cout << "[Worker] Current mode: " << currentMode << std::endl;

    // If currently recording, stop and prompt for save
    if (currentMode == RECORDING) {
        // Transition back to live mode but keep the recording data
        mode_.store(LIVE);

        // Emit signal to prompt user for save/discard decision
        int frameCount = zedRecorder_ ? static_cast<int>(zedRecorder_->getFrameCount()) : 0;
        bool hasData = (frameCount > 0);
        std::cout << "[Worker] Recording stopped. Frame count: " << frameCount
                  << ", hasData: " << hasData << std::endl;
        std::cout << "[Worker] Emitting recordingReadyToSave signal..." << std::endl;
        emit recordingReadyToSave(hasData, frameCount);
        emit liveStarted();
    } else {
        std::cout << "[Worker] Not recording, doing nothing." << std::endl;
    }
    // If not recording, do nothing
}

void ZEDBackendWorker::saveRecording(const QString& filepath) {
    if (!zedRecorder_) {
        emit error("No recording to save.");
        return;
    }

    try {
        std::string stdFilepath = filepath.toStdString();
        zedRecorder_->finishRecording(stdFilepath);
        std::cout << "Recording saved successfully to: " << stdFilepath << std::endl;
    } catch (const std::exception& e) {
        emit error(QString("Failed to save recording: %1").arg(e.what()));
    }
}

void ZEDBackendWorker::discardRecording() {
    if (!zedRecorder_) {
        return;
    }

    try {
        zedRecorder_->discardRecording();
        std::cout << "Recording discarded." << std::endl;
    } catch (const std::exception& e) {
        emit error(QString("Failed to discard recording: %1").arg(e.what()));
    }
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

            // Get ZED timestamp
            sl::Timestamp zedTimestamp = zedWrapper_->getCamera().getTimestamp(sl::TIME_REFERENCE::IMAGE);

            // If recording, add frame to recorder
            if (mode_.load() == RECORDING && zedRecorder_ && zedRecorder_->isRecording()) {
                try {
                    zedRecorder_->addFrameToRecording(zedWrapper_->getCamera(), zedTimestamp);
                    recordedFrameCount_++;

                    // Check for timeout if configured
                    if (maxRecordingDurationSeconds_ > 0) {
                        auto elapsed = std::chrono::steady_clock::now() - recordingStartTime_;
                        auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
                        if (elapsedSeconds >= maxRecordingDurationSeconds_) {
                            std::cout << "Recording timeout reached (" << maxRecordingDurationSeconds_
                                      << " seconds). Stopping recording." << std::endl;
                            stopRecording();
                        }
                    }
                } catch (const std::exception& e) {
                    emit error(QString("Error adding frame to recording: %1").arg(e.what()));
                }
            }

            // Get left and right images
            // IMPORTANT: Clone the images to make them thread-safe for cross-thread Qt signals
            // OpenCV Mat uses reference counting that is not thread-safe by default
            cv::Mat leftImage = zedWrapper_->getLeftImage().clone();
            cv::Mat rightImage = zedWrapper_->getRightImage().clone();

            // Get timestamp in nanoseconds
            int64_t timestamp = static_cast<int64_t>(zedTimestamp.getNanoseconds());

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
