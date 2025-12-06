#ifndef _ZED_BACKEND_WORKER_H_
#define _ZED_BACKEND_WORKER_H_

#include <QObject>
#include <QThread>
#include <QString>
#include <atomic>
#include <memory>
#include <array>

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>

#include "orbis_slam/zed_wrapper.h"

namespace Orbis {

/**
 * @brief Backend worker for ZED camera operations in a separate thread
 *
 * This class manages the ZED camera in a separate thread to avoid blocking
 * the GUI. It can run in two modes:
 * - LIVE: Continuously capture and emit frames for live streaming
 * - RECORDING: Capture frames and save them while emitting for preview
 */
class ZEDBackendWorker : public QObject {
    Q_OBJECT

public:
    enum WorkerMode {
        STOPPED,
        LIVE,
        RECORDING
    };

    explicit ZEDBackendWorker(QObject* parent = nullptr);
    ~ZEDBackendWorker();

signals:
    /**
     * @brief Emitted when a new frame is ready for display
     * @param leftImage Left stereo image (OpenCV Mat)
     * @param rightImage Right stereo image (OpenCV Mat)
     * @param timestamp Frame timestamp in nanoseconds
     * @param frameIndex Current frame index
     */
    void frameReady(const cv::Mat& leftImage, const cv::Mat& rightImage,
                    int64_t timestamp, int frameIndex);

    /**
     * @brief Emitted when an error occurs
     * @param errorMessage Description of the error
     */
    void error(const QString& errorMessage);

    /**
     * @brief Emitted when camera is successfully initialized
     */
    void cameraInitialized();

    /**
     * @brief Emitted when live mode starts
     */
    void liveStarted();

    /**
     * @brief Emitted when recording starts
     */
    void recordingStarted();

    /**
     * @brief Emitted when worker stops
     */
    void stopped();

public slots:
    /**
     * @brief Initialize the ZED camera
     */
    void initializeCamera();

    /**
     * @brief Start live streaming mode
     */
    void startLive();

    /**
     * @brief Start recording mode
     * @param filename Path to save the recording
     */
    void startRecording(const QString& filename);

    /**
     * @brief Stop recording and return to live streaming mode
     */
    void stopRecording();

    /**
     * @brief Stop current operation completely
     */
    void stop();

private:
    void captureLoop();

    std::unique_ptr<ZEDWrapper> zedWrapper_;
    std::atomic<WorkerMode> mode_;
    std::atomic<bool> shouldStop_;
    std::atomic<bool> cameraReady_;
    int frameCounter_;
};

} // namespace Orbis

#endif // _ZED_BACKEND_WORKER_H_
