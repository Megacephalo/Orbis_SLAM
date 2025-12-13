#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QGroupBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QTimer>
#include <QDateTime>
#include <QMessageBox>
#include <QThread>
#include <QFileDialog>
#include <QLineEdit>
#include <QCheckBox>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <array>
#include <fstream>

#include "orbis_slam/recorder_and_player/zed_backend_worker.h"
#include "zed_recording.pb.h"

// OpenGL Widget for rendering stereo images and overlays
class StereoImageViewer : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    StereoImageViewer(QWidget* parent = nullptr) : QOpenGLWidget(parent), recordingIndicatorVisible(false), recordingFlash(true) {
        setMinimumSize(1280, 480);
        recordingTimer = new QTimer(this);
        connect(recordingTimer, &QTimer::timeout, this, [this]() {
            recordingFlash = !recordingFlash;
            update();
        });
    }

    void setFrameData(const cv::Mat& leftImage, const cv::Mat& rightImage,
                      int64_t timestamp, int frameIndex) {
        frameTimestamp = timestamp;
        frameIndex_ = frameIndex;

        if (leftImage.empty() || rightImage.empty()) {
            return;
        }

        // Update image dimensions
        imageWidth = leftImage.cols;
        imageHeight = leftImage.rows;

        // Convert BGR to RGB for OpenGL
        cv::Mat leftRGB, rightRGB;
        cv::cvtColor(leftImage, leftRGB, cv::COLOR_BGR2RGB);
        cv::cvtColor(rightImage, rightRGB, cv::COLOR_BGR2RGB);

        // Upload left image to texture
        makeCurrent();
        glBindTexture(GL_TEXTURE_2D, textures[0]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imageWidth, imageHeight, 0,
                     GL_RGB, GL_UNSIGNED_BYTE, leftRGB.data);

        // Upload right image to texture
        glBindTexture(GL_TEXTURE_2D, textures[1]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imageWidth, imageHeight, 0,
                     GL_RGB, GL_UNSIGNED_BYTE, rightRGB.data);
        doneCurrent();

        hasValidFrames = true;
        update();
    }

    void displayDemoFrame() {
        // Demo: just trigger a repaint
        update();
    }

    void clearFrames() {
        // Clear the display and show placeholders
        hasValidFrames = false;
        update();
    }

    void setRecordingIndicator(bool isRecording) {
        recordingIndicatorVisible = isRecording;
        if (isRecording) {
            recordingFlash = true;
            recordingTimer->start(500); // Flash every 500ms
        } else {
            recordingTimer->stop();
        }
        update();
    }

protected:
    void initializeGL() override {
        initializeOpenGLFunctions();
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        // Initialize texture IDs for left and right images
        glGenTextures(2, textures);
        for (int i = 0; i < 2; ++i) {
            glBindTexture(GL_TEXTURE_2D, textures[i]);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        }
    }

    void resizeGL(int w, int h) override {
        glViewport(0, 0, w, h);
    }

    void paintGL() override {
        glClear(GL_COLOR_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, width(), height(), 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        if (hasValidFrames) {
            // Calculate aspect-ratio-preserving dimensions for each image
            float imageAspect = static_cast<float>(imageWidth) / static_cast<float>(imageHeight);
            float halfWidgetWidth = width() / 2.0f;
            float widgetHeight = height();
            float halfWidgetAspect = halfWidgetWidth / widgetHeight;

            float displayWidth, displayHeight;
            float offsetX, offsetY;

            // Fit image to half-width while preserving aspect ratio
            if (imageAspect > halfWidgetAspect) {
                // Image is wider than the half-width space - fit to width
                displayWidth = halfWidgetWidth;
                displayHeight = halfWidgetWidth / imageAspect;
                offsetX = 0.0f;
                offsetY = (widgetHeight - displayHeight) / 2.0f;
            } else {
                // Image is taller than the half-width space - fit to height
                displayWidth = widgetHeight * imageAspect;
                displayHeight = widgetHeight;
                offsetX = (halfWidgetWidth - displayWidth) / 2.0f;
                offsetY = 0.0f;
            }

            // Enable texturing
            glEnable(GL_TEXTURE_2D);
            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

            // Draw left image (in left half)
            glBindTexture(GL_TEXTURE_2D, textures[0]);
            glBegin(GL_QUADS);
            glTexCoord2f(0.0f, 0.0f); glVertex2f(offsetX, offsetY);
            glTexCoord2f(1.0f, 0.0f); glVertex2f(offsetX + displayWidth, offsetY);
            glTexCoord2f(1.0f, 1.0f); glVertex2f(offsetX + displayWidth, offsetY + displayHeight);
            glTexCoord2f(0.0f, 1.0f); glVertex2f(offsetX, offsetY + displayHeight);
            glEnd();

            // Draw right image (in right half)
            float rightOffsetX = halfWidgetWidth + offsetX;
            glBindTexture(GL_TEXTURE_2D, textures[1]);
            glBegin(GL_QUADS);
            glTexCoord2f(0.0f, 0.0f); glVertex2f(rightOffsetX, offsetY);
            glTexCoord2f(1.0f, 0.0f); glVertex2f(rightOffsetX + displayWidth, offsetY);
            glTexCoord2f(1.0f, 1.0f); glVertex2f(rightOffsetX + displayWidth, offsetY + displayHeight);
            glTexCoord2f(0.0f, 1.0f); glVertex2f(rightOffsetX, offsetY + displayHeight);
            glEnd();

            glDisable(GL_TEXTURE_2D);

            // Draw background bars if there's letterboxing/pillarboxing
            if (offsetY > 0 || offsetX > 0) {
                glColor4f(0.1f, 0.1f, 0.1f, 1.0f);

                // Top and bottom bars (letterboxing)
                if (offsetY > 0) {
                    glBegin(GL_QUADS);
                    // Top bar left
                    glVertex2f(0, 0);
                    glVertex2f(halfWidgetWidth, 0);
                    glVertex2f(halfWidgetWidth, offsetY);
                    glVertex2f(0, offsetY);
                    // Bottom bar left
                    glVertex2f(0, offsetY + displayHeight);
                    glVertex2f(halfWidgetWidth, offsetY + displayHeight);
                    glVertex2f(halfWidgetWidth, widgetHeight);
                    glVertex2f(0, widgetHeight);
                    // Top bar right
                    glVertex2f(halfWidgetWidth, 0);
                    glVertex2f(width(), 0);
                    glVertex2f(width(), offsetY);
                    glVertex2f(halfWidgetWidth, offsetY);
                    // Bottom bar right
                    glVertex2f(halfWidgetWidth, offsetY + displayHeight);
                    glVertex2f(width(), offsetY + displayHeight);
                    glVertex2f(width(), widgetHeight);
                    glVertex2f(halfWidgetWidth, widgetHeight);
                    glEnd();
                }

                // Left and right bars (pillarboxing)
                if (offsetX > 0) {
                    glBegin(GL_QUADS);
                    // Left bar in left half
                    glVertex2f(0, 0);
                    glVertex2f(offsetX, 0);
                    glVertex2f(offsetX, widgetHeight);
                    glVertex2f(0, widgetHeight);
                    // Right bar in left half
                    glVertex2f(offsetX + displayWidth, 0);
                    glVertex2f(halfWidgetWidth, 0);
                    glVertex2f(halfWidgetWidth, widgetHeight);
                    glVertex2f(offsetX + displayWidth, widgetHeight);
                    // Left bar in right half
                    glVertex2f(halfWidgetWidth, 0);
                    glVertex2f(rightOffsetX, 0);
                    glVertex2f(rightOffsetX, widgetHeight);
                    glVertex2f(halfWidgetWidth, widgetHeight);
                    // Right bar in right half
                    glVertex2f(rightOffsetX + displayWidth, 0);
                    glVertex2f(width(), 0);
                    glVertex2f(width(), widgetHeight);
                    glVertex2f(rightOffsetX + displayWidth, widgetHeight);
                    glEnd();
                }
            }
        } else {
            // Draw placeholder rectangles when no frames available
            glColor4f(0.2f, 0.3f, 0.4f, 1.0f);
            glBegin(GL_QUADS);
            glVertex2f(0, 0);
            glVertex2f(width() / 2.0f, 0);
            glVertex2f(width() / 2.0f, height());
            glVertex2f(0, height());
            glEnd();

            glColor4f(0.3f, 0.2f, 0.4f, 1.0f);
            glBegin(GL_QUADS);
            glVertex2f(width() / 2.0f, 0);
            glVertex2f(width(), 0);
            glVertex2f(width(), height());
            glVertex2f(width() / 2.0f, height());
            glEnd();
        }

        // Draw center divider line
        glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
        glBegin(GL_LINES);
        glVertex2f(width() / 2.0f, 0);
        glVertex2f(width() / 2.0f, height());
        glEnd();

        // Draw recording indicator (red flashing dot in upper right)
        if (recordingIndicatorVisible && recordingFlash) {
            glColor4f(1.0f, 0.0f, 0.0f, 0.8f); // Red with transparency
            float dotRadius = 15.0f;
            float centerX = width() - 30.0f;
            float centerY = 30.0f;

            glBegin(GL_TRIANGLE_FAN);
            for (int i = 0; i < 32; ++i) {
                float angle = 2.0f * 3.14159f * i / 32.0f;
                float x = centerX + dotRadius * std::cos(angle);
                float y = centerY + dotRadius * std::sin(angle);
                glVertex2f(x, y);
            }
            glEnd();
        }
    }

private:
    int64_t frameTimestamp = 0;
    int frameIndex_ = 0;
    glm::mat4 cameraPose = glm::mat4(1.0f);
    bool recordingIndicatorVisible = false;
    bool recordingFlash = true;
    bool hasValidFrames = false;
    QTimer* recordingTimer = nullptr;
    GLuint textures[2];
    int imageWidth = 640;
    int imageHeight = 480;
};

// Main application window
class ZEDMediaRecorderPlayer : public QMainWindow {
    Q_OBJECT

    enum OperationMode {
        LIVE_RECORD_MODE,
        PLAYBACK_MODE
    };

public:
    ZEDMediaRecorderPlayer(QWidget* parent = nullptr)
        : QMainWindow(parent)
        , currentMode_(LIVE_RECORD_MODE)
        , cameraInitialized_(false)
        , isRecording(false)
        , isPlaying(false)
        , demoFrame(0)
        , loadingDialog_(nullptr) {
        setWindowTitle("ZED Media Recorder/Player - Stereo Visualizer");
        setGeometry(100, 100, 1600, 900);

        // Create central widget
        QWidget* centralWidget = new QWidget();
        setCentralWidget(centralWidget);
        QVBoxLayout* mainLayout = new QVBoxLayout(centralWidget);

        // Create splitter for viewer and controls
        QSplitter* splitter = new QSplitter(Qt::Horizontal);

        // Create stereo image viewer
        stereoViewer = new StereoImageViewer();
        splitter->addWidget(stereoViewer);

        // Control panel on the right
        QWidget* controlPanel = createControlPanel();
        splitter->addWidget(controlPanel);
        splitter->setSizes({1200, 300});

        mainLayout->addWidget(splitter);

        // Create playback timer
        playbackTimer = new QTimer(this);
        connect(playbackTimer, &QTimer::timeout, this, &ZEDMediaRecorderPlayer::onPlaybackTick);

        // Demo: simulate playback
        demoFrame = 0;

        // Initialize backend worker for camera operations
        initializeBackendWorker();

        // Set initial UI state based on mode
        updateControlsForMode();
    }

private slots:
    void onModeChanged() {
        // Determine which mode was selected
        if (liveRecordRadio->isChecked()) {
            currentMode_ = LIVE_RECORD_MODE;
        } else {
            currentMode_ = PLAYBACK_MODE;
        }

        // Stop any ongoing operations when switching modes
        if (isRecording) {
            onRecordPressed(); // Stop recording
        }
        if (isPlaying) {
            onStopPressed(); // Stop playback
        }

        // If switching to playback mode, explicitly stop the camera and clear display
        if (currentMode_ == PLAYBACK_MODE && worker_) {
            QMetaObject::invokeMethod(worker_, "stop", Qt::QueuedConnection);
            // Clear the viewer to show placeholders instead of last frame
            stereoViewer->clearFrames();
        }

        // Update UI based on mode
        updateControlsForMode();
    }

    void onRecordPressed() {
        if (!isRecording) {
            // Start recording while continuing live stream
            isRecording = true;
            recordButton->setText("Stop Recording");
            recordButton->setStyleSheet("background-color: #ff4444; color: white;");
            stereoViewer->setRecordingIndicator(true);

            std::cout << "[GUI] Starting recording..." << std::endl;

            // Start recording via backend worker (transitions from LIVE to RECORDING)
            if (workerThread_ && worker_) {
                QString filename = "zed_recording.svo";
                QMetaObject::invokeMethod(worker_, "startRecording", Qt::QueuedConnection,
                                        Q_ARG(QString, filename));
            }
        } else {
            // Stop recording but continue live streaming
            isRecording = false;
            recordButton->setText("Start Recording");
            recordButton->setStyleSheet("");
            stereoViewer->setRecordingIndicator(false);

            std::cout << "[GUI] Stopping recording..." << std::endl;

            // Stop recording, returns to live mode seamlessly
            if (workerThread_ && worker_) {
                QMetaObject::invokeMethod(worker_, "stopRecording", Qt::QueuedConnection);
            }
        }
    }

    void onFrameReceived(const cv::Mat& leftImage, const cv::Mat& rightImage,
                         int64_t timestamp, int frameIndex) {
        // Ignore camera frames when in playback mode
        if (currentMode_ == PLAYBACK_MODE) {
            return;
        }

        // Update the stereo viewer with new frames
        stereoViewer->setFrameData(leftImage, rightImage, timestamp, frameIndex);

        // Update frame info display
        updateFrameInfo(frameIndex);
    }

    void onCameraError(const QString& errorMessage) {
        // Dismiss the loading dialog if it's still showing
        if (loadingDialog_) {
            loadingDialog_->close();
            loadingDialog_->deleteLater();
            loadingDialog_ = nullptr;
        }

        QMessageBox::critical(this, "Camera Error", errorMessage);
    }

    void onCameraInitialized() {
        // Mark camera as initialized
        cameraInitialized_ = true;

        // Dismiss the loading dialog
        if (loadingDialog_) {
            loadingDialog_->close();
            loadingDialog_->deleteLater();
            loadingDialog_ = nullptr;
        }

        // Camera is ready, start live mode if in live/record mode
        if (currentMode_ == LIVE_RECORD_MODE && worker_) {
            QMetaObject::invokeMethod(worker_, "startLive", Qt::QueuedConnection);
        }
    }

    void onRecordingReadyToSave(bool hasData, int frameCount) {
        std::cout << "[GUI] onRecordingReadyToSave called! hasData=" << hasData
                  << ", frameCount=" << frameCount << std::endl;

        if (!hasData || frameCount == 0) {
            std::cout << "[GUI] No frames recorded, showing info message" << std::endl;
            QMessageBox::information(this, "Recording Complete",
                                    "No frames were recorded.");
            return;
        }

        std::cout << "[GUI] Opening file save dialog..." << std::endl;

        // Show file save dialog
        QString defaultFilename = QString("recording_%1.orbis")
            .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));

        QString filepath = QFileDialog::getSaveFileName(
            this,
            "Save Recording",
            defaultFilename,
            "ORBIS Recording Files (*.orbis);;All Files (*)"
        );

        if (filepath.isEmpty()) {
            // User clicked Cancel - show confirmation dialog
            QMessageBox::StandardButton reply = QMessageBox::warning(
                this,
                "Discard Recording?",
                QString("Are you sure you don't want to save the last recording?\n"
                       "This recording contains %1 frames and will be lost.").arg(frameCount),
                QMessageBox::Yes | QMessageBox::No,
                QMessageBox::No  // Default button
            );

            if (reply == QMessageBox::Yes) {
                // User confirmed discard
                if (worker_) {
                    QMetaObject::invokeMethod(worker_, "discardRecording", Qt::QueuedConnection);
                }
                QMessageBox::information(this, "Recording Discarded",
                                        "The recording has been discarded.");
            } else {
                // User wants to save - show the dialog again
                onRecordingReadyToSave(hasData, frameCount);
            }
        } else {
            // User selected a file path - save the recording
            if (worker_) {
                QMetaObject::invokeMethod(worker_, [this, filepath]() {
                    worker_->saveRecording(filepath);
                }, Qt::QueuedConnection);
            }
            QMessageBox::information(this, "Recording Saved",
                                    QString("Recording saved successfully to:\n%1").arg(filepath));
        }
    }

    void onBrowsePlaybackFile() {
        QString filepath = QFileDialog::getOpenFileName(
            this,
            "Open Recording",
            "",
            "ORBIS Recording Files (*.orbis);;All Files (*)"
        );

        if (!filepath.isEmpty()) {
            loadRecordingFile(filepath);
        }
    }

    void loadRecordingFile(const QString& filepath) {
        // Try to load the protobuf recording file
        std::ifstream input(filepath.toStdString(), std::ios::binary);
        if (!input) {
            QMessageBox::critical(this, "Load Error",
                QString("Failed to open file:\n%1").arg(filepath));
            return;
        }

        // Parse the protobuf recording
        auto recording = std::make_unique<orbis_slam::Recording>();
        if (!recording->ParseFromIstream(&input)) {
            QMessageBox::critical(this, "Parse Error",
                QString("Failed to parse recording file:\n%1\nThe file may be corrupted.").arg(filepath));
            return;
        }

        input.close();

        // Store the loaded recording
        loadedRecording_ = std::move(recording);
        playbackFilePath_ = filepath;
        totalPlaybackFrames_ = loadedRecording_->frames_size();
        currentPlaybackFrame_ = 0;

        // Update UI
        filePathEdit->setText(filepath);
        frameSlider->setMaximum(std::max(0, totalPlaybackFrames_ - 1));
        frameSlider->setValue(0);
        playButton->setEnabled(totalPlaybackFrames_ > 0);

        // Show info message
        QMessageBox::information(this, "Recording Loaded",
            QString("Successfully loaded recording:\n%1\n\nFrames: %2\nDuration: %3 ms")
                .arg(filepath)
                .arg(totalPlaybackFrames_)
                .arg((loadedRecording_->end_timestamp_ns() - loadedRecording_->start_timestamp_ns()) / 1000000));

        // Display first frame
        if (totalPlaybackFrames_ > 0) {
            displayPlaybackFrame(0);
        }
    }

    void onPlayPressed() {
        if (currentMode_ == PLAYBACK_MODE && !loadedRecording_) {
            QMessageBox::warning(this, "No Recording Loaded",
                "Please select a recording file first.");
            return;
        }

        if (!isPlaying) {
            playbackTimer->start(33); // ~30 FPS
            playButton->setText("Pause");
            stopButton->setEnabled(true);
        } else {
            playbackTimer->stop();
            playButton->setText("Play");
        }
        isPlaying = !isPlaying;
    }

    void onStopPressed() {
        playbackTimer->stop();
        isPlaying = false;
        playButton->setText("Play");
        stopButton->setEnabled(false);

        if (currentMode_ == PLAYBACK_MODE && loadedRecording_) {
            currentPlaybackFrame_ = 0;
            frameSlider->blockSignals(true);
            frameSlider->setValue(0);
            frameSlider->blockSignals(false);
            displayPlaybackFrame(0);
        } else {
            demoFrame = 0;
            frameSlider->blockSignals(true);
            frameSlider->setValue(0);
            frameSlider->blockSignals(false);
            updateFrameInfo(0);
            stereoViewer->displayDemoFrame();
        }
    }

    void onFrameSliderChanged(int value) {
        if (!isPlaying && currentMode_ == PLAYBACK_MODE && loadedRecording_) {
            currentPlaybackFrame_ = value;
            displayPlaybackFrame(value);
        } else if (!isPlaying) {
            updateFrameInfo(value);
            stereoViewer->displayDemoFrame();
        }
    }

    void onPlaybackTick() {
        if (currentMode_ == PLAYBACK_MODE && loadedRecording_) {
            // Playback mode: advance through recorded frames
            currentPlaybackFrame_++;
            if (currentPlaybackFrame_ >= totalPlaybackFrames_) {
                // Check if loop is enabled
                if (loopCheckbox && loopCheckbox->isChecked()) {
                    // Loop back to start
                    currentPlaybackFrame_ = 0;
                } else {
                    // Stop playback at the end and rewind to beginning
                    playbackTimer->stop();
                    isPlaying = false;
                    playButton->setText("Play");
                    stopButton->setEnabled(false);

                    // Rewind to the beginning for next playback
                    currentPlaybackFrame_ = 0;
                    frameSlider->blockSignals(true);
                    frameSlider->setValue(0);
                    frameSlider->blockSignals(false);
                    displayPlaybackFrame(0);
                    return; // Exit early to avoid displaying the last frame
                }
            }

            frameSlider->blockSignals(true);
            frameSlider->setValue(currentPlaybackFrame_);
            frameSlider->blockSignals(false);

            displayPlaybackFrame(currentPlaybackFrame_);
        } else {
            // Demo mode (old behavior)
            demoFrame++;
            if (demoFrame > 1000) demoFrame = 0;
            frameSlider->blockSignals(true);
            frameSlider->setValue(demoFrame);
            frameSlider->blockSignals(false);
            updateFrameInfo(demoFrame);
            stereoViewer->displayDemoFrame();
        }
    }

    void displayPlaybackFrame(int frameIndex) {
        if (!loadedRecording_ || frameIndex < 0 || frameIndex >= totalPlaybackFrames_) {
            return;
        }

        const orbis_slam::Frame& frame = loadedRecording_->frames(frameIndex);

        // Decompress left and right images
        cv::Mat leftImage = decompressImage(frame.left_image());
        cv::Mat rightImage = decompressImage(frame.right_image());

        if (leftImage.empty() || rightImage.empty()) {
            std::cerr << "Failed to decompress images for frame " << frameIndex << std::endl;
            return;
        }

        // Display the images
        stereoViewer->setFrameData(leftImage, rightImage, frame.timestamp_ns(), frameIndex);

        // Update frame info with real data
        frameIndexLabel->setText(QString("Frame: %1 / %2")
            .arg(frameIndex).arg(totalPlaybackFrames_ - 1));
        timestampLabel->setText(QString("Timestamp: %1 ms")
            .arg(frame.timestamp_ns() / 1000000));

        // Display pose if available
        if (frame.has_pose()) {
            const orbis_slam::CameraPose& pose = frame.pose();
            poseXLabel->setText(QString("Pose X: %1").arg(pose.tx(), 0, 'f', 3));
            poseYLabel->setText(QString("Pose Y: %1").arg(pose.ty(), 0, 'f', 3));
            poseZLabel->setText(QString("Pose Z: %1").arg(pose.tz(), 0, 'f', 3));
        } else {
            poseXLabel->setText("Pose X: N/A");
            poseYLabel->setText("Pose Y: N/A");
            poseZLabel->setText("Pose Z: N/A");
        }
    }

    cv::Mat decompressImage(const orbis_slam::CompressedImage& compressed) {
        if (compressed.data().empty()) {
            return cv::Mat();
        }

        // Convert bytes to vector for imdecode
        std::vector<uint8_t> buffer(compressed.data().begin(), compressed.data().end());

        // Decode based on format
        cv::Mat image;
        switch (compressed.format()) {
            case orbis_slam::CompressedImage::JPEG:
            case orbis_slam::CompressedImage::PNG:
            case orbis_slam::CompressedImage::WEBP:
                image = cv::imdecode(buffer, cv::IMREAD_COLOR);
                break;
            case orbis_slam::CompressedImage::RAW:
                // Raw format: reconstruct from dimensions and channels
                if (compressed.channels() == 3) {
                    image = cv::Mat(compressed.height(), compressed.width(), CV_8UC3,
                                    const_cast<char*>(compressed.data().data())).clone();
                } else if (compressed.channels() == 1) {
                    image = cv::Mat(compressed.height(), compressed.width(), CV_8UC1,
                                    const_cast<char*>(compressed.data().data())).clone();
                }
                break;
            default:
                std::cerr << "Unknown image format: " << compressed.format() << std::endl;
                break;
        }

        return image;
    }

    void updateFrameInfo(int frameIdx) {
        // Simulate frame data
        int64_t timestamp = frameIdx == 0 ? 0 : 1700000000000LL + frameIdx * 33; // Mock timestamp in ms
        frameIndexLabel->setText(QString("Frame: %1").arg(frameIdx));
        timestampLabel->setText(QString("Timestamp: %1 ms").arg(timestamp));

        // Mock pose data
        float x = std::sin(frameIdx * 0.01f) * 0.5f;
        float y = std::cos(frameIdx * 0.01f) * 0.3f;
        float z = frameIdx * 0.001f;
        poseXLabel->setText(QString("Pose X: %1").arg(x, 0, 'f', 3));
        poseYLabel->setText(QString("Pose Y: %1").arg(y, 0, 'f', 3));
        poseZLabel->setText(QString("Pose Z: %1").arg(z, 0, 'f', 3));
    }

    void updateControlsForMode() {
        // Enable/disable control groups based on current mode
        bool isLiveMode = (currentMode_ == LIVE_RECORD_MODE);
        bool isPlaybackMode = (currentMode_ == PLAYBACK_MODE);

        // Enable/disable recording controls
        recordGroup->setEnabled(isLiveMode);

        // Enable/disable playback controls
        playGroup->setEnabled(isPlaybackMode);

        // Update status message
        if (isLiveMode) {
            modeStatusLabel->setText("Mode: Live Streaming / Recording");
            modeStatusLabel->setStyleSheet("color: green; font-weight: bold;");

            // Start live streaming only if camera is already initialized
            // Otherwise, wait for onCameraInitialized() to be called
            if (worker_ && cameraInitialized_) {
                QMetaObject::invokeMethod(worker_, "startLive", Qt::QueuedConnection);
            }
        } else {
            modeStatusLabel->setText("Mode: Playback");
            modeStatusLabel->setStyleSheet("color: blue; font-weight: bold;");

            // Stop camera when switching to playback
            if (worker_) {
                QMetaObject::invokeMethod(worker_, "stop", Qt::QueuedConnection);
            }
        }
    }

    void initializeBackendWorker() {
        // Create and show simple loading message box
        loadingDialog_ = new QMessageBox(this);
        loadingDialog_->setWindowTitle("Initializing Camera");
        loadingDialog_->setText("Initializing ZED camera, please wait...");
        loadingDialog_->setStandardButtons(QMessageBox::NoButton);
        loadingDialog_->setWindowModality(Qt::WindowModal);
        loadingDialog_->show();

        // Create worker thread
        workerThread_ = new QThread(this);
        worker_ = new Orbis::ZEDBackendWorker();
        worker_->moveToThread(workerThread_);

        // Connect signals
        connect(worker_, &Orbis::ZEDBackendWorker::frameReady,
                this, &ZEDMediaRecorderPlayer::onFrameReceived);
        connect(worker_, &Orbis::ZEDBackendWorker::error,
                this, &ZEDMediaRecorderPlayer::onCameraError);
        connect(worker_, &Orbis::ZEDBackendWorker::cameraInitialized,
                this, &ZEDMediaRecorderPlayer::onCameraInitialized);
        connect(worker_, &Orbis::ZEDBackendWorker::recordingReadyToSave,
                this, &ZEDMediaRecorderPlayer::onRecordingReadyToSave);

        // Start thread
        workerThread_->start();

        // Initialize camera (runs in worker thread)
        QMetaObject::invokeMethod(worker_, "initializeCamera", Qt::QueuedConnection);
    }

private:
    QWidget* createControlPanel() {
        QWidget* panel = new QWidget();
        QVBoxLayout* layout = new QVBoxLayout(panel);

        // Mode selection group
        QGroupBox* modeGroup = new QGroupBox("Operation Mode");
        QVBoxLayout* modeLayout = new QVBoxLayout(modeGroup);

        // Radio buttons for mode selection
        liveRecordRadio = new QRadioButton("Live Streaming / Recording");
        playbackRadio = new QRadioButton("Playback");
        liveRecordRadio->setChecked(true); // Default to live mode

        // Button group to make them mutually exclusive
        QButtonGroup* modeButtonGroup = new QButtonGroup(this);
        modeButtonGroup->addButton(liveRecordRadio);
        modeButtonGroup->addButton(playbackRadio);

        // Connect mode change signal
        connect(liveRecordRadio, &QRadioButton::toggled, this, &ZEDMediaRecorderPlayer::onModeChanged);
        connect(playbackRadio, &QRadioButton::toggled, this, &ZEDMediaRecorderPlayer::onModeChanged);

        modeLayout->addWidget(liveRecordRadio);
        modeLayout->addWidget(playbackRadio);

        // Add status label
        modeStatusLabel = new QLabel("Mode: Live Streaming / Recording");
        modeStatusLabel->setStyleSheet("color: green; font-weight: bold;");
        modeLayout->addWidget(modeStatusLabel);

        layout->addWidget(modeGroup);

        // Recording controls
        recordGroup = new QGroupBox("Recording");
        QVBoxLayout* recordLayout = new QVBoxLayout(recordGroup);
        recordButton = new QPushButton("Start Recording");
        connect(recordButton, &QPushButton::clicked, this, &ZEDMediaRecorderPlayer::onRecordPressed);
        recordLayout->addWidget(recordButton);
        layout->addWidget(recordGroup);

        // Playback controls
        playGroup = new QGroupBox("Playback");
        QVBoxLayout* playLayout = new QVBoxLayout(playGroup);

        // File path selector
        QHBoxLayout* fileLayout = new QHBoxLayout();
        filePathEdit = new QLineEdit();
        filePathEdit->setPlaceholderText("Select a recording file to play...");
        filePathEdit->setReadOnly(true);
        QPushButton* browseButton = new QPushButton("Browse...");
        connect(browseButton, &QPushButton::clicked, this, &ZEDMediaRecorderPlayer::onBrowsePlaybackFile);
        fileLayout->addWidget(filePathEdit);
        fileLayout->addWidget(browseButton);
        playLayout->addLayout(fileLayout);

        QHBoxLayout* buttonLayout = new QHBoxLayout();
        playButton = new QPushButton("Play");
        playButton->setEnabled(false); // Disabled until file is loaded
        stopButton = new QPushButton("Stop");
        stopButton->setEnabled(false); // Disabled until playback starts
        connect(playButton, &QPushButton::clicked, this, &ZEDMediaRecorderPlayer::onPlayPressed);
        connect(stopButton, &QPushButton::clicked, this, &ZEDMediaRecorderPlayer::onStopPressed);
        buttonLayout->addWidget(playButton);
        buttonLayout->addWidget(stopButton);
        playLayout->addLayout(buttonLayout);

        // Loop checkbox
        loopCheckbox = new QCheckBox("Loop playback");
        loopCheckbox->setChecked(true); // Default to loop enabled
        playLayout->addWidget(loopCheckbox);

        frameSlider = new QSlider(Qt::Horizontal);
        frameSlider->setMinimum(0);
        frameSlider->setMaximum(1000);
        frameSlider->setValue(0);
        connect(frameSlider, &QSlider::valueChanged, this, &ZEDMediaRecorderPlayer::onFrameSliderChanged);
        playLayout->addWidget(frameSlider);

        layout->addWidget(playGroup);

        // Frame information
        QGroupBox* frameGroup = new QGroupBox("Frame Data");
        QVBoxLayout* frameLayout = new QVBoxLayout(frameGroup);
        frameIndexLabel = new QLabel("Frame: 0");
        timestampLabel = new QLabel("Timestamp: 0 ms");
        resolutionLabel = new QLabel("Resolution: 1280x720");
        fpsLabel = new QLabel("FPS: 30");
        poseXLabel = new QLabel("Pose X: 0.000");
        poseYLabel = new QLabel("Pose Y: 0.000");
        poseZLabel = new QLabel("Pose Z: 0.000");
        frameLayout->addWidget(frameIndexLabel);
        frameLayout->addWidget(timestampLabel);
        frameLayout->addWidget(resolutionLabel);
        frameLayout->addWidget(fpsLabel);
        frameLayout->addWidget(poseXLabel);
        frameLayout->addWidget(poseYLabel);
        frameLayout->addWidget(poseZLabel);
        layout->addWidget(frameGroup);

        layout->addStretch();

        return panel;
    }

    StereoImageViewer* stereoViewer = nullptr;
    QPushButton* recordButton = nullptr;
    QPushButton* playButton = nullptr;
    QPushButton* stopButton = nullptr;
    QSlider* frameSlider = nullptr;
    QLabel* frameIndexLabel = nullptr;
    QLabel* timestampLabel = nullptr;
    QLabel* resolutionLabel = nullptr;
    QLabel* fpsLabel = nullptr;
    QLabel* poseXLabel = nullptr;
    QLabel* poseYLabel = nullptr;
    QLabel* poseZLabel = nullptr;
    QTimer* playbackTimer = nullptr;
    QLineEdit* filePathEdit = nullptr;
    QCheckBox* loopCheckbox = nullptr;

    // Mode toggle components
    QRadioButton* liveRecordRadio = nullptr;
    QRadioButton* playbackRadio = nullptr;
    QLabel* modeStatusLabel = nullptr;
    QGroupBox* recordGroup = nullptr;
    QGroupBox* playGroup = nullptr;
    OperationMode currentMode_;

    // Backend worker for camera operations
    QThread* workerThread_ = nullptr;
    Orbis::ZEDBackendWorker* worker_ = nullptr;
    bool cameraInitialized_ = false;

    bool isRecording = false;
    bool isPlaying = false;
    int demoFrame = 0;

    // Playback state
    QString playbackFilePath_;
    std::unique_ptr<orbis_slam::Recording> loadedRecording_;
    int currentPlaybackFrame_ = 0;
    int totalPlaybackFrames_ = 0;

    // Loading message box for camera initialization
    QMessageBox* loadingDialog_ = nullptr;
};

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    
    ZEDMediaRecorderPlayer window;
    window.show();
    
    return app.exec();
}

#include "ZEDRecPlay.moc"