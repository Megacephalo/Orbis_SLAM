#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QSpinBox>
#include <QComboBox>
#include <QGroupBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QTimer>
#include <QDateTime>
#include <QMessageBox>
#include <QThread>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <array>

#include "orbis_slam/recorder_and_player/zed_backend_worker.h"

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
            // Enable texturing
            glEnable(GL_TEXTURE_2D);
            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

            // Draw left image
            glBindTexture(GL_TEXTURE_2D, textures[0]);
            glBegin(GL_QUADS);
            glTexCoord2f(0.0f, 0.0f); glVertex2f(0, 0);
            glTexCoord2f(1.0f, 0.0f); glVertex2f(width() / 2.0f, 0);
            glTexCoord2f(1.0f, 1.0f); glVertex2f(width() / 2.0f, height());
            glTexCoord2f(0.0f, 1.0f); glVertex2f(0, height());
            glEnd();

            // Draw right image
            glBindTexture(GL_TEXTURE_2D, textures[1]);
            glBegin(GL_QUADS);
            glTexCoord2f(0.0f, 0.0f); glVertex2f(width() / 2.0f, 0);
            glTexCoord2f(1.0f, 0.0f); glVertex2f(width(), 0);
            glTexCoord2f(1.0f, 1.0f); glVertex2f(width(), height());
            glTexCoord2f(0.0f, 1.0f); glVertex2f(width() / 2.0f, height());
            glEnd();

            glDisable(GL_TEXTURE_2D);
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

        // Draw grid/crosshair
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
        , demoFrame(0) {
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

            // Start recording via backend worker (transitions from LIVE to RECORDING)
            if (workerThread_ && worker_) {
                QMetaObject::invokeMethod(worker_, [this]() {
                    worker_->startRecording("zed_recording.svo");
                }, Qt::QueuedConnection);
            }
        } else {
            // Stop recording but continue live streaming
            isRecording = false;
            recordButton->setText("Start Recording");
            recordButton->setStyleSheet("");
            stereoViewer->setRecordingIndicator(false);

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
        QMessageBox::critical(this, "Camera Error", errorMessage);
    }

    void onCameraInitialized() {
        // Mark camera as initialized
        cameraInitialized_ = true;

        // Camera is ready, start live mode if in live/record mode
        if (currentMode_ == LIVE_RECORD_MODE && worker_) {
            QMetaObject::invokeMethod(worker_, "startLive", Qt::QueuedConnection);
        }
    }

    void onPlayPressed() {
        if (!isPlaying) {
            playbackTimer->start(33); // ~30 FPS
            playButton->setText("Pause");
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
        demoFrame = 0;
        frameSlider->blockSignals(true);
        frameSlider->setValue(0);
        frameSlider->blockSignals(false);
        updateFrameInfo(0);
        stereoViewer->displayDemoFrame();
    }

    void onFrameSliderChanged(int value) {
        if (!isPlaying) {
            updateFrameInfo(value);
            stereoViewer->displayDemoFrame();
        }
    }

    void onPlaybackTick() {
        demoFrame++;
        if (demoFrame > 1000) demoFrame = 0;
        frameSlider->blockSignals(true);
        frameSlider->setValue(demoFrame);
        frameSlider->blockSignals(false);
        updateFrameInfo(demoFrame);
        stereoViewer->displayDemoFrame();
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

    void onResolutionChanged() {
        QString res = resolutionCombo->currentText();
        resolutionLabel->setText(QString("Selected: %1").arg(res));
    }

    void onFPSChanged(int fps) {
        fpsLabel->setText(QString("FPS: %1").arg(fps));
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
        
        QHBoxLayout* buttonLayout = new QHBoxLayout();
        playButton = new QPushButton("Play");
        QPushButton* stopButton = new QPushButton("Stop");
        connect(playButton, &QPushButton::clicked, this, &ZEDMediaRecorderPlayer::onPlayPressed);
        connect(stopButton, &QPushButton::clicked, this, &ZEDMediaRecorderPlayer::onStopPressed);
        buttonLayout->addWidget(playButton);
        buttonLayout->addWidget(stopButton);
        playLayout->addLayout(buttonLayout);

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
        poseXLabel = new QLabel("Pose X: 0.000");
        poseYLabel = new QLabel("Pose Y: 0.000");
        poseZLabel = new QLabel("Pose Z: 0.000");
        frameLayout->addWidget(frameIndexLabel);
        frameLayout->addWidget(timestampLabel);
        frameLayout->addWidget(poseXLabel);
        frameLayout->addWidget(poseYLabel);
        frameLayout->addWidget(poseZLabel);
        layout->addWidget(frameGroup);

        // Recording settings
        QGroupBox* settingsGroup = new QGroupBox("Settings");
        QVBoxLayout* settingsLayout = new QVBoxLayout(settingsGroup);
        
        resolutionCombo = new QComboBox();
        resolutionCombo->addItem("1280x720");
        resolutionCombo->addItem("1920x1080");
        resolutionCombo->addItem("2560x1440");
        connect(resolutionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), 
                this, &ZEDMediaRecorderPlayer::onResolutionChanged);
        resolutionLabel = new QLabel("Selected: 1280x720");
        settingsLayout->addWidget(new QLabel("Resolution:"));
        settingsLayout->addWidget(resolutionCombo);
        settingsLayout->addWidget(resolutionLabel);

        QHBoxLayout* fpsLayout = new QHBoxLayout();
        QSpinBox* fpsSpinBox = new QSpinBox();
        fpsSpinBox->setMinimum(10);
        fpsSpinBox->setMaximum(120);
        fpsSpinBox->setValue(30);
        fpsLabel = new QLabel("FPS: 30");
        connect(fpsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), 
                this, &ZEDMediaRecorderPlayer::onFPSChanged);
        fpsLayout->addWidget(new QLabel("FPS:"));
        fpsLayout->addWidget(fpsSpinBox);
        settingsLayout->addLayout(fpsLayout);
        settingsLayout->addWidget(fpsLabel);

        layout->addWidget(settingsGroup);
        layout->addStretch();

        return panel;
    }

    StereoImageViewer* stereoViewer = nullptr;
    QPushButton* recordButton = nullptr;
    QPushButton* playButton = nullptr;
    QSlider* frameSlider = nullptr;
    QLabel* frameIndexLabel = nullptr;
    QLabel* timestampLabel = nullptr;
    QLabel* poseXLabel = nullptr;
    QLabel* poseYLabel = nullptr;
    QLabel* poseZLabel = nullptr;
    QComboBox* resolutionCombo = nullptr;
    QLabel* resolutionLabel = nullptr;
    QLabel* fpsLabel = nullptr;
    QTimer* playbackTimer = nullptr;

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
};

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    
    ZEDMediaRecorderPlayer window;
    window.show();
    
    return app.exec();
}

#include "ZEDRecPlay.moc"