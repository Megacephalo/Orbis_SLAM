<div align="center">

![Orbis SLAM Logo](documentation/orbis_logo_cropped.jpeg)

# Orbis SLAM

**專為 ROS 2 打造的現代化視覺慣性雙目 SLAM 系統**

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen)](https://github.com/Megacephalo/Orbis_SLAM)

[English](README.md) | **繁體中文**

</div>

---

## 目錄

- [概覽](#概覽)
- [功能特色](#功能特色)
- [相依套件](#相依套件)
- [安裝](#安裝)
- [快速上手](#快速上手)
- [使用方式](#使用方式)
- [組態設定](#組態設定)
- [系統架構](#系統架構)
- [開發者文件](#開發者文件)
- [發展藍圖](#發展藍圖)
- [參與貢獻](#參與貢獻)
- [授權條款](#授權條款)

## 概覽

Orbis SLAM 是一套現代化、即時運作的視覺慣性雙目同步定位與地圖建構（SLAM）系統，專為 ROS 2 與 Stereolabs ZED 相機所設計。本系統以效能與精準度為核心，建立在優秀的 ZED SDK 之上，運用先進的電腦視覺演算法與最佳化技術，為機器人應用提供穩健的定位與建圖能力。

本系統特別針對 Stereolabs ZED 相機配置進行最佳化，並提供：

- **即時效能**：針對即時機器人應用最佳化
- **穩健追蹤**：進階的關鍵影格選擇與位姿最佳化
- **原生支援 ROS 2**：從零開始為 ROS 2 生態系打造
- **跨平台相容**：可同時在 x86 與 ARM 架構的機器上執行
- **模組化設計**：架構清晰、易於擴充，適合研究與開發
- **對開發者友善**：程式碼易讀，即使不是 SLAM 專家也能上手

### 主要元件

- **視覺前端**：雙目相機影像處理與 ORB 特徵擷取
- **迴路閉合偵測**：以 DBoW3 為基礎的場景辨識，並可自動建立詞彙庫
- **共視圖**：追蹤關鍵影格之間的關係與共享的地圖點
- **關鍵影格管理**：為建圖智慧挑選關鍵影格
- **位姿最佳化**：使用 g2o 進行光束法平差與位姿圖最佳化
- **座標轉換整合**：與 ROS 2 tf2 系統無縫整合

## 功能特色

✅ **即時雙目視覺 SLAM**
✅ **以 DBoW3 為基礎的迴路閉合偵測**：自動場景辨識與全域最佳化
✅ **自動建立詞彙庫**：不需預先訓練的詞彙庫，直接從您的環境建立
✅ **共視圖追蹤**：高效管理關鍵影格之間的關係
✅ **開箱即用支援 ROS 2 Humble**
✅ **不依賴 `zed-ros2-wrapper`**
✅ **採用 ZED SDK 與 CUDA：廣受採用、成熟的業界標準函式庫**
✅ **跨平台支援 x86 與 ARM 機器：輕鬆部署於 NVIDIA 開發套件**
✅ **相依套件精簡，便於維護並保持程式碼清晰**
✅ **以 g2o 實作進階的圖最佳化位姿估測**
✅ **以 Sophus 實作基於李代數的座標轉換**
✅ **模組化且可擴充的架構**
✅ **整合 TF2 座標轉換：與機器人無縫整合**

### 🚧 **規劃中的功能**
🔄 強化跨平台相容性
🔄 通用化的視覺里程計前端（支援更多相機類型）
🔄 IMU 整合
🔄 稠密建圖功能
🔄 外掛式架構
🔄 效能評測工具
🔄 即時調校與視覺化
🔄 完整的自動化 CI/CD 流程
🔄 容器化支援

## 相依套件

Orbis SLAM 需要以下幾項關鍵相依套件以發揮最佳效能：

### 核心函式庫

| 函式庫 | 版本 | 用途 | 安裝方式 |
|---------|---------|---------|--------------|
| **Ceres Solver** | 最新版 | 非線性最佳化 | 從原始碼建置（已作為子模組納入） |
| **Sophus** | 最新版 | 李代數運算 | 從原始碼建置（已作為子模組納入） |
| **g2o** | 最新版 | 圖最佳化 | 從原始碼建置（已作為子模組納入） |
| **DBoW3** | 最新版 | 迴路閉合偵測 | 系統安裝或從原始碼建置 |
| **Eigen3** | 3.3+ | 線性代數 | `sudo apt install libeigen3-dev` |
| **TBB** | 最新版 | 平行運算 | `sudo apt install libtbb-dev` |
| **FMT** | 最新版 | 字串格式化 | `sudo apt install libfmt-dev` |
| **OpenCV** | 4.x+ | 電腦視覺與 ORB 特徵 | 隨 ROS 2 附帶，或 `sudo apt install libopencv-dev` |

### ROS 2 相依套件

| 套件 | 用途 |
|---------|---------|
| `rclcpp` | ROS 2 C++ 用戶端函式庫 |
| `sensor_msgs` | 感測器訊息型別 |
| `geometry_msgs` | 幾何訊息型別 |
| `nav_msgs` | 導航訊息型別 |
| `cv_bridge` | OpenCV 與 ROS 的橋接 |
| `image_transport` | 影像傳輸 |
| `tf2_ros` | 座標轉換函式庫 |
| `tf2_geometry_msgs` | TF2 幾何工具 |

### 資料集錄製與回放

| 函式庫 | 版本 | 用途 | 安裝方式 |
|---------|---------|---------|--------------|
| **protobuf** | 3 | 錄製與回放的資料序列化 | `sudo apt install -y protobuf-compiler libprotobuf-dev` |
| **Qt6** | 6.x | 錄製／回放工具的圖形介面 | `sudo apt install -y qt6-base-dev libqt6opengl6-dev` |
| **GLM** | 最新版 | 視覺化所需的 OpenGL 數學函式庫 | `sudo apt install -y libglm-dev` |

#### 建置 Protocol Buffers

本專案使用 Protocol Buffers 序列化 ZED 相機資料。建置 proto 定義檔：

```bash
# Navigate to the proto directory
cd /path/to/orbis_slam/proto

# Generate C++ code from proto definitions
protoc -I=. --cpp_out=. zed_recording.proto
```

**注意**：建置套件時，CMake 建置系統會自動處理此步驟。

#### 錄製 ZED 資料

使用 `ZEDRecPlay` 圖形介面工具錄製 ZED 相機資料：

```bash
# Launch the recorder/player tool
ros2 run orbis_slam ZEDRecPlay
```

**功能**：
- 即時雙目影像預覽（左、右相機）
- 錄製含深度資料的 ZED 相機影格
- 相機位姿追蹤與視覺化
- 可調整的錄製設定
- 將錄製內容儲存為 protobuf 格式（`.pb` 檔）

#### 回放錄製資料

將錄製的資料集送入 SLAM 流程進行回放：

```bash
# Run the playback node with a recording file
ros2 run orbis_slam zed_playback_node --ros-args \
  -p recording_file:=/path/to/your/recording.pb \
  -p playback_rate:=1.0 \
  -p enable_slam:=true
```

也可以使用提供的 launch 檔，更方便地設定參數：

```bash
ros2 launch orbis_slam zed_playback.launch.py
```

`zed_playback.launch.py` 這個 launch 檔可讓您方便地設定所有回放參數。可用選項請參閱下方的參數列表。

**參數**：
- `recording_file`：`.pb` 錄製檔的路徑（必填）
- `playback_rate`：回放速度倍率（預設值：1.0）
- `enable_slam`：啟用後端位姿最佳化（預設值：true）
- `world_frame`：全域參考座標系（預設值："map"）
- `odom_frame`：里程計座標系（預設值："odom"）
- `robot_baselink_frame`：機器人本體座標系（預設值："base_link"）
- `left_camera_frame`：左相機座標系（預設值："left_camera_frame"）

**發布的 Topic**：
- `~/left/image_raw`：左相機影像
- `~/right/image_raw`：右相機影像
- `~/depth/image_raw`：深度影像
- `/tf`：含 SLAM 位姿的座標轉換樹

回放節點會將錄製資料送入完整的 Orbis SLAM 流程處理，讓您可以：
- 離線測試與除錯 SLAM 演算法
- 在錄製的資料集上評估效能
- 重現並分析特定情境
- 在沒有實體相機的情況下進行開發

### 硬體需求

- **x86 或 ARM 機器**：配備支援 CUDA 之 GPU 的筆電或開發套件
- **ZED 相機**：雙目相機（ZED、ZED 2、ZED 2i、ZED X）
- **支援 CUDA 的 GPU**：強烈建議使用以獲得最佳效能
- **ZED SDK**：Stereolabs 提供的最新版本

### 系統需求

- **作業系統**：Ubuntu 20.04 以上，或 Ubuntu 22.04（建議）
- **ROS 2**：Humble Hawksbill
- **編譯器**：GCC 9+ 或 Clang 10+
- **CMake**：3.24+
- **記憶體**：建議 8GB 以上 RAM
- **儲存空間**：至少 2GB 可用空間

## 安裝

請依照以下步驟，從原始碼在您的系統上安裝 Orbis SLAM。

### 事前準備

首先，確認系統已更新至最新：

```bash
sudo apt update && sudo apt upgrade -y
```

### 步驟 1：安裝系統相依套件

#### CUDA

請參考 [CUDA 安裝指南](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/) 設定 CUDA 環境。

#### Eigen3（線性代數函式庫）
```bash
sudo apt install -y libeigen3-dev libtf2-eigen-dev
```

#### FMT（字串格式化函式庫）
```bash
sudo apt install -y libfmt-dev
```

#### Intel TBB（Threading Building Blocks）
```bash
sudo apt install -y libtbb-dev
```

### 步驟 2：建置第三方函式庫

Orbis SLAM 以子模組形式納入數個第三方函式庫。請先初始化並建置它們：

```bash
# Clone the repository if you haven't already
git clone https://github.com/Megacephalo/Orbis_SLAM.git
cd Orbis_SLAM

# Initialize submodules
git submodule update --init --recursive
```

#### 建置 Ceres Solver

Ceres 是非線性最佳化的核心，也是 Sophus 函式庫的相依套件。從內含的子模組建置：

```bash
cd third_party/ceres-solver
cmake -DCMAKE_BUILD_TYPE=Release -B build
cmake --build build -j$(nproc)
cd build && sudo make install
cd ../../..
```

#### 建置 Sophus

Sophus 提供位姿轉換所需的李代數運算：

```bash
cd third_party/Sophus
cmake -DCMAKE_BUILD_TYPE=Release -B build
cmake --build build -j$(nproc)
cd build && sudo make install
cd ../../..
```

> **記憶體有限之系統的注意事項**：若因記憶體不足而建置失敗（例如在 NVIDIA Jetson 裝置上），請改用單執行緒編譯：
> ```bash
> cmake --build build -j1
> ```

### 步驟 3：安裝 ZED SDK

#### 清除先前的安裝（如適用）

若先前已安裝過 ZED SDK，請先將其移除：

```bash
# Remove SDK files
sudo rm -rf /usr/local/zed

# Uninstall Python wrapper
pip3 uninstall pyzed

# Remove configuration data
rm -rf ~/.config/zed ~/.local/share/zed
```

#### 安裝 ZED SDK

1. 從 [Stereolabs 下載頁面](https://www.stereolabs.com/developers/release/) 下載 ZED SDK
2. 安裝 SDK：

```bash
# Install zstd if needed
sudo apt install -y zstd

# Make installer executable
chmod +x ZED_SDK_Ubuntu*.zstd.run

# Run the installer
./ZED_SDK_Ubuntu*.zstd.run
```

3. 依照安裝程式的提示操作
4. 重新開機：

```bash
sudo reboot
```

### 步驟 4：建置 Orbis SLAM

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws

# Build the package
colcon build --packages-select orbis_slam --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

### 驗證安裝

確認節點能否正常啟動，以驗證安裝結果：

```bash
ros2 run orbis_slam orbis_slam_main
```

若安裝成功，您會看到 Orbis SLAM 節點完成初始化，並顯示相機啟動狀態的相關訊息。

## 快速上手

只需幾個指令，就能讓 Orbis SLAM 開始運作：

### 1. 連接 ZED 相機

請確認 ZED 相機已透過 USB 3.0 或 USB-C 正確連接。**提示**：建議先用 ZED SDK 範例程式或 **zed-ros2-wrapper** 的 launch 檔測試相機，確認運作正常。

### 2. 啟動 SLAM 系統

```bash
# Source your ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# Launch Orbis SLAM with sensor-only mode (treats the camera as its own robot)
ros2 launch orbis_slam orbis_slam_sensor_only.py

# Or run the main node directly. See "ROS 2 Topics" section for configurable topics and parameters
ros2 run orbis_slam orbis_slam_main
```

### 3. 確認運作狀態

檢查系統是否正在發布座標轉換並處理影格：

```bash
# GUI to visualize TF tree and node graph
rqt

# Monitor TF tree
ros2 run tf2_tools view_frames

# Check published topics
ros2 topic list

# Monitor pose output
ros2 topic echo /orbis_slam/pose
```

## 使用方式

### 基本運作

Orbis SLAM 以 ROS 2 節點的形式運作，處理雙目相機資料並發布位姿估測與座標轉換。系統會自動：

1. **初始化** ZED 相機與校正參數
2. **處理** 輸入的雙目影格
3. **挑選** 用於建圖的關鍵影格
4. **最佳化** 位姿（使用光束法平差）
5. **發布** 座標轉換至 ROS 2 TF 樹

### Launch 檔

#### 僅感測器模式
```bash
ros2 launch orbis_slam orbis_slam_sensor_only.py
```

此 launch 檔以僅感測器模式啟動 Orbis SLAM，方便只用相機進行測試。它同時也是一個範例，示範 ROS 2 節點的介面，以及如何將其整合至您的移動平台。

### ROS 2 Topic

#### 訂閱的 Topic
- `/zed/zed_node/left/image_rect_color`：左相機影像
- `/zed/zed_node/right/image_rect_color`：右相機影像
- `/zed/zed_node/depth/depth_registered`：深度影像

#### 發布的 Topic
- `/orbis_slam/pose`：目前的位姿估測
- `/orbis_slam/trajectory`：完整軌跡
- `/tf`：座標轉換樹更新

### 參數

主要參數可透過 ROS 2 參數進行設定：

```yaml
orbis_slam:
  ros__parameters:
    world_frame: "map"
    odom_frame: "odom"
    robot_baselink_frame: "base_link"
    left_camera_frame: "zed_left_camera_frame"
    enable_slam: true
```

- **enable_slam**：啟用時，節點會對挑選出的關鍵影格執行全域位姿最佳化，以最佳化相機在世界座標系中的位姿。停用時，相機的全域位姿即等同於里程計估測的位姿。

## 組態設定

### 相機校正

Orbis SLAM 會自動使用 ZED 相機內建的校正參數。若使用自訂的雙目配置，請修改組態檔中的相機參數。

### 座標系設定

本系統採用標準的 ROS 2 座標系慣例：

- `map`：全域參考座標系
- `odom`：里程計座標系
- `base_link`：機器人本體座標系
- `zed_left_camera_frame`：左相機座標系

### 效能調校

為獲得最佳效能：

1. **GPU 記憶體**：確保有足夠的 GPU 記憶體（建議 4GB 以上）
2. **CPU 核心**：多核心系統在平行最佳化時表現更好
3. **相機 FPS**：較高的影格率可提升追蹤精準度，但也會增加運算負擔

## 系統架構

### 系統概觀

```
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│   ZED Camera    │────│  Visual Frontend │────│ Keyframe Selector│
└─────────────────┘    └──────────────────┘    └──────────────────┘
                                │                        │
                                ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  TF2 Publisher  │◄───│ Pose Optimization│◄───│   Map Manager    │
└─────────────────┘    └──────────────────┘    └──────────────────┘
```

### 核心元件

#### 1. **OrbisSLAMPipeline**（`orbis_slam_pipeline.h`）
- 主要處理流程
- 協調所有子系統
- 管理 ROS 2 節點生命週期

#### 2. **ZEDWrapper**（`zed_wrapper.h`）
- 與 ZED SDK 介接
- 負責相機初始化與影格擷取
- 提供校正資料

#### 3. **KeyFrameSelector**（`keyframe_selector.h`）
- 實作關鍵影格選擇策略
- 管理影格間的追蹤
- 決定何時建立新的關鍵影格

#### 4. **PoseOptimizer**（`pose_optimizer.h`）
- 使用 Ceres Solver 進行光束法平差
- 使用 g2o 實作位姿圖最佳化
- 處理局部與全域最佳化

#### 5. **核心資料結構**（`essential_data_structure.h`）
- 定義核心資料型別（Frame、Trajectory 等）
- 提供執行緒安全且高效的位姿儲存與存取（即軌跡）

## 開發者文件

程式碼參考文件以 Doxygen 產生於 `documentation/doxygen/html`。

```bash
# From the repository root
doxygen documentation/Doxyfile
```

開啟以下檔案即可瀏覽產生的文件：

```text
documentation/doxygen/html/index.html
```

若偏好使用本機 HTTP 伺服器：

```bash
python3 -m http.server 8000 --directory documentation/doxygen/html
```

接著前往 `http://localhost:8000`。

## 發展藍圖

### 目前功能（v0.1.0）
- ✅ 即時雙目視覺 SLAM
- ✅ ZED 相機整合
- ✅ 整合 IMU 實現 VIO
- ✅ 支援 ROS 2 Humble
- ✅ TF2 整合
- ✅ 以 g2o 進行位姿最佳化

### 即將推出（v0.2.0）
- 🔄 現代化的迴路閉合偵測
- 🔄 稠密建圖功能
- 🔄 即時視覺化工具
- 🔄 流暢的安裝與設定體驗

### 未來發展
- 📋 效能評測套件
- 📋 Docker 容器化
- 📋 與 Nav2 整合

### 效能目標
- 在現代硬體上以 30 FPS 即時運作
- 一般室內環境下記憶體用量低於 2GB
- 在標準資料集與即時串流上，誤差在軌跡長度的 1% 以內

## 參與貢獻

歡迎為 Orbis SLAM 貢獻心力！您可以這樣參與：

### 開始參與

1. **Fork** 本儲存庫
2. **Clone** 您的 fork 到本機
3. **建立** 功能分支
4. **進行** 修改
5. **充分測試**
6. **提交** Pull Request

### 開發準則

- 遵循現有的程式碼風格與慣例
- 為新功能加上單元測試
- API 變更時同步更新文件
- 提交前確認所有測試皆通過

### 程式碼風格

- 使用現代 C++17 特性
- 遵循 Google C++ Style Guide
- 使用具意義的變數與函式名稱
- 為複雜演算法撰寫完整註解

### 回報問題

請至 [GitHub Issues](https://github.com/Megacephalo/Orbis_SLAM/issues) 頁面回報錯誤或提出功能需求，並附上：

- 問題的詳細描述
- 重現步驟
- 系統規格
- ROS 2 與相機的配置細節

## 授權條款

本專案採用 MIT 授權條款，詳情請參閱 [LICENSE](LICENSE) 檔案。

### 第三方函式庫

- **Ceres Solver**：[New BSD License](https://ceres-solver.org/license.html)
- **Sophus**：[MIT License](https://github.com/strasdat/Sophus/blob/master/LICENSE.txt)
- **g2o**：[BSD License](https://github.com/RainerKuemmerle/g2o/blob/master/LICENSE)
- **ZED SDK**：[Stereolabs License](https://www.stereolabs.com/legal/license/)

---

<div align="center">

**用 ❤️ 為機器人社群打造**

[回報錯誤](https://github.com/Megacephalo/Orbis_SLAM/issues) · [提出功能需求](https://github.com/Megacephalo/Orbis_SLAM/issues) · [參與貢獻](https://github.com/Megacephalo/Orbis_SLAM/pulls)

</div>
