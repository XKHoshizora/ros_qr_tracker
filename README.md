# ROS QR Tracker

ROS QR Tracker 是一个将 QR 码检测和跟踪功能集成到 ROS（机器人操作系统）环境中的项目。它使用 OpenCV 进行图像处理，在 Ubuntu 和 macOS 上使用 ZBar，在 Windows 上使用 quirc 进行 QR 码检测，使机器人能够实时识别和跟踪 QR 码。

## 目录

1. [前提条件](#前提条件)
2. [项目结构](#项目结构)
3. [安装](#安装)
4. [源代码差异](#源代码差异)
5. [构建项目](#构建项目)
6. [运行应用](#运行应用)
7. [ROS 集成](#ros-集成)
8. [故障排除](#故障排除)

## 前提条件

- ROS（Ubuntu 20.04 使用 Noetic，Ubuntu 18.04 使用 Melodic）
- CMake（3.10 版本或更高）
- 支持 C++14 的 C++ 编译器
- OpenCV
- ZBar（Ubuntu 和 macOS）或 quirc（Windows）

## 项目结构

```
ros_qr_tracker/
├── include/
│   └── QRScanner.h
├── src/
│   ├── QRScanner.cpp
│   └── main.cpp
├── CMakeLists.txt
└── package.xml
```

## 安装

### Ubuntu 和 macOS

1. 安装 ROS（按照官方 ROS 安装指南进行安装）
2. 安装依赖：
   ```
   sudo apt update
   sudo apt install cmake libopencv-dev libzbar-dev ros-<distro>-cv-bridge
   ```
   将 `<distro>` 替换为您的 ROS 发行版（例如 noetic, melodic）

### Windows

1. 安装 Visual Studio（2019 或更新版本），选择"使用 C++ 的桌面开发"工作负载。
2. 从 [cmake.org](https://cmake.org/download/) 下载并安装 CMake。
3. 从 [opencv.org](https://opencv.org/releases/) 下载 Windows 版 OpenCV 并解压。
4. 下载并编译 quirc 库。

## 源代码差异

项目的核心代码（QRScanner.h 和 QRScanner.cpp）在三个操作系统上有一些关键的差异：

### 头文件 (QRScanner.h)

Ubuntu 和 macOS:

```cpp
#include <opencv2/opencv.hpp>
#include <zbar.h>

class QRScanner {
    // ...
private:
    cv::VideoCapture cap;
    zbar::ImageScanner scanner;
    // ...
};
```

Windows:

```cpp
#include <opencv2/opencv.hpp>
#include <quirc.h>

class QRScanner {
    // ...
private:
    cv::VideoCapture cap;
    struct quirc* qr;
    // ...
};
```

### 实现文件 (QRScanner.cpp)

主要差异在于 QR 码检测和解码的实现：

Ubuntu 和 macOS:

```cpp
void QRScanner::processFrame(cv::Mat& frame) {
    // 使用 ZBar 进行 QR 码检测
    zbar::Image image(frame.cols, frame.rows, "Y800", grey.data, frame.cols * frame.rows);
    scanner.scan(image);

    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
        // 处理检测到的 QR 码
        // ...
    }
}
```

Windows:

```cpp
void QRScanner::processFrame(cv::Mat& frame) {
    // 使用 quirc 进行 QR 码检测
    quirc_resize(qr, width, height);
    uint8_t* image = quirc_begin(qr, nullptr, nullptr);
    memcpy(image, grey.data, width * height);
    quirc_end(qr);

    int count = quirc_count(qr);
    for (int i = 0; i < count; i++) {
        struct quirc_code code;
        struct quirc_data data;
        quirc_extract(qr, i, &code);
        if (quirc_decode(&code, &data) == 0) {
            // 处理检测到的 QR 码
            // ...
        }
    }
}
```

## 构建项目

### Ubuntu 和 macOS

1. 创建一个 catkin 工作空间：
   ```
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   git clone <repository-url> ros_qr_tracker
   cd ~/catkin_ws
   catkin_make
   ```

### Windows

1. 使用 CMake 生成 Visual Studio 项目：
   ```
   mkdir build && cd build
   cmake .. -G "Visual Studio 16 2019" -A x64
   ```
2. 打开生成的 .sln 文件，在 Visual Studio 中构建解决方案。

## 运行应用

### Ubuntu 和 macOS（带 ROS）

1. 源化您的工作空间：
   ```
   source ~/catkin_ws/devel/setup.bash
   ```
2. 运行节点：
   ```
   rosrun ros_qr_tracker ros_qr_tracker_node
   ```

### macOS（不带 ROS）和 Windows

直接运行编译生成的可执行文件。

## ROS 集成

本项目在 Ubuntu 和支持 ROS 的 macOS 环境下设计为 ROS 节点。它发布检测到的 QR 码信息，并订阅相机图像话题。

主要 ROS 特性：

- 发布到 `/qr_tracker/detections` 话题
- 订阅 `/camera/image_raw` 话题

Windows 版本通常不包含 ROS 集成。

## 故障排除

- 库加载问题：检查路径设置和库文件位置。
- Windows DLL 问题：确保所需 DLL 在系统 PATH 中或与可执行文件在同一目录。
- ROS 问题：检查 ROS 核心是否运行，以及话题连接是否正确。

## 贡献

欢迎贡献！请通过创建 issue 或提交 pull request 来帮助改进这个项目。

## 许可

本项目采用 MIT 许可证。详情请见 LICENSE 文件。
