# ROS QR Tracker

> QR Scanner Implementation Guide for macOS and Ubuntu

## 1. 环境准备

### macOS:

1. 安装 Homebrew：

   ```
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   ```

2. 安装依赖：
   ```
   brew install cmake opencv zbar
   ```

### Ubuntu:

1. 更新包列表：

   ```
   sudo apt update
   ```

2. 安装依赖：
   ```
   sudo apt install cmake libopencv-dev libzbar-dev
   ```

## 2. 项目结构

创建以下项目结构（两个系统相同）：

```
project_root/
├── include/
│   └── QRScanner.h
├── src/
│   ├── QRScanner.cpp
│   └── main.cpp
└── CMakeLists.txt
```

## 3. 代码实现

代码实现部分在 macOS 和 Ubuntu 上是相同的。

### QRScanner.h

```cpp
#pragma once

#include <opencv2/opencv.hpp>
#include <zbar.h>

class QRScanner {
public:
    QRScanner();
    ~QRScanner();
    void run();

private:
    cv::VideoCapture cap;
    zbar::ImageScanner scanner;

    void processFrame(cv::Mat& frame);
    void decodeAndDraw(cv::Mat& frame, zbar::Image& image);
};
```

### QRScanner.cpp

```cpp
#include "QRScanner.h"
#include <iostream>

QRScanner::QRScanner() : cap(0) {
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
}

QRScanner::~QRScanner() {
    cap.release();
}

void QRScanner::run() {
    std::cout << "Entering QRScanner::run()" << std::endl;
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open camera" << std::endl;
        return;
    }
    std::cout << "Camera opened successfully" << std::endl;

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Unable to capture frame" << std::endl;
            break;
        }
        std::cout << "Frame captured" << std::endl;

        processFrame(frame);

        cv::imshow("Camera", frame);
        std::cout << "Frame displayed" << std::endl;

        if (cv::waitKey(1) >= 0) {
            std::cout << "Key pressed. Exiting..." << std::endl;
            break;
        }
    }
    std::cout << "Exiting QRScanner::run()" << std::endl;
}

void QRScanner::processFrame(cv::Mat& frame) {
    cv::Mat grey;
    cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);

    zbar::Image image(frame.cols, frame.rows, "Y800", grey.data, frame.cols * frame.rows);

    decodeAndDraw(frame, image);
}

void QRScanner::decodeAndDraw(cv::Mat& frame, zbar::Image& image) {
    scanner.scan(image);

    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
        std::cout << "二维码内容: " << symbol->get_data() << std::endl;

        std::vector<cv::Point> points;
        for (int i = 0; i < symbol->get_location_size(); i++) {
            points.emplace_back(symbol->get_location_x(i), symbol->get_location_y(i));
        }

        cv::polylines(frame, points, true, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, symbol->get_data(), points[0], cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(255, 0, 0), 2);
    }
}
```

### main.cpp

```cpp
#include "QRScanner.h"
#include <iostream>

int main() {
    std::cout << "Starting QR Scanner..." << std::endl;
    QRScanner scanner;
    std::cout << "QR Scanner initialized." << std::endl;
    scanner.run();
    std::cout << "QR Scanner finished." << std::endl;
    return 0;
}
```

## 4. CMake 配置

### macOS:

```cmake
cmake_minimum_required(VERSION 3.10)
project(QRScanner)

set(CMAKE_CXX_STANDARD 14)

# Set OpenCV_DIR to the correct path
set(OpenCV_DIR "/opt/homebrew/opt/opencv/lib/cmake/opencv4")

# Find OpenCV
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found. Please install OpenCV or set OpenCV_DIR.")
endif()

# Find ZBar
find_path(ZBAR_INCLUDE_DIR NAMES zbar.h
    PATHS
    /opt/homebrew/include
    /usr/local/include
    /usr/include
)
find_library(ZBAR_LIBRARY NAMES zbar
    PATHS
    /opt/homebrew/lib
    /usr/local/lib
    /usr/lib
)
if(NOT ZBAR_INCLUDE_DIR OR NOT ZBAR_LIBRARY)
    message(FATAL_ERROR "ZBar not found. Please install ZBar or set ZBAR_INCLUDE_DIR and ZBAR_LIBRARY manually.")
endif()

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} ${ZBAR_INCLUDE_DIR} include)

# Add executable
add_executable(QRScanner src/main.cpp src/QRScanner.cpp)

# Link libraries
target_link_libraries(QRScanner ${OpenCV_LIBS} ${ZBAR_LIBRARY})
```

### Ubuntu:

```cmake
cmake_minimum_required(VERSION 3.10)
project(QRScanner)

set(CMAKE_CXX_STANDARD 14)

# Find OpenCV
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found. Please install OpenCV.")
endif()

# Find ZBar
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED IMPORTED_TARGET zbar)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} include)

# Add executable
add_executable(QRScanner src/main.cpp src/QRScanner.cpp)

# Link libraries
target_link_libraries(QRScanner ${OpenCV_LIBS} PkgConfig::ZBAR)
```

## 5. 编译和运行

### macOS 和 Ubuntu:

1. 创建 build 目录：

   ```
   mkdir build && cd build
   ```

2. 运行 CMake：

   ```
   cmake ..
   ```

3. 编译项目：

   ```
   make
   ```

4. 运行程序：
   ```
   ./QRScanner
   ```

## 6. 故障排除

### macOS:

如果遇到库加载问题，请检查并更新符号链接：

```
sudo rm /opt/homebrew/opt/opencv
sudo ln -s /opt/homebrew/Cellar/opencv/[version] /opt/homebrew/opt/opencv
```

更新动态库搜索路径：

```
export DYLD_LIBRARY_PATH=/opt/homebrew/opt/opencv/lib:$DYLD_LIBRARY_PATH
```

### Ubuntu:

如果遇到库加载问题，可以尝试更新动态库缓存：

```
sudo ldconfig
```

## 7. 系统权限

### macOS:

确保在系统偏好设置中授予程序摄像头访问权限。

### Ubuntu:

通常不需要特别的权限设置。如果遇到权限问题，可以尝试将用户添加到 video 组：

```
sudo usermod -a -G video $USER
```

然后注销并重新登录以使更改生效。

## 8. 调试

对于两个系统，如果程序没有显示窗口或没有输出，添加调试语句并检查每个步骤的输出。

在 Ubuntu 上，如果遇到 "cannot connect to X server" 错误，确保你不是在远程 SSH 会话中运行程序，或者正确设置了 X11 转发。

## 结论

这个指南涵盖了在 macOS 和 Ubuntu 系统上实现 QR 码扫描器的完整过程。主要的区别在于依赖项的安装方式和 CMake 配置文件。代码实现部分在两个系统上是相同的。

在 Ubuntu 上，由于包管理系统的差异，安装过程通常更简单，而且 CMake 配置文件也略有不同，主要是在查找和链接 ZBar 库的方式上。

如果在任何一个系统上遇到问题，请仔细检查每个步骤，确保所有依赖项都正确安装和配置。如果问题持续，可以查看系统日志或添加更多的调试输出来定位问题。
