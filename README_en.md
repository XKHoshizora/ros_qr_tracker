# ROS QR Tracker

ROS QR Tracker is a project that integrates QR code detection and tracking capabilities into a ROS (Robot Operating System) environment. It uses OpenCV for image processing, ZBar on Ubuntu and macOS, and quirc on Windows for QR code detection, allowing robots to identify and track QR codes in real-time.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Project Structure](#project-structure)
3. [Installation](#installation)
4. [Source Code Differences](#source-code-differences)
5. [Building the Project](#building-the-project)
6. [Running the Application](#running-the-application)
7. [ROS Integration](#ros-integration)
8. [Troubleshooting](#troubleshooting)

## Prerequisites

- ROS (Noetic for Ubuntu 20.04, Melodic for Ubuntu 18.04)
- CMake (version 3.10 or higher)
- C++ compiler with C++14 support
- OpenCV
- ZBar (Ubuntu and macOS) or quirc (Windows)

## Project Structure

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

## Installation

### Ubuntu and macOS

1. Install ROS (follow the official ROS installation guide)
2. Install dependencies:
   ```
   sudo apt update
   sudo apt install cmake libopencv-dev libzbar-dev ros-<distro>-cv-bridge
   ```
   Replace `<distro>` with your ROS distribution (e.g., noetic, melodic)

### Windows

1. Install Visual Studio (2019 or later) with "Desktop development with C++" workload.
2. Download and install CMake from [cmake.org](https://cmake.org/download/).
3. Download OpenCV for Windows from [opencv.org](https://opencv.org/releases/) and extract it.
4. Download and compile the quirc library.

## Source Code Differences

The core code (QRScanner.h and QRScanner.cpp) has key differences across the three operating systems:

### Header File (QRScanner.h)

Ubuntu and macOS:

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

### Implementation File (QRScanner.cpp)

The main difference is in the QR code detection and decoding implementation:

Ubuntu and macOS:

```cpp
void QRScanner::processFrame(cv::Mat& frame) {
    // Using ZBar for QR code detection
    zbar::Image image(frame.cols, frame.rows, "Y800", grey.data, frame.cols * frame.rows);
    scanner.scan(image);

    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
        // Process detected QR code
        // ...
    }
}
```

Windows:

```cpp
void QRScanner::processFrame(cv::Mat& frame) {
    // Using quirc for QR code detection
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
            // Process detected QR code
            // ...
        }
    }
}
```

## Building the Project

### Ubuntu and macOS

1. Create a catkin workspace:
   ```
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   git clone <repository-url> ros_qr_tracker
   cd ~/catkin_ws
   catkin_make
   ```

### Windows

1. Generate Visual Studio project using CMake:
   ```
   mkdir build && cd build
   cmake .. -G "Visual Studio 16 2019" -A x64
   ```
2. Open the generated .sln file and build the solution in Visual Studio.

## Running the Application

### Ubuntu and macOS (with ROS)

1. Source your workspace:
   ```
   source ~/catkin_ws/devel/setup.bash
   ```
2. Run the node:
   ```
   rosrun ros_qr_tracker ros_qr_tracker_node
   ```

### macOS (without ROS) and Windows

Run the compiled executable directly.

## ROS Integration

This project is designed as a ROS node on Ubuntu and ROS-supported macOS environments. It publishes detected QR code information and subscribes to camera image topics.

Key ROS features:

- Publishes to `/qr_tracker/detections` topic
- Subscribes to `/camera/image_raw` topic

The Windows version typically does not include ROS integration.

## Troubleshooting

- Library loading issues: Check path settings and library file locations.
- Windows DLL issues: Ensure required DLLs are in the system PATH or in the same directory as the executable.
- ROS issues: Check if the ROS core is running and if topic connections are correct.

## Contributing

Contributions are welcome! Please help improve this project by creating issues or submitting pull requests.

## License

This project is licensed under the MIT License. See the LICENSE file for details.
