# macOS
cmake_minimum_required(VERSION 3.10)
project(QRScanner)

set(CMAKE_CXX_STANDARD 14)

# Set OpenCV_DIR to the correct path
set(OpenCV_DIR "/opt/homebrew/Cellar/opencv/4.9.0_12.reinstall/lib/cmake/opencv4")

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

# Print some information
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "ZBar library status:")
message(STATUS "    include path: ${ZBAR_INCLUDE_DIR}")
message(STATUS "    library: ${ZBAR_LIBRARY}")