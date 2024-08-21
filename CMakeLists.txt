cmake_minimum_required(VERSION 3.10)
project(QRScanner VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# OpenCV
if(NOT DEFINED OpenCV_DIR)
    if(APPLE)
        set(OpenCV_DIR "/opt/homebrew/Cellar/opencv/4.10.0_3/lib/cmake/opencv4" CACHE PATH "Path to OpenCVConfig.cmake")
    elseif(WIN32)
        set(OpenCV_DIR "C:/opencv/build" CACHE PATH "Path to OpenCVConfig.cmake")
    endif()
endif()

find_package(OpenCV REQUIRED)

if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found. Please install OpenCV or set OpenCV_DIR.")
endif()

# ZBar
if(UNIX AND NOT APPLE)
    # Ubuntu: Use pkg-config
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ZBAR REQUIRED IMPORTED_TARGET zbar)

    if(NOT ZBAR_FOUND)
        message(FATAL_ERROR "ZBar not found. Please install ZBar.")
    endif()
else()
    # macOS and Windows: Manual find
    find_path(ZBAR_INCLUDE_DIR NAMES zbar.h
        PATHS
        /opt/homebrew/include
        /usr/local/include
        /usr/include
        C:/ZBar/include
    )
    find_library(ZBAR_LIBRARY NAMES zbar
        PATHS
        /opt/homebrew/lib
        /usr/local/lib
        /usr/lib
        C:/ZBar/lib
    )

    if(NOT ZBAR_INCLUDE_DIR OR NOT ZBAR_LIBRARY)
        message(FATAL_ERROR "ZBar not found. Please install ZBar or set ZBAR_INCLUDE_DIR and ZBAR_LIBRARY manually.")
    endif()
endif()

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)

if(UNIX AND NOT APPLE)
    include_directories(${ZBAR_INCLUDE_DIRS})
else()
    include_directories(${ZBAR_INCLUDE_DIR})
endif()

# Add executable
add_executable(${PROJECT_NAME} src/main.cpp src/QRScanner.cpp)

# Link libraries
if(UNIX AND NOT APPLE)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} PkgConfig::ZBAR)
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} ${ZBAR_LIBRARY})
endif()

# Set output directory
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# Install rules
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

# Print information
include(CMakePrintHelpers)
cmake_print_variables(CMAKE_SYSTEM_NAME CMAKE_CXX_STANDARD)
cmake_print_properties(
    TARGETS ${PROJECT_NAME}
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY
)
cmake_print_variables(OpenCV_VERSION OpenCV_LIBS)

if(UNIX AND NOT APPLE)
    cmake_print_variables(ZBAR_VERSION ZBAR_LIBRARIES)
else()
    cmake_print_variables(ZBAR_INCLUDE_DIR ZBAR_LIBRARY)
endif()