cmake_minimum_required(VERSION 3.15)
project(BattlefieldSensorFusion VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags for optimization and warnings
if(MSVC)
    add_compile_options(/W4 /O2)
else()
    add_compile_options(-Wall -Wextra -O3 -march=native)
endif()

# Find required packages
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

# Source files
set(COMMON_SOURCES
    src/common/Logger.cpp
)

set(SENSOR_SOURCES
    src/sensors/SensorData.cpp
    src/sensors/SyntheticSensorGenerator.cpp
)

set(FUSION_SOURCES
    src/fusion/KalmanFilter.cpp
    src/fusion/FusionEngine.cpp
    src/fusion/EntityTracker.cpp
)

set(OUTPUT_SOURCES
    src/output/WebSocketServer.cpp
    src/output/CLIVisualizer.cpp
)

set(SYSTEM_SOURCES
    src/system/SensorFusionSystem.cpp
)

# Main executable
add_executable(sensor_fusion
    src/main.cpp
    ${COMMON_SOURCES}
    ${SENSOR_SOURCES}
    ${FUSION_SOURCES}
    ${OUTPUT_SOURCES}
    ${SYSTEM_SOURCES}
)

target_link_libraries(sensor_fusion
    Threads::Threads
    Eigen3::Eigen
    Boost::system
)

# Demo executable
add_executable(demo
    demo/demo_script.cpp
    ${COMMON_SOURCES}
    ${SENSOR_SOURCES}
    ${FUSION_SOURCES}
    ${OUTPUT_SOURCES}
    ${SYSTEM_SOURCES}
)

target_link_libraries(demo
    Threads::Threads
    Eigen3::Eigen
    Boost::system
)

# Installation
install(TARGETS sensor_fusion demo
    RUNTIME DESTINATION bin
)
