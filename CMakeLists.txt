cmake_policy(SET CMP0048 NEW)

if (ORIN)
    MESSAGE("Compile for ORIN ... ")

    execute_process(
        COMMAND uname -m
        OUTPUT_VARIABLE RAW_ARCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    string(TOLOWER "${RAW_ARCH}" DETECTED_ARCH)
    message(STATUS "[System] uname -m = ${DETECTED_ARCH}")

    # 统一判断逻辑
    if(DETECTED_ARCH MATCHES "x86_64|i[3456]86|amd64")
        SET(CMAKE_CXX_COMPILER "/usr/bin/aarch64-linux-gnu-g++-12")
        SET(CMAKE_C_COMPILER "/usr/bin/aarch64-linux-gnu-gcc-12")
    endif()

    set(CMAKE_SYSTEM_NAME Linux)
    set(CMAKE_SYSTEM_PROCESSOR aarch64)
    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

else()
    SET(CMAKE_CXX_STANDARD 17)
endif()

project(
  booster_robotics_sdk_ros2
  VERSION 1.0.0
  LANGUAGES CXX)

cmake_minimum_required(VERSION 3.15...3.27)

SET(LIB_BOOSTER_ROBOTICS_SDK_ROS2 booster_robotics_sdk_ros2 CACHE STRING "Booster Robotics SDK ROS2")


if(ORIN)
    set(PLATFORM_SUFFIX "_orin")
    message(STATUS "Architecture is: ORIN")

    if(NOT INSTALL_DIR)
        set(INSTALL_DIR /opt/robosys CACHE PATH "rs_sdk install dir")
    endif()

    set(LIB_INSTALL_PREFIX ${INSTALL_DIR}/${LIB_BOOSTER_ROBOTICS_SDK_ROS2}${PLATFORM_SUFFIX})
else()
    set(LIB_INSTALL_PREFIX ${CMAKE_INSTALL_LIBDIR})
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(booster_interface REQUIRED)
find_package(booster_msgs)
find_package(rosidl_default_generators REQUIRED)

SET(BOOSTER_SDK_INCLUDE_DIR ${CMAKE_SOURCE_DIR}../booster_robotics_sdk/include)
include_directories(BEFORE $(PROJECT_SOURCE_DIR)/include/robot/ai)

file(GLOB_RECURSE BOOSTER_ROBOTICS_SDK_ROS2_SOURCES
    "src/*.cpp"
    "src/*.cxx"
    "src/*.ipp")

file(GLOB_RECURSE BOOSTER_ROBOTICS_SDK_ROS2_HEADERS 
    "include/*.h"
    "include/*.hpp")

add_library(${LIB_BOOSTER_ROBOTICS_SDK_ROS2} ${BOOSTER_ROBOTICS_SDK_ROS2_SOURCES})

ament_target_dependencies(booster_robotics_sdk_ros2
  rclcpp
  std_msgs
  booster_interface)

link_libraries(booster_robotics_sdk_ros2)
