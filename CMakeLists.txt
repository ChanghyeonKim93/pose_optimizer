cmake_minimum_required(VERSION 3.10)
project(pose_optimizer)

message("-- [PROJECT]: " ${CMAKE_PROJECT_NAME})

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  message("-- Detected architecture: x86_64")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -msse -mavx -mavx2")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("-- Detected architecture: ARM64 (aarch64)")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -ftree-vectorize -march=armv8-a+simd -mtune=cortex-a72")
else()
  message("-- Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(
  ${PROJECT_NAME} SHARED 
  pose_optimizer/time_monitor.cc
  pose_optimizer/summary.cc
  pose_optimizer/single_pose_optimizer/single_pose_optimizer.cc
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
         ${EIGEN3_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME} PUBLIC ${EIGEN3_DEFINITIONS})

install(
  TARGETS ${PROJECT_NAME} 
  DESTINATION lib
)

install(
  DIRECTORY ./pose_optimizer/
  DESTINATION include/pose_optimizer
  FILES_MATCHING
  PATTERN "*.h"
)
