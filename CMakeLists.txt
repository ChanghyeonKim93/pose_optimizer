cmake_minimum_required(VERSION 3.10)
project(pose_optimizer)

message("-- [PROJECT]: " ${CMAKE_PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  message("-- Detected CPU architecture: x86_64")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mavx2")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("-- Detected CPU architecture: ARM64 (aarch64)")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Ofast -ftree-vectorize")
else()
  message("-- Unknown CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

message("  -- CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message("  -- CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")

find_package(Eigen3 REQUIRED)

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
