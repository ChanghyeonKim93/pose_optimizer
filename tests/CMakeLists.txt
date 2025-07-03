cmake_minimum_required(VERSION 3.10)
project(pose_optimizer_tests)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

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

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(
  ${PROJECT_NAME}_test_reprojection_error_minimization 
  test_3d_pose_optimization/test_reprojection_error_minimization.cc
)
target_link_libraries(
  ${PROJECT_NAME}_test_reprojection_error_minimization 
  pose_optimizer
)

add_executable(
  ${PROJECT_NAME}_test_variadic_cost 
  test_3d_pose_optimization/test_variadic_cost.cc
)