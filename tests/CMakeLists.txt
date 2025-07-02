add_executable(
  ${PROJECT_NAME}_test_mahalanobis_distance_minimization 
  test_3d_pose_optimization/test_mahalanobis_distance_minimization.cc
)
target_link_libraries(
  ${PROJECT_NAME}_test_mahalanobis_distance_minimization 
  pose_optimizer
)