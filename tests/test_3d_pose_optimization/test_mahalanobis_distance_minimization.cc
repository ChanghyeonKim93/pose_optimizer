#include <iostream>

#include "pose_optimizer/single_pose_optimizer/cost_function.h"
#include "pose_optimizer/single_pose_optimizer/single_pose_optimizer.h"

using namespace pose_optimizer::single_pose_optimizer;

struct Correspondence {
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d sqrt_information{Eigen::Matrix3d::Zero()};
  Eigen::Vector3d mean{Eigen::Vector3d::Zero()};
  Eigen::Vector3d plane_normal_vector{Eigen::Vector3d::Zero()};
  bool is_planar{false};
};

class MahalanobisDistanceCostFunctor : SizedCostFunction<3, 3> {
 public:
  MahalanobisDistanceCostFunctor(const Correspondence& correspondence)
      : correspondence_(correspondence) {}

  bool Evaluate(const RotationMatrix& R, const TranslationVector& t,
                double* jacobian_matrix_ptr,
                double* residual_vector_ptr) final {
    if (jacobian_matrix_ptr == nullptr || residual_vector_ptr == nullptr)
      return false;

    Eigen::Map<Eigen::Matrix<double, 3, 6>> jacobian(jacobian_matrix_ptr);
    Eigen::Map<Eigen::Matrix<double, 3, 1>> residual(residual_vector_ptr);
    jacobian.setZero();
    residual.setZero();

    const auto& p = correspondence_.point;
    const auto& mean = correspondence_.mean;
    const auto& sqrt_information = correspondence_.sqrt_information;

    const Eigen::Matrix<double, 3, 1> warped_p = R * p + t;
    const Eigen::Matrix<double, 3, 1> e = warped_p - mean;
    residual = sqrt_information * e;

    // == Mat3x3 R_skew_p = R * skew(p);
    Eigen::Matrix3d R_skew_p{Eigen::Matrix3d::Zero()};
    R_skew_p(0, 0) = R(0, 1) * p(2) - R(0, 2) * p(1);
    R_skew_p(0, 1) = R(0, 2) * p(0) - R(0, 0) * p(2);
    R_skew_p(0, 2) = R(0, 0) * p(1) - R(0, 1) * p(0);
    R_skew_p(1, 0) = R(1, 1) * p(2) - R(1, 2) * p(1);
    R_skew_p(1, 1) = R(1, 2) * p(0) - R(1, 0) * p(2);
    R_skew_p(1, 2) = R(1, 0) * p(1) - R(1, 1) * p(0);
    R_skew_p(2, 0) = R(2, 1) * p(2) - R(2, 2) * p(1);
    R_skew_p(2, 1) = R(2, 2) * p(0) - R(2, 0) * p(2);
    R_skew_p(2, 2) = R(2, 0) * p(1) - R(2, 1) * p(0);
    jacobian.block<3, 3>(0, 0) = sqrt_information;
    jacobian.block<3, 3>(0, 3) = -sqrt_information * R_skew_p;

    return true;
  }

 private:
  const Correspondence correspondence_;
};

int main(int argc, char** argv) {
  std::unique_ptr<pose_optimizer::single_pose_optimizer::SinglePoseOptimizer<3>>
      optimizer = std::make_unique<
          pose_optimizer::single_pose_optimizer::SinglePoseOptimizer<3>>();

  pose_optimizer::single_pose_optimizer::Problem<3> problem;
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  pose_optimizer::Options options;
  pose_optimizer::Summary summary;
  optimizer->Solve(problem, options, &pose, &summary);

  std::cerr << summary.GetBriefReport() << std::endl;

  return 0;
}