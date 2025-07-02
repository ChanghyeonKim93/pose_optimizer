#ifndef NONLINEAR_OPTIMIZER_POSE_OPTIMIZER_COST_FUNCTION_H_
#define NONLINEAR_OPTIMIZER_POSE_OPTIMIZER_COST_FUNCTION_H_

#include "Eigen/Dense"

namespace pose_optimizer {
namespace single_pose_optimizer {

template <int kDimPose>
class CostFunction {
 protected:
  static const int kDimTranslation = kDimPose;
  static const int kDimRotation = kDimPose;
  static const int kDimPoseParam = kDimPose == 2 ? 3 : 6;
  using RotationMatrix = Eigen::Matrix<double, kDimRotation, kDimRotation>;
  using TranslationVector = Eigen::Matrix<double, kDimTranslation, 1>;
  using HessianMatrix = Eigen::Matrix<double, kDimPoseParam, kDimPoseParam>;
  using GradientVector = Eigen::Matrix<double, kDimPoseParam, 1>;

 public:
  CostFunction() {}

  virtual ~CostFunction() {}

  virtual bool Evaluate(const RotationMatrix& rotation_matrix,
                        const TranslationVector& translation,
                        double* jacobian_matrix_ptr,
                        double* residual_vector_ptr) = 0;

  int GetDimResidual() const { return dim_residual_; }

  int GetDimPose() const { return kDimPose; }

  int GetDimPoseParameter() const { return kDimPoseParam; }

  int GetDimTranslation() const { return kDimTranslation; }

  int GetDimRotation() const { return kDimRotation; }

 protected:
  void SetDimResidual(const int dim_residual) { dim_residual_ = dim_residual; }

 private:
  int dim_residual_{-1};
};

template <int kDimResidual, int kDimPose>
class SizedCostFunction : public CostFunction<kDimPose> {
 public:
  SizedCostFunction() { this->SetDimResidual(kDimResidual); }

  virtual ~SizedCostFunction() {}
};

}  // namespace single_pose_optimizer
}  // namespace pose_optimizer

#endif  // NONLINEAR_OPTIMIZER_POSE_OPTIMIZER_COST_FUNCTION_H_