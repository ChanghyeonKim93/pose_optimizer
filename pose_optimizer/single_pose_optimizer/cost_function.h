#ifndef NONLINEAR_OPTIMIZER_POSE_OPTIMIZER_COST_FUNCTION_H_
#define NONLINEAR_OPTIMIZER_POSE_OPTIMIZER_COST_FUNCTION_H_

#include "Eigen/Dense"

namespace pose_optimizer {
namespace single_pose_optimizer {

template <int kDimTranslation, int kDimRotation>
class CostFunction {
 protected:
  static constexpr int kDimPose = kDimTranslation + kDimRotation;
  using RotationMatrix = Eigen::Matrix<double, kDimRotation, kDimRotation>;
  using TranslationVector = Eigen::Matrix<double, kDimTranslation, 1>;
  using HessianMatrix = Eigen::Matrix<double, kDimPose, kDimPose>;
  using GradientVector = Eigen::Matrix<double, kDimPose, 1>;

 public:
  virtual ~CostFunction() {}

  virtual bool Evaluate(const RotationMatrix& rotation_matrix,
                        const TranslationVector& translation,
                        double* jacobian_matrix_ptr,
                        double* residual_vector_ptr) = 0;

  int GetDimResidual() const { return dim_residual_; }

  int GetDimPose() const { return kDimPose; }

  int GetDimTranslation() const { return kDimTranslation; }

  int GetDimRotation() const { return kDimRotation; }

 protected:
  void SetDimResidual(const int dim_residual) { dim_residual_ = dim_residual; }

 private:
  int dim_residual_{-1};
};

template <int kDimResidual, int kDimTranslation, int kDimRotation>
class SizedCostFunction : public CostFunction<kDimTranslation, kDimRotation> {
 public:
  SizedCostFunction() { this->SetDimResidual(kDimResidual); }

  virtual ~SizedCostFunction() {}
};

}  // namespace single_pose_optimizer
}  // namespace pose_optimizer

#endif  // NONLINEAR_OPTIMIZER_POSE_OPTIMIZER_COST_FUNCTION_H_