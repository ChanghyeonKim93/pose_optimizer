#ifndef POSE_OPTIMIZER_SINGLE_POSE_OPTIMIZER_RESIDUAL_BLOCK_H_
#define POSE_OPTIMIZER_SINGLE_POSE_OPTIMIZER_RESIDUAL_BLOCK_H_

#include "pose_optimizer/single_pose_optimizer/cost_function.h"
#include "pose_optimizer/single_pose_optimizer/loss_function.h"
#include "pose_optimizer/types.h"

#include <iostream>

namespace pose_optimizer {
namespace single_pose_optimizer {

template <int kDimPose>
class ResidualBlock {
 protected:
  static const int kDimTranslation = kDimPose;
  static const int kDimRotation = kDimPose;
  static const int kDimPoseParam = kDimPose == 2 ? 3 : 6;
  using CostFunctionPtr = std::shared_ptr<CostFunction<kDimPose>>;
  using LossFunctionPtr = std::shared_ptr<LossFunction>;
  using ResidualBlockPtr = std::shared_ptr<ResidualBlock<kDimPose>>;

  using RotationMatrix = Eigen::Matrix<double, kDimRotation, kDimRotation>;
  using TranslationVector = Eigen::Matrix<double, kDimTranslation, 1>;
  using HessianMatrix = Eigen::Matrix<double, kDimPoseParam, kDimPoseParam>;
  using GradientVector = Eigen::Matrix<double, kDimPoseParam, 1>;

 public:
  ResidualBlock(const CostFunctionPtr& cost_function,
                const LossFunctionPtr& loss_function = nullptr)
      : cost_function_(cost_function), loss_function_(loss_function) {}

  bool Evaluate(const RotationMatrix& R, const TranslationVector& t,
                HessianMatrix* local_hessian_ptr,
                GradientVector* local_gradient_ptr, double* cost) {
    static std::vector<double> jacobian(kDimPoseParam * kDimPoseParam);
    static std::vector<double> residual(kDimPoseParam * kDimPoseParam);
    if (local_hessian_ptr == nullptr || local_gradient_ptr == nullptr ||
        cost == nullptr) {
      return false;
    }

    const int dim_residual = cost_function_->GetDimResidual();
    jacobian.resize(dim_residual * kDimPoseParam, 0.0);
    residual.resize(dim_residual, 0.0);
    if (!cost_function_->Evaluate(R, t, jacobian.data(), residual.data())) {
      std::cerr << "Failed to evaluate cost function." << std::endl;
      return false;
    }

    ComputeHessianUpperTriangle(jacobian.data(), dim_residual,
                                local_hessian_ptr);
    ComputeGradient(jacobian.data(), residual.data(), dim_residual,
                    local_gradient_ptr);

    const double squared_residual =
        ComputeSquaredResidual(residual.data(), dim_residual);

    if (loss_function_.get() == nullptr) {
      *cost = 0.5 * squared_residual;
    } else {
      double loss_output[3];
      loss_function_->Evaluate(squared_residual, loss_output);
      *cost = 0.5 * squared_residual;
      const double weight = loss_output[1];
      MultiplyWeight(weight, local_hessian_ptr, local_gradient_ptr);
    }

    return true;
  }

 private:
  void ComputeHessianUpperTriangle(const double* jacobian_data_ptr,
                                   const int dim_residual,
                                   HessianMatrix* hessian) {
    hessian->setZero();
    const double* jac_col_i = jacobian_data_ptr;
    for (int i = 0; i < kDimPoseParam; ++i, jac_col_i += dim_residual) {
      const double* jac_col_j = jac_col_i;
      for (int j = i; j < kDimPoseParam; ++j, jac_col_j += dim_residual)
        for (int k = 0; k < dim_residual; ++k)
          (*hessian)(i, j) += jac_col_i[k] * jac_col_j[k];
    }
  }

  void ComputeGradient(const double* jacobian_data_ptr,
                       const double* residual_data_ptr, const int dim_residual,
                       GradientVector* gradient) {
    gradient->setZero();
    const double* jac_col_i = jacobian_data_ptr;
    for (int i = 0; i < kDimPoseParam; ++i, jac_col_i += dim_residual) {
      for (int k = 0; k < dim_residual; ++k)
        (*gradient)(i) += jac_col_i[k] * residual_data_ptr[k];
    }
  }

  void MultiplyWeight(const double weight, HessianMatrix* hessian,
                      GradientVector* gradient_vector) {
    for (int i = 0; i < kDimPoseParam; ++i) {
      (*gradient_vector)(i) *= weight;
      for (int j = i; j < kDimPoseParam; ++j) (*hessian)(i, j) *= weight;
    }
  }

  double ComputeSquaredResidual(const double* residual_data_ptr,
                                const int dim_residual) {
    double squared_residual = 0.0;
    for (int i = 0; i < dim_residual; ++i)
      squared_residual += residual_data_ptr[i] * residual_data_ptr[i];
    return squared_residual;
  }

  CostFunctionPtr cost_function_{nullptr};
  LossFunctionPtr loss_function_{nullptr};
};

extern template class ResidualBlock<3>;
extern template class ResidualBlock<2>;

}  // namespace single_pose_optimizer
}  // namespace pose_optimizer

#endif  // POSE_OPTIMIZER_SINGLE_POSE_OPTIMIZER_RESIDUAL_BLOCK_H_
