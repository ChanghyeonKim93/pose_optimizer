#ifndef POSE_OPTIMIZER_SINGLE_POSE_OPTIMIZER_PROBLEM_H_
#define POSE_OPTIMIZER_SINGLE_POSE_OPTIMIZER_PROBLEM_H_

#include "pose_optimizer/single_pose_optimizer/cost_function.h"
#include "pose_optimizer/single_pose_optimizer/loss_function.h"
#include "pose_optimizer/single_pose_optimizer/residual_block.h"

namespace pose_optimizer {
namespace single_pose_optimizer {

template <int kDimTranslation, int kDimRotation>
class Problem {
 protected:
  static constexpr int kDimPose = kDimTranslation + kDimRotation;
  using CostFunctionPtr =
      std::shared_ptr<CostFunction<kDimTranslation, kDimRotation>>;
  using LossFunctionPtr = std::shared_ptr<LossFunction>;
  using ResidualBlockPtr =
      std::shared_ptr<ResidualBlock<kDimTranslation, kDimRotation>>;

 public:
  Problem() {}

  void AddResidualBlock(CostFunctionPtr* cost_function,
                        LossFunctionPtr* loss_function = nullptr) {
    ResidualBlockPtr residual_block =
        std::make_shared<ResidualBlock>(cost_function, loss_function);
    residual_block_set_.insert(residual_block);
  }

  const std::unordered_set<ResidualBlockPtr>& GetResidualBlocks() const {
    return residual_block_set_;
  }

 private:
  std::unordered_set<ResidualBlockPtr> residual_block_set_;
};

}  // namespace single_pose_optimizer
}  // namespace pose_optimizer

#endif  // POSE_OPTIMIZER_SINGLE_POSE_OPTIMIZER_PROBLEM_H_