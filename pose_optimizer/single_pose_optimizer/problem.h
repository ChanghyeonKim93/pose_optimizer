#ifndef POSE_OPTIMIZER_SINGLE_POSE_OPTIMIZER_PROBLEM_H_
#define POSE_OPTIMIZER_SINGLE_POSE_OPTIMIZER_PROBLEM_H_

#include "pose_optimizer/single_pose_optimizer/cost_function.h"
#include "pose_optimizer/single_pose_optimizer/loss_function.h"
#include "pose_optimizer/single_pose_optimizer/residual_block.h"

namespace pose_optimizer {
namespace single_pose_optimizer {

template <int kDimPose>
class Problem {
 protected:
  using CostFunctionPtr = std::shared_ptr<CostFunction<kDimPose>>;
  using LossFunctionPtr = std::shared_ptr<LossFunction>;
  using ResidualBlockPtr = std::shared_ptr<ResidualBlock<kDimPose>>;

 public:
  Problem() {}

  void AddResidualBlock(const CostFunctionPtr& cost_function,
                        const LossFunctionPtr& loss_function) {
    ResidualBlockPtr residual_block =
        std::make_shared<ResidualBlock<kDimPose>>(cost_function, loss_function);
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