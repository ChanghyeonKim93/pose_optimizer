#ifndef POSE_OPTIMIZER_LOSS_FUNCTION_H_
#define POSE_OPTIMIZER_LOSS_FUNCTION_H_

namespace pose_optimizer {

class LossFunction {
 public:
  LossFunction() {}

  virtual ~LossFunction() {}

  virtual void Evaluate(const double& squared_residual, double output[3]) = 0;
};

}  // namespace pose_optimizer

#endif  // POSE_OPTIMIZER_LOSS_FUNCTION_H_
