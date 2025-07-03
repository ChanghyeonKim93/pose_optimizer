#include <iostream>
#include <memory>
#include <utility>

template <typename CostFunctor,
          int kNumResiduals,  // Number of residuals, or ceres::DYNAMIC.
          int... Ns>          // Number of parameters in each parameter block.
class AutoDiffCostFunction final
    : public SizedCostFunction<kNumResiduals, Ns...> {};

// For fixed size cost functors
template <typename CustomFunctor, typename T, int... Indices>
inline bool VariadicEvaluate(const CustomFunctor& custom_functor,
                             T const* const* input, T* output,
                             std::false_type /*is_dynamic*/,
                             std::integer_sequence<int, Indices...>) {
  static_assert(sizeof...(Indices) > 0,
                "Invalid number of parameter blocks. At least one parameter "
                "block must be specified.");
  return custom_functor(input[Indices]..., output);
}

class NdtCostFunctor {
 public:
  NdtCostFunctor() {}

  template <typename T>
  bool operator()(const T* const position1, const T* const orientation1,
                  const T* const position2, const T* const orientation2,
                  T* residual) const {
    std::cerr << "p1: " << position1[0] << ", " << position1[1] << ", "
              << position1[2] << std::endl;
    std::cerr << "p2: " << position2[0] << ", " << position2[1] << ", "
              << position2[2] << std::endl;
    residual[0] = position1[0] - position2[0];
    residual[1] = position1[1] - position2[1];
    residual[2] = position1[2] - position2[2];
    return true;
  }

 private:
};

int main(int argc, char** argv) {
  double p1[3] = {1, 2, 3};
  double p2[3] = {1, 2, 3};
  double q1[4] = {1, 0, 0, 0};
  double q2[4] = {1, 0, 0, 0};
  double const* inputs[] = {p1, q1, p2, q2};
  double residual[3] = {0.0, 0.0, 0.0};

  NdtCostFunctor ndt_cost_functor;
  VariadicEvaluate(ndt_cost_functor, inputs, residual, std::false_type{},
                   std::integer_sequence<int, 0, 1, 2, 3>{});

  return 0;
}