#ifndef POSE_OPTIMIZER_TYPES_H_
#define POSE_OPTIMIZER_TYPES_H_

#include "Eigen/Dense"

namespace pose_optimizer {

using Mat1x1 = Eigen::Matrix<double, 1, 1>;
using Mat1x2 = Eigen::Matrix<double, 1, 2>;
using Mat1x3 = Eigen::Matrix<double, 1, 3>;
using Mat1x4 = Eigen::Matrix<double, 1, 4>;
using Mat1x5 = Eigen::Matrix<double, 1, 5>;
using Mat1x6 = Eigen::Matrix<double, 1, 6>;

using Mat1x1 = Eigen::Matrix<double, 1, 1>;
using Mat2x1 = Eigen::Matrix<double, 2, 1>;
using Mat3x1 = Eigen::Matrix<double, 3, 1>;
using Mat4x1 = Eigen::Matrix<double, 4, 1>;
using Mat5x1 = Eigen::Matrix<double, 5, 1>;
using Mat6x1 = Eigen::Matrix<double, 6, 1>;

using Mat2x2 = Eigen::Matrix<double, 2, 2>;
using Mat2x3 = Eigen::Matrix<double, 2, 3>;
using Mat3x2 = Eigen::Matrix<double, 3, 2>;

using Mat3x3 = Eigen::Matrix<double, 3, 3>;

using Mat3x6 = Eigen::Matrix<double, 3, 6>;
using Mat6x3 = Eigen::Matrix<double, 6, 3>;
using Mat6x6 = Eigen::Matrix<double, 6, 6>;

using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec4 = Eigen::Matrix<double, 4, 1>;
using Vec5 = Eigen::Matrix<double, 5, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;

using Pose2 = Eigen::Transform<double, 2, 1>;
using Pose3 = Eigen::Transform<double, 3, 1>;

using Orientation = Eigen::Quaternion<double>;

}  // namespace pose_optimizer

#endif  // POSE_OPTIMIZER_TYPES_H_