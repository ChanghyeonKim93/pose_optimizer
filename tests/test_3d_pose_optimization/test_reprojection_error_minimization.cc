#include <iostream>
#include <random>

#include "pose_optimizer/single_pose_optimizer/cost_function.h"
#include "pose_optimizer/single_pose_optimizer/single_pose_optimizer.h"

using namespace pose_optimizer::single_pose_optimizer;

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;

struct CameraIntrinsics {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  double inv_fx{0.0};
  double inv_fy{0.0};
  int width{0};
  int height{0};
};

struct Correspondence {
  Vec3 local_point{Vec3::Zero()};    // represented in reference frame
  Vec2 matched_pixel{Vec2::Zero()};  // represented in query frame
};

class ReprojectionErrorCostFunctor : public SizedCostFunction<2, 3> {
 public:
  ReprojectionErrorCostFunctor(
      const Correspondence& correspondence,
      const std::shared_ptr<CameraIntrinsics>& camera_intrinsics)
      : correspondence_(correspondence),
        camera_intrinsics_(camera_intrinsics) {}

  bool Evaluate(const RotationMatrix& R, const TranslationVector& t,
                double* jacobian_matrix_ptr,
                double* residual_vector_ptr) final {
    if (jacobian_matrix_ptr == nullptr || residual_vector_ptr == nullptr)
      return false;

    Eigen::Map<Eigen::Matrix<double, 2, 6>> jacobian(jacobian_matrix_ptr);
    Eigen::Map<Eigen::Matrix<double, 2, 1>> residual(residual_vector_ptr);
    jacobian.setZero();
    residual.setZero();

    const auto& p = correspondence_.local_point;
    const auto& pixel = correspondence_.matched_pixel;

    const Vec3 warped_p = R * p + t;
    if (warped_p.z() <= 0.02) {
      // std::cerr << "Error: warped point has non-positive depth: "
      //           << warped_p.z() << std::endl;
      return false;
    }

    const double inv_z = 1.0 / warped_p.z();
    const double sq_inv_z = inv_z * inv_z;
    Vec2 projected_pixel;
    projected_pixel.x() =
        camera_intrinsics_->fx * warped_p.x() * inv_z + camera_intrinsics_->cx;
    projected_pixel.y() =
        camera_intrinsics_->fy * warped_p.y() * inv_z + camera_intrinsics_->cy;

    residual = projected_pixel - pixel;

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
    Eigen::Matrix<double, 2, 3> dpi_dx{Eigen::Matrix<double, 2, 3>::Zero()};
    dpi_dx(0, 0) = camera_intrinsics_->fx * inv_z;
    dpi_dx(0, 1) = 0.0;
    dpi_dx(0, 2) = -camera_intrinsics_->fx * warped_p.x() * sq_inv_z;
    dpi_dx(1, 0) = 0.0;
    dpi_dx(1, 1) = camera_intrinsics_->fy * inv_z;
    dpi_dx(1, 2) = -camera_intrinsics_->fy * warped_p.y() * sq_inv_z;
    jacobian.block<2, 3>(0, 0) = dpi_dx;
    jacobian.block<2, 3>(0, 3) = -dpi_dx * R_skew_p;
    return true;
  }

 private:
  const Correspondence correspondence_;
  std::shared_ptr<CameraIntrinsics> camera_intrinsics_;
};

std::vector<Vec3> GenerateReferencePoints();
std::vector<Vec3> WarpPoints(const std::vector<Vec3>& points,
                             const Eigen::Isometry3d& pose);
std::vector<Vec2> ProjectToPixel(const std::vector<Vec3>& local_points,
                                 const CameraIntrinsics& camera_intrinsics);

using Pose = Eigen::Isometry3d;

int main(int argc, char** argv) {
  CameraIntrinsics camera_intrinsics;
  camera_intrinsics.fx = 525.0;
  camera_intrinsics.fy = 525.0;
  camera_intrinsics.cx = 320.0;
  camera_intrinsics.cy = 240.0;
  camera_intrinsics.inv_fx = 1.0 / camera_intrinsics.fx;
  camera_intrinsics.inv_fy = 1.0 / camera_intrinsics.fy;
  camera_intrinsics.width = 640;
  camera_intrinsics.height = 480;
  std::shared_ptr<CameraIntrinsics> cam_intrinsic_ptr =
      std::make_shared<CameraIntrinsics>(camera_intrinsics);

  // Make global points
  const auto reference_points = GenerateReferencePoints();
  std::cerr << "# points: " << reference_points.size() << std::endl;

  // Set true pose
  Pose true_pose{Pose::Identity()};
  true_pose.translation() = Vec3(-0.1, 0.123, -0.5);
  true_pose.linear() =
      Eigen::AngleAxisd(0.1, Vec3(0.0, 0.0, 1.0)).toRotationMatrix();

  const auto query_points = WarpPoints(reference_points, true_pose.inverse());
  const auto matched_pixels = ProjectToPixel(query_points, camera_intrinsics);
  std::vector<Correspondence> correspondences;
  for (size_t index = 0; index < query_points.size(); ++index) {
    Correspondence corr;
    corr.local_point = reference_points.at(index);
    corr.matched_pixel = matched_pixels.at(index);
    correspondences.push_back(corr);
  }

  std::unique_ptr<pose_optimizer::single_pose_optimizer::SinglePoseOptimizer<3>>
      optimizer = std::make_unique<
          pose_optimizer::single_pose_optimizer::SinglePoseOptimizer<3>>();

  pose_optimizer::single_pose_optimizer::Problem<3> problem;
  // Add residual blocks
  for (const auto& corr : correspondences) {
    std::shared_ptr<CostFunction<3>> cost_function =
        std::make_shared<ReprojectionErrorCostFunctor>(corr, cam_intrinsic_ptr);
    std::shared_ptr<LossFunction> loss_function = nullptr;
    problem.AddResidualBlock(cost_function, loss_function);
  }

  Pose pose{Pose::Identity()};
  pose_optimizer::Options options;
  pose_optimizer::Summary summary;
  optimizer->Solve(problem, options, &pose, &summary);

  std::cerr << summary.GetBriefReport() << std::endl;

  return 0;
}

std::vector<Vec3> GenerateReferencePoints() {
  std::random_device rd;
  std::normal_distribution<double> nd_z(0.0, 0.1);

  const double z = 3.0;
  const double x_min = -1.6;
  const double x_max = 1.6;
  const double y_min = -1.2;
  const double y_max = 1.2;
  const double point_step = 0.1;

  std::vector<Eigen::Vector3d> points;
  double x, y;
  for (x = x_min; x <= x_max; x += point_step) {
    for (y = y_min; y <= y_max; y += point_step) {
      points.push_back(Vec3(x, y, z));
    }
  }

  return points;
}

std::vector<Vec3> WarpPoints(const std::vector<Vec3>& points,
                             const Eigen::Isometry3d& pose) {
  std::vector<Vec3> warped_points;
  warped_points.reserve(points.size());
  for (const auto& point : points) warped_points.push_back(pose * point);
  return warped_points;
}

std::vector<Vec2> ProjectToPixel(const std::vector<Vec3>& local_points,
                                 const CameraIntrinsics& camera_intrinsics) {
  std::vector<Vec2> projected_pixels;
  for (const auto& local_point : local_points) {
    Vec2 pixel;
    const double inv_z = 1.0 / local_point.z();
    pixel.x() =
        camera_intrinsics.fx * local_point.x() * inv_z + camera_intrinsics.cx;
    pixel.y() =
        camera_intrinsics.fy * local_point.y() * inv_z + camera_intrinsics.cy;
    projected_pixels.push_back(pixel);
  }
  return projected_pixels;
}
