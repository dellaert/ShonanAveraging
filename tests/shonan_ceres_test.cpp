/**
 * Test optimizing with Ceres
 */

#include <Eigen/Dense>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

using ceres::AutoDiffCostFunction;
// using ceres::DynamicAutoDiffCostFunction;
// using ceres::CauchyLoss;
using ceres::CostFunction;
// using ceres::LossFunction;
using ceres::Problem;
using ceres::Solver;
#include "gtest/gtest.h"

namespace {

// Relative pose error as one might use in SE(3) pose graph optimization.
// The measurement is a relative pose T_i_j, and the values are absolute
// poses T_w_i and T_w_j. For the residual we use the log of the the residual
// pose, in split representation SO(3) x R^3.
struct RelativePoseError {
  RelativePoseError(const Eigen::Quaterniond &q_i_j,
                    const Eigen::Vector3d &t_i_j)
      : meas_q_i_j_(q_i_j), meas_t_i_j_(t_i_j) {}

  template <typename T>
  inline bool operator()(const T *const pose_i_ptr, const T *const pose_j_ptr,
                         T *residuals_ptr) const {
    Eigen::Map<const Eigen::Quaternion<T>> q_w_i(pose_i_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_w_i(pose_i_ptr + 4);
    Eigen::Map<const Eigen::Quaternion<T>> q_w_j(pose_j_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_w_j(pose_j_ptr + 4);
    Eigen::Map<Eigen::Matrix<T, 6, 1>> error(residuals_ptr);

    // Compute estimate of relative pose from i to j.
    const Eigen::Quaternion<T> est_q_j_i = q_w_j.conjugate() * q_w_i;
    const Eigen::Matrix<T, 3, 1> est_t_j_i =
        q_w_j.conjugate() * (t_w_i - t_w_j);

    // Compute residual pose.
    const Eigen::Quaternion<T> res_q = meas_q_i_j_.cast<T>() * est_q_j_i;
    const Eigen::Matrix<T, 3, 1> res_t =
        meas_q_i_j_.cast<T>() * est_t_j_i + meas_t_i_j_;

    // Convert quaternion to ceres convention (Eigen stores xyzw, Ceres wxyz).
    Eigen::Matrix<T, 4, 1> res_q_ceres;
    res_q_ceres << res_q.w(), res_q.vec();

    // Residual is log of pose. Use split representation SO(3) x R^3.
    QuaternionToAngleAxis(res_q_ceres.data(), error.data());
    error.template bottomRows<3>() = res_t;

    return true;
  }

private:
  // Measurement of relative pose from j to i.
  Eigen::Quaterniond meas_q_i_j_;
  Eigen::Vector3d meas_t_i_j_;
};

struct CostFunctor {
  template <typename T> bool operator()(const T *const x, T *residual) const {
    residual[0] = T(10.0) - x[0];
    return true;
  }
};

class QuadraticFactor : public ceres::SizedCostFunction<1, 1> {
public:
  virtual ~QuadraticFactor() {}
  virtual bool Evaluate(double const *const *values, double *error,
                        double **jacobians) const {
    const double x = values[0][0];
    error[0] = 10 - x;

    // Compute the Jacobian if asked for.
    if (jacobians != nullptr && jacobians[0] != nullptr) {
      jacobians[0][0] = -1;
    }
    return true;
  }
};

// Plus(x, delta) = [cos(|delta|), sin(|delta|) delta / |delta|] * x
// with * being the quaternion multiplication operator. Here we assume
// that the first element of the quaternion vector is the real (cos
// theta) part.
class CERES_EXPORT QuaternionParameterization : public ceres::LocalParameterization {
 public:
  virtual ~QuaternionParameterization() {}
  bool Plus(const double* x,
            const double* delta,
            double* x_plus_delta) const override;
  bool ComputeJacobian(const double* x, double* jacobian) const override;
  int GlobalSize() const override { return 4; }
  int LocalSize() const override { return 3; }
};

bool QuaternionParameterization::Plus(const double* x,
                                      const double* delta,
                                      double* x_plus_delta) const {
  const double norm_delta =
      sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
  if (norm_delta > 0.0) {
    const double sin_delta_by_delta = (sin(norm_delta) / norm_delta);
    double q_delta[4];
    q_delta[0] = cos(norm_delta);
    q_delta[1] = sin_delta_by_delta * delta[0];
    q_delta[2] = sin_delta_by_delta * delta[1];
    q_delta[3] = sin_delta_by_delta * delta[2];
    ceres::QuaternionProduct(q_delta, x, x_plus_delta);
  } else {
    for (int i = 0; i < 4; ++i) {
      x_plus_delta[i] = x[i];
    }
  }
  return true;
}

bool QuaternionParameterization::ComputeJacobian(const double* x,
                                                 double* jacobian) const {
  // clang-format off
  jacobian[0] = -x[1];  jacobian[1]  = -x[2];   jacobian[2]  = -x[3];
  jacobian[3] =  x[0];  jacobian[4]  =  x[3];   jacobian[5]  = -x[2];
  jacobian[6] = -x[3];  jacobian[7]  =  x[0];   jacobian[8]  =  x[1];
  jacobian[9] =  x[2];  jacobian[10] = -x[1];   jacobian[11] =  x[0];
  // clang-format on
  return true;
}

// Tests some.
TEST(ShonanCeresTest, Trivial) {
  // The variable to solve for with its initial value.
  double initial_x = 5.0;
  double x = initial_x;

  // Build the problem.
  Problem problem;

  QuadraticFactor *cost_function = new QuadraticFactor();
  problem.AddResidualBlock(cost_function, nullptr, &x);

  // Run the solver!
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x << " -> " << x << "\n";
  // EXPECT_EQ(1, 0);
}

} // namespace