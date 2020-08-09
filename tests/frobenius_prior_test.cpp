/**
 * Test optimizing with Ceres
 */

#include "SOn_parameterization.h"
#include "frobenius_prior.h"
#include "numerical_derivative.h"

#include "ceres/ceres.h"

#include "gtest/gtest.h"

#include <iostream>

using namespace std;
using namespace shonan;

namespace so3 {
Vector v1 = (Vector(3) << 1, 2, 3).finished();
SOn R1 = SOn::Retract(v1);
Vector v2 = (Vector(3) << 0.01, 0.02, 0.03).finished();
SOn R2 = SOn::Retract(v2);
// SOn R12 = R1.between(R2);
} // namespace so3

TEST(ShonanCeresTest, FrobeniusPriorSO3) {
  // Create factor aka "cost function"
  using namespace ::so3;
  auto factor = new FrobeniusPrior(R2.matrix());

  // Initialize parameters
  double block0[9];
  double *parameters[1]{block0};
  Eigen::Map<Matrix> R(block0, 3, 3);
  R = R1.matrix();
  std::cout << "initial R:\n" << R << endl;

  // Call Evaluate
  double error[9];
  ASSERT_TRUE(factor->Evaluate(parameters, error, nullptr));

  // Check result
  Vector expected = R1.vec() - R2.vec();
  Eigen::Map<Vector> actual(error, 9, 1);
  ASSERT_TRUE(expected.isApprox(actual));

  // Check Jacobian against numerical derivative for SO(3)
  auto h = [factor](const Vector &Rvec) -> Matrix {
    double block0[9];
    double *parameters[1]{block0};
    Eigen::Map<Vector>(block0, 9, 1) = Rvec;
    double e[9];
    factor->Evaluate(parameters, e, nullptr);
    auto E = Eigen::Map<Matrix>(e, 3, 3);
    return E;
  };
  Matrix expectedH = numericalDerivative(h, R1.vec());
  RowMajorMatrix actualH(9, 9);
  double *jacobians[1]{actualH.data()};
  factor->Evaluate(parameters, error, jacobians);
  ASSERT_TRUE(expectedH.isApprox(actualH, 1e-9));

  // Build a problem.
  ceres::Problem problem;
  problem.AddResidualBlock(factor, nullptr, block0);
  auto *SOn_local_parameterization = new SOnParameterization(3);
  problem.SetParameterization(block0, SOn_local_parameterization);

  // Set solver options
  using ceres::Solver;
  Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;

  // Run the solver!
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << " ->\n" << R << endl;
  ASSERT_TRUE(R.isApprox(R2.matrix(), 1e-9));
}
