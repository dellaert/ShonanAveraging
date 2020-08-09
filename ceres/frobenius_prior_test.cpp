/**
 * @file frobenius_prior_test.h
 * @brief Tests for FrobeniusPrior factor.
 * @author Frank Dellaert
 * @date August 2020
 */

#include "SOn_parameterization.h"
#include "frobenius_prior.h"
#include "numerical_derivative.h"
#include "parameters.h"

#include "ceres/ceres.h"

#include "gtest/gtest.h"

#include <iostream>

using namespace std;
using namespace shonan;

using Matrix = ceres::Matrix;
using Vector = ceres::Vector;

TEST(FrobeniusPrior, SO3) {
  // Some test values
  Vector v1 = (Vector(3) << 1, 2, 3).finished();
  SOn R1 = SOn::Retract(v1);
  Vector v2 = (Vector(3) << 0.01, 0.02, 0.03).finished();
  SOn R2 = SOn::Retract(v2);

  // Create factor aka "cost function"
  auto factor = new FrobeniusPrior(R2.matrix());

  // Initialize parameters
  ceres::Parameters<> parameters;
  parameters.Insert(1, R1);
  std::cout << "initial:\n" << parameters.At<SOn>(1) << endl;

  // Call unsafe Evaluate
  std::vector<size_t> keys{1};
  std::vector<double *> unsafe = parameters.Unsafe(keys);
  double error[9];
  ASSERT_TRUE(factor->Evaluate(unsafe.data(), error, nullptr));

  // Check result
  Vector expected = R1.vec() - R2.vec();
  Eigen::Map<Vector> actual(error, 9, 1);
  ASSERT_TRUE(expected.isApprox(actual));

  // Check result of safe Evaluate
  ASSERT_TRUE(expected.isApprox(factor->Evaluate(R1)));

  // Check Jacobian against numerical derivative for SO(3)
  auto h = [factor](const SOn &R) -> Vector {
    return factor->Evaluate(R);
  };
  Matrix expectedH = ceres::numericalDerivative<Matrix, SOn>(h, R1);
  Matrix actualH(9, 9);
  double *jacobians[1]{actualH.data()};
  factor->Evaluate(unsafe.data(), error, jacobians);
  ASSERT_TRUE(expectedH.isApprox(actualH, 1e-9));

  // Build a problem.
  ceres::Problem problem;
  problem.AddResidualBlock(factor, nullptr, parameters.Unsafe(1));
  auto *SOn_local_parameterization = new SOnParameterization(3);
  problem.SetParameterization(parameters.Unsafe(1), SOn_local_parameterization);

  // Set solver options
  using ceres::Solver;
  Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;

  // Run the solver!
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << " ->\n" << parameters.At<SOn>(1) << endl;
  ASSERT_TRUE(R2.matrix().isApprox(parameters.At<SOn>(1).matrix(), 1e-9));
}
