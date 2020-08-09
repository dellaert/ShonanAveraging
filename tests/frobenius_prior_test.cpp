/**
 * Test optimizing with Ceres
 */

#include "SOn.h"
#include "frobenius_prior.h"

#include "ceres/ceres.h"

#include "gtest/gtest.h"

#include <Eigen/Dense>

#include <iostream>

using ceres::CostFunction;

using namespace std;
using namespace shonan;

namespace so3 {
Vector v1 = (Vector(3) << 0.1, 0, 0).finished();
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
  double block0[9], error[9];
  double *parameters[1]{block0};
  Eigen::Map<Matrix> R(block0, 3, 3);
  R = R1.matrix();

  // Call Evaluate
  ASSERT_TRUE(factor->Evaluate(parameters, error, nullptr));

  // Check result
  Vector expected = R1.vec() - R2.vec();
  Eigen::Map<Vector> actual(error, 9, 1);
  ASSERT_TRUE(expected.isApprox(actual));
}
