/**
 * Test optimizing with Ceres
 */

#include "SOn_parameterization.h"

#include "numerical_derivative.h"

#include "gtest/gtest.h"

#include <iostream>

using namespace std;
using namespace shonan;

// Check ComputeJacobian against numerical derivative for SO(3)
TEST(SOnParameterization, SO3) {
  Vector v(3);
  v << 1, 2, 3;
  const SOn Q(SOn::Retract(v));
  SOnParameterization param(3);
  auto h = [&](const Vector &xi) -> Matrix {
    return SOnParameterization::Retract(Q.matrix(), xi);
  };
  Vector xi(3);
  xi.setZero();
  Matrix expected = numericalDerivative(h, xi);
  RowMajorMatrix actual(9, 3);
  param.ComputeJacobian(Q.matrix().data(), actual.data());
  ASSERT_TRUE(expected.isApprox(actual, 1e-9));
}

// Check ComputeJacobian against numerical derivative for SO(4)
TEST(SOnParameterization, SO4) {
  Vector v(6);
  v << 1, 2, 3, 4, 5, 6;
  const SOn Q(SOn::Retract(v));
  SOnParameterization param(4);
  auto h = [&](const Vector &xi) -> Matrix {
    return SOnParameterization::Retract(Q.matrix(), xi);
  };
  Vector xi(6);
  xi.setZero();
  Matrix expected = numericalDerivative(h, xi);
  RowMajorMatrix actual(16, 6);
  param.ComputeJacobian(Q.matrix().data(), actual.data());
  ASSERT_TRUE(expected.isApprox(actual, 1e-9));
}
