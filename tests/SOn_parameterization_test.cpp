/**
 * Test optimizing with Ceres
 */

#include "SOn_parameterization.h"

#include "numerical_derivative.h"

#include "gtest/gtest.h"

#include <iostream>

using namespace std;
using namespace shonan;

using Matrix = ceres::Matrix;
using Vector = ceres::Vector;

// Check Plus with Retract
TEST(SOnParameterization, SO3Plus) {
  Vector v(3);
  v << 1, 2, 3;
  const SOn Q(SOn::Retract(v));
  SOnParameterization param(3);
  Vector xi(3);
  v << 4, 5, 6;
  auto expected = SOnParameterization::Retract(Q, xi);
  Matrix actual(3, 3);
  param.Plus(Q.matrix().data(), xi.data(), actual.data());
  ASSERT_TRUE(expected.matrix().isApprox(actual, 1e-9));
}

// Check derivative of mul against numerical derivative
TEST(SOnParameterization, DMul) {
  Vector u(3);
  u << 1, 2, 3;
  const SOn A(SOn::Retract(u));
  SOnParameterization param(3);
  auto h = [A](const SOn &B) -> SOn { return A * B; };
  Vector v(3);
  v << 4, 5, 6;
  const SOn B(SOn::Retract(v));
  Matrix expected = ceres::numericalDerivative<SOn, SOn>(h, B);
  Matrix actual(9, 9);
  param.DMul(A.matrix(), &actual);
  ASSERT_TRUE(expected.isApprox(actual, 1e-9));
}

// Check ComputeJacobian against numerical derivative for SO(3)
TEST(SOnParameterization, SO3ComputeJacobian) {
  Vector v(3);
  v << 1, 2, 3;
  const SOn Q(SOn::Retract(v));
  SOnParameterization param(3);
  auto h = [&](const Vector &xi) -> SOn {
    return SOnParameterization::Retract(Q, xi);
  };
  Vector xi(3);
  xi.setZero();
  Matrix expected = ceres::numericalDerivative<SOn, Vector>(h, xi);
  Matrix actual(9, 3);
  param.ComputeJacobian(Q.matrix().data(), actual.data());
  cout << expected << endl << endl;
  cout << actual << endl;
  ASSERT_TRUE(expected.isApprox(actual, 1e-9));
}

// Check ComputeJacobian against numerical derivative for SO(4)
TEST(SOnParameterization, SO4ComputeJacobian) {
  Vector v(6);
  v << 1, 2, 3, 4, 5, 6;
  const SOn Q(SOn::Retract(v));
  SOnParameterization param(4);
  auto h = [&](const Vector &xi) -> SOn {
    return SOnParameterization::Retract(Q, xi);
  };
  Vector xi(6);
  xi.setZero();
  Matrix expected = ceres::numericalDerivative<SOn, Vector>(h, xi);
  Matrix actual(16, 6);
  param.ComputeJacobian(Q.matrix().data(), actual.data());
  ASSERT_TRUE(expected.isApprox(actual, 1e-9));
}
