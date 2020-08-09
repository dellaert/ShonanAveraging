#pragma once
#include <Eigen/Dense>

namespace shonan {

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd; // column major

Vector vec(const Matrix &Q) {
  return Eigen::Map<const Vector>(Q.data(), Q.rows() * Q.cols(), 1);
}

/// Numerical derivate of Matrix-valued function h at Vector-valued x
Matrix numericalDerivative(std::function<Matrix(const Vector &)> h,
                           const Vector &x, double delta = 1e-5) {
  // get value at x, and corresponding chart
  const Matrix hx = h(x);

  // Bit of a hack for now to find number of rows
  const size_t m = hx.rows() * hx.cols(), n = x.size();

  // Prepare a tangent vector to perturb x with, only works for fixed size
  Vector dx(n);
  dx.setZero();

  // Fill in Jacobian H
  Matrix H(m, n);
  const double factor = 1.0 / (2.0 * delta);
  for (size_t j = 0; j < n; j++) {
    dx(j) = delta;
    const auto dy1 = vec(hx - h(x + dx));
    dx(j) = -delta;
    const auto dy2 = vec(hx - h(x + dx));
    dx(j) = 0;
    H.col(j) = (dy1 - dy2) * factor;
  }
  return H;
}
} // namespace shonan