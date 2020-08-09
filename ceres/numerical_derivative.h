#pragma once

#include "ceres/internal/eigen.h"
#include "traits.h"

namespace ceres {

using Vector = ceres::Vector;
using Matrix = ceres::Matrix; // row major

/// Numerical derivate of Y-valued function h at x with type X
template <typename Y, typename X>
Matrix numericalDerivative(std::function<Y(const X &)> h, X x,
                           double delta = 1e-5) {
  // Vectorize x
  const size_t n = traits<X>::AmbientDim(x);
  Vector v(n);
  traits<X>::Vec(x, v.data());

  // get value h(x) at x and vectorize
  const Y hx = h(x);
  const size_t m = traits<Y>::AmbientDim(hx);
  Vector hv(m);
  traits<Y>::Vec(hx, hv.data());

  // Fill in Jacobian H
  Matrix H(m, n);
  Vector y1(m), y2(m);
  const double factor = 1.0 / (2.0 * delta);
  for (size_t j = 0; j < n; j++) {
    v(j) += delta;
    x = traits<X>::Unvec(n, v.data());
    traits<Y>::Vec(h(x), y1.data());
    v(j) -= 2.0 * delta;
    x = traits<X>::Unvec(n, v.data());
    traits<Y>::Vec(h(x), y2.data());
    v(j) += delta;
    H.col(j) = (y1 - y2) * factor;
  }
  return H;
}
} // namespace ceres