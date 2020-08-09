/**
 * Ceres local parameterization for SOn
 */

#pragma once

#include "SOn.h"
#include "ceres/ceres.h"

namespace shonan {
// Plus(x, delta) = x * SOn::Retract(delta)
// with * being the SOn multiplication operator.
class CERES_EXPORT SOnParameterization : public ceres::LocalParameterization {
  size_t n_, nn_, dim_;

public:
  explicit SOnParameterization(size_t n)
      : n_(n), nn_(n * n), dim_(SOn::Dimension(n)) {}
  virtual ~SOnParameterization() {}
  int GlobalSize() const override { return nn_; }
  int LocalSize() const override { return dim_; }

  // x * SOn::Retract(delta)
  static Matrix Retract(const Matrix &R, const Vector &xi) {
    return R * SOn::Retract(xi).matrix();
  }

  bool Plus(const double *x, const double *delta,
            double *x_plus_delta) const override {
    Eigen::Map<Matrix> Rplus(x_plus_delta, n_, n_);
    Eigen::Map<const Matrix> R(x, n_, n_);
    Eigen::Map<const Vector> xi(delta, dim_);
    Rplus = R * SOn::Retract(xi).matrix();
    return true;
  }

  bool ComputeJacobian(const double *x, double *jacobian) const override {
    if (jacobian) {
      // Jacobian of multiplication is - kronecker_product(I, R)
      Matrix Dmul(nn_, nn_);
      Dmul.setZero();
      Eigen::Map<const Matrix> R(x, n_, n_);
      for (size_t i = 0; i < n_; i++) {
        Dmul.block(n_ * i, n_ * i, n_, n_) = -R;
      }
      // Apply chain rule
      Eigen::Map<RowMajorMatrix> H(jacobian, nn_, dim_);
      H = Dmul * SOn::RetractJacobian<Eigen::RowMajor>(n_);
    }
    return true;
  }
};

} // namespace shonan