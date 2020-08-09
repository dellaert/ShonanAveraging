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
  size_t n_, nn_, dim_;          ///< relevant dimensions
  mutable Matrix Rplus_H_Delta_; ///< Jacobian storage
  Matrix Delta_H_xi_;            ///< const intermediate Jacobian

public:
  explicit SOnParameterization(size_t n)
      : n_(n), nn_(n * n), dim_(SOn::Dimension(n)), Rplus_H_Delta_(nn_, nn_),
        Delta_H_xi_(SOn::RetractJacobian<Eigen::RowMajor>(n_)) {
    Rplus_H_Delta_.setZero();
  }
  virtual ~SOnParameterization() {}
  int GlobalSize() const override { return nn_; }
  int LocalSize() const override { return dim_; }

  // x * SOn::Retract(delta)
  static SOn Retract(const SOn &Q, const Vector &xi) {
    return Q * SOn::Retract(xi);
  }

  bool Plus(const double *x, const double *delta,
            double *x_plus_delta) const override {
    Eigen::Map<Matrix> Rplus(x_plus_delta, n_, n_);
    Eigen::Map<const Matrix> R(x, n_, n_);
    Eigen::Map<const Vector> xi(delta, dim_);
    auto Delta = SOn::Retract(xi).matrix();
    Rplus = R * Delta;
    return true;
  }

  bool ComputeJacobian(const double *x, double *jacobian) const override {
    if (jacobian) {
      // Jacobian of $R * Delta$ is Kronecker product $I \otimes R$
      Eigen::Map<const Matrix> R(x, n_, n_);
      for (size_t i = 0; i < n_; i++) {
        Rplus_H_Delta_.block(n_ * i, n_ * i, n_, n_) = R;
      }
      // Apply chain rule
      Eigen::Map<RowMajorMatrix> H(jacobian, nn_, dim_);
      H = Rplus_H_Delta_ * Delta_H_xi_;
    }
    return true;
  }
};

} // namespace shonan