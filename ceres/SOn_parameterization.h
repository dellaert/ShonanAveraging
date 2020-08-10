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
public:
  using Matrix = ceres::Matrix;
  using Vector = ceres::Vector;

private:
  size_t n_, nn_, dim_;          ///< relevant dimensions
  mutable Matrix Rplus_H_Delta_; ///< Jacobian storage
  Matrix Delta_H_xi_;            ///< const intermediate Jacobian

public:
  explicit SOnParameterization(size_t n)
      : n_(n), nn_(n * n), dim_(SOn::Dimension(n)), Rplus_H_Delta_(nn_, nn_),
        Delta_H_xi_(SOn::RetractJacobian<Eigen::RowMajor>(n_)) {}
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

  // Jacobian of A*B in B is Kronecker product $A \otimes I$
  void DMul(const Matrix &A, Matrix *H) const {
    const Matrix I_nxn = Matrix::Identity(n_, n_);
    for (size_t i = 0; i < n_; i++) {
      for (size_t j = 0; j < n_; j++) {
        H->block(n_ * i, n_ * j, n_, n_) = A(i, j) * I_nxn;
      }
    }
  }

  /// Calculate Jacobian from a Matrix
  // TODO(frank): use Ref?
  Matrix Jacobian(const Matrix&R) const  {
    DMul(R, &Rplus_H_Delta_);
    // Apply chain rule
    return Rplus_H_Delta_ * Delta_H_xi_;
  }
  
  /// Calculate Jacobian
  bool ComputeJacobian(const double *x, double *jacobian) const override {
    if (jacobian) {
      Eigen::Map<const Matrix> R(x, n_, n_);
      Eigen::Map<Matrix>(jacobian, nn_, dim_) = Jacobian(R);
    }
    return true;
  }
};

} // namespace shonan
