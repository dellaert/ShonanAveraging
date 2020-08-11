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
  using MatrixMap = ceres::MatrixRef;           // badly named as really a map
  using ConstMatrixMap = ceres::ConstMatrixRef; // badly named as really a map

private:
  size_t n_, nn_, dim_;          ///< relevant dimensions
  mutable Matrix Rplus_H_Delta_; ///< Jacobian storage
  Matrix Delta_H_xi_;            ///< const nn x dim Jacobian of Retract at 0

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
    MatrixMap Rplus(x_plus_delta, n_, n_);
    ConstMatrixMap R(x, n_, n_);
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
  Matrix Jacobian(const Matrix &R) const {
    DMul(R, &Rplus_H_Delta_);
    // Apply chain rule
    return Rplus_H_Delta_ * Delta_H_xi_;
  }

  /// Calculate Jacobian
  bool ComputeJacobian(const double *x, double *jacobian) const override {
    if (jacobian) {
      ConstMatrixMap R(x, n_, n_);
      MatrixMap(jacobian, nn_, dim_) = Jacobian(R);
    }
    return true;
  }

  /// Overwrite MultiplyByJacobian, as it can benefit from a tiny bit of special
  /// structure in that Rplus_H_Delta_ = $R \otimes I$.
  bool MultiplyByJacobian(const double *x, const int num_rows,
                          const double *global_matrix,
                          double *local_matrix) const override {
    ConstMatrixMap R(x, n_, n_);
    MatrixMap L(local_matrix, num_rows, dim_);
    ConstMatrixMap J(global_matrix, num_rows, nn_);
    Matrix JR(num_rows, nn_);
    for (size_t j = 0; j < n_; j++) {
      auto block_j = JR.middleCols(n_ * j, n_);
      block_j = J.middleCols(n_ * 0, n_) * R(0, j);
      for (size_t k = 1; k < n_; k++) {
        block_j += J.middleCols(n_ * k, n_) * R(k, j);
      }
    }
    L = JR * Delta_H_xi_;
    return true;
  }
};

} // namespace shonan
