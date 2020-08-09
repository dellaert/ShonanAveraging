/**
 * @file frobenius_prior.h
 * @brief Factor that minimizes Frobenius error to a given "prior mean" matrix
 * @author Frank Dellaert
 * @date August 2020
 */

#pragma once

#include "SOn.h"
#include "ceres/ceres.h"

namespace shonan {

/// Calculates the Frobenius norm between a given matrix and element of SO(n).
class FrobeniusPrior : public ceres::CostFunction {
public:
  using Matrix = ceres::Matrix;
  using Vector = ceres::Vector;

private:
  Vector vectorized_mean_;
  size_t n_, nn_; ///< dimensionality constants

public:
  /// Construct from rotation variable index and prior mean, as an nxn matrix.
  explicit FrobeniusPrior(const Matrix &mean)
      : vectorized_mean_(ceres::vec(mean)), n_(mean.rows()), nn_(n_ * n_) {
    assert(mean.cols() == n_);
    mutable_parameter_block_sizes()->push_back(nn_); // elements in SO(n) matrix
    set_num_residuals(nn_); // elements in Frobenius norm
  }

  virtual ~FrobeniusPrior() {}

  // Evaluate Frobenius error
  Vector Evaluate(const SOn &Q) const { return Q.vec() - vectorized_mean_; }

  // Evaluate Frobenius error
  bool Evaluate(double const *const *values, double *error,
                double **jacobians) const override {
    Eigen::Map<const Vector> V(values[0], nn_, 1);
    Eigen::Map<Vector> e(error, nn_, 1);
    e = V - vectorized_mean_;

    // Compute the Jacobian if asked for.
    if (jacobians != nullptr && jacobians[0] != nullptr) {
      Eigen::Map<Matrix> H(jacobians[0], nn_, nn_);
      H.setIdentity();
      // It would be nicer if we could return this much smaller matrix and not
      // incur a large matrix multiplication within Ceres
      // H = SOn::RetractJacobian<Eigen::RowMajor>(n_);
    }
    return true;
  }
};

} // namespace shonan
