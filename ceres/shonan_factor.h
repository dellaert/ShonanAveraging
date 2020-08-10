/**
 * @file shonan_factor.h
 * @brief Calculates Frobenius error between 2 Stiefel projections of SO(n)
 * @author Frank Dellaert
 * @date August 2020
 */

#pragma once

#include "SOn.h"
#include "ceres/ceres.h"
#include "matrix.h"
#include "parameters.h"

#include <iostream>

namespace shonan {

/// Calculates Frobenius error between 2 Stiefel projections of SO(n).
template <size_t d = 3> class ShonanFactor : public ceres::CostFunction {
public:
  using Matrix = ceres::Matrix;
  using Vector = ceres::Vector;
  using MatrixRef = ceres::MatrixRef;

private:
  Matrix M_;            ///< measured rotation between R1 and R2
  size_t p_, pp_, dim_; ///< dimensionality constants
  Matrix H1_, H2_;      ///< constant Jacobians

public:
  /// Construct from rotation variable index and prior mean, as an nxn matrix.
  explicit ShonanFactor(const SOn &R12, size_t p)
      : M_(R12.matrix()),                //
        p_(p), pp_(p * p), dim_(p_ * d), //
        H1_(dim_, pp_), H2_(dim_, pp_) {
    assert(R12.matrix().rows() == d);
    mutable_parameter_block_sizes()->push_back(pp_); // elements in SO(p) matrix
    mutable_parameter_block_sizes()->push_back(pp_); // same for second block
    set_num_residuals(d); // elements in Stiefel Frobenius norm

    // Pre-calculate Jacobians
    // TODO(frank): they are constant (and sparse!), how to handle in ceres?

    // There are p_ 3*3 blocks, one corresponding to each 1*3 row
    H1_.setZero();
    const Matrix minMt = -M_.transpose();
    for (size_t k = 0; k < p_; k++) {
      H1_.block<3, 3>(k * 3, k * p_) = minMt;
    }

    // There are p_ 3*3 identity matrices, one corresponding to each 1*3 row
    H2_.setZero();
    for (size_t k = 0; k < p_; k++) {
      H2_.block<3, 3>(k * 3, k * p_).setIdentity();
    }
  }

  virtual ~ShonanFactor() {}

  // Evaluate Frobenius error, with Matrix arguments
  // TODO: use MatrixRef for Maps?
  Vector Evaluate(const Matrix &M1, const Matrix &M2) const {
    // Vectorize and extract only d leftmost columns, i.e. vec(M2*P)
    Vector fQ2 = ceres::vec(M2.leftCols<d>());

    // Vectorize M1*P*R12
    Vector hQ1 = ceres::vec(M1.leftCols<3>() * M_);

    return fQ2 - hQ1;
  }

  // Evaluate Frobenius error, with SOn values
  Vector Evaluate(const SOn &Q1, const SOn &Q2) const {
    return Evaluate(Q1.matrix(), Q2.matrix());
  }

  // Evaluate Frobenius error
  bool Evaluate(double const *const *values, double *error,
                double **jacobians) const override {
    Eigen::Map<const Matrix> M1(values[0], p_, p_);
    Eigen::Map<const Matrix> M2(values[1], p_, p_);
    std::cout << values[0] << "," << values[1] << std::endl;
    if (error != nullptr) {
      Eigen::Map<Vector> e(error, dim_, 1);
      e = Evaluate(M1, M2);
      std::cout << error << ": " << e.transpose() << std::endl;
    }

    // Compute the Jacobian if asked for.
    if (jacobians != nullptr) {
      if (jacobians[0] != nullptr) {
        Eigen::Map<Matrix>(jacobians[0], dim_, pp_) = H1_;
      }
      if (jacobians[1] != nullptr) {
        Eigen::Map<Matrix>(jacobians[1], dim_, pp_) = H2_;
      }
    }
    return true;
  }
};

} // namespace shonan
