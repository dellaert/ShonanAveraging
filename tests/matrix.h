/**
 * @file matrix.h
 * @brief Matrix typedefs and utilities.
 * @author Frank Dellaert
 * @date August 2020
 */

#pragma once
#include <Eigen/Dense>

namespace shonan {

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd; // column major
using RowMajorMatrix =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

/// Vectorize a matrix, column major
Vector vec(const Matrix &Q) {
  return Eigen::Map<const Vector>(Q.data(), Q.rows() * Q.cols(), 1);
}

} // namespace shonan