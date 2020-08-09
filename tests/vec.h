#pragma once
#include <Eigen/Dense>

namespace shonan {

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd; // column major

/// Vectorize a matrix
Vector vec(const Matrix &Q) {
  return Eigen::Map<const Vector>(Q.data(), Q.rows() * Q.cols(), 1);
}

} // namespace shonan