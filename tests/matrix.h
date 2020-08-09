/**
 * @file matrix.h
 * @brief Matrix typedefs and utilities.
 * @author Frank Dellaert
 * @date August 2020
 */

#pragma once
#include "ceres/internal/eigen.h"
#include "traits.h"

namespace ceres {

/// Vectorize a matrix, row-major
Vector vec(const Matrix &Q) {
  return Eigen::Map<const Vector>(Q.data(), Q.rows() * Q.cols(), 1);
}

// define ceres traits for Vector
template <> struct traits<Vector> {
  /// Ambient dimensions = n^2
  static size_t AmbientDim(const Vector &t) { return t.size(); }

  /// Vectorize to doubles in pre-allocated block
  static void Vec(const Vector &t, double *const block) {
    Eigen::Map<Vector>(block, t.size(), 1) = t;
  }

  /// initialize from vectorized storage
  static Vector Unvec(size_t n, const double *const block) {
    return Vector(Eigen::Map<const Vector>(block, n, 1));
  }
};

// define ceres traits for Matrix
template <> struct traits<Matrix> {
  /// Ambient dimensions = n^2
  static size_t AmbientDim(const Matrix &t) { return t.size(); }

  /// Vectorize to doubles in pre-allocated block
  static void Vec(const Matrix &t, double *const block) {
    Eigen::Map<Matrix>(block, t.rows(), t.cols()) = t;
  }

  /// initialize from vectorized storage
  /// Always returns a vector, so in general Unvec(Vec(M)) != M
  static Matrix Unvec(size_t n, const double *const block) {
    return Matrix(Eigen::Map<const Matrix>(block, n, 1));
  }
};
} // namespace ceres