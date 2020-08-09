/**
 * @file SOn.h
 * @brief Class to represent rtations in n-dimensions SO(n)
 * @author Frank Dellaert
 * @date August 2020
 */

#pragma once

#include "matrix.h"
#include "traits.h"

#include <Eigen/Dense>

namespace shonan {

class SOn {
public:
protected:
  Matrix matrix_; ///< Rotation matrix

public:
  /// @name Constructors
  /// @{

  /// Constructor from Eigen Matrix, dynamic version
  template <typename Derived>
  explicit SOn(const Eigen::MatrixBase<Derived> &R) : matrix_(R.eval()) {}

  /// Named constructor from lower dimensional matrix
  template <typename Derived>
  static SOn Lift(size_t n, const Eigen::MatrixBase<Derived> &R) {
    Matrix Q = Matrix::Identity(n, n);
    size_t p = R.rows();
    assert(p <= n && R.cols() == p);
    Q.topLeftCorner(p, p) = R;
    return SOn(Q);
  }

  /// Named constructor from lower dimensional SOn
  static SOn Lift(size_t n, const SOn &R) { return Lift(n, R.matrix()); }

  /// @}
  /// @name Standard methods
  /// @{

  /// Return matrix
  const Matrix &matrix() const { return matrix_; }

  /// Multiplication
  SOn operator*(const SOn &g) const { return SOn(matrix() * g.matrix()); }

  /// Between
  SOn between(const SOn &g) const {
    return SOn(matrix().inverse() * g.matrix());
  }

  /// Return vectorized rotation matrix in column order.
  Vector vec(double **H = nullptr) const {
    // If requested, calculate H as (I \oplus Q) * P,
    // where Q is the N*N rotation matrix, and P is calculated below.
    if (H) {
      throw std::runtime_error("SOn::vec jacobian not implemented.");
    }
    return shonan::vec(matrix());
  }

  friend std::ostream &operator<<(std::ostream &os, const SOn &Q) {
    os << Q.matrix();
    return os;
  }

  /// @}
  /// @name Manifold
  /// @{

  // Calculate ambient dimension n from manifold dimensionality d.
  static size_t AmbientDim(size_t d) { return (1 + std::sqrt(1 + 8 * d)) / 2; }

  // Calculate manifold dimensionality for SO(n).
  static size_t Dimension(size_t n) { return n * (n - 1) / 2; }

  /**
   * Hat operator creates Lie algebra element corresponding to d-vector, where d
   * is the dimensionality of the manifold. This function is implemented
   * recursively, and the d-vector is assumed to laid out such that the last
   * element corresponds to so(2), the last 3 to so(3), the last 6 to so(4)
   * etc... For example, the vector-space isomorphic to so(5) is laid out as:
   *   a b c d | u v w | x y | z
   * where the latter elements correspond to "telescoping" sub-algebras:
   *   0 -z  y  w -d
   *   z  0 -x -v  c
   *  -y  x  0  u -b
   *  -w  v -u  0  a
   *   d -c  b -a  0
   * This scheme behaves exactly as expected for SO(2) and SO(3).
   */
  static Matrix Hat(const Vector &xi) {
    size_t n = AmbientDim(xi.size());
    if (n < 2)
      throw std::invalid_argument("SO<N>::Hat: n<2 not supported");

    Matrix X(n, n); // allocate space for n*n skew-symmetric matrix
    X.setZero();
    if (n == 2) {
      // Handle SO(2) case as recursion bottom
      assert(xi.size() == 1);
      X << 0, -xi(0), xi(0), 0;
    } else {
      // Recursively call SO(n-1) call for top-left block
      const size_t dmin = (n - 1) * (n - 2) / 2;
      X.topLeftCorner(n - 1, n - 1) = Hat(xi.tail(dmin));

      // determine sign of last element (signs alternate)
      double sign = pow(-1.0, xi.size());
      // Now fill last row and column
      for (size_t i = 0; i < n - 1; i++) {
        const size_t j = n - 2 - i;
        X(n - 1, j) = -sign * xi(i);
        X(j, n - 1) = -X(n - 1, j);
        sign = -sign;
      }
    }
    return X;
  }

  static SOn Retract(const Vector &xi, double **H = nullptr) {
    if (H) {
      throw std::runtime_error("SOn::Retract jacobian not implemented.");
    }
    const Matrix X = Hat(xi / 2.0);
    size_t n = AmbientDim(xi.size());
    const auto I = Eigen::MatrixXd::Identity(n, n);
    return SOn((I + X) * (I - X).inverse());
  }

  /**
   * Jacobian of Retract at xi==0 is the n^2 x dim matrix of vectorized Lie
   * algebra generators for SO(n). We allow return a row-major matrix here to
   * play nicely with Ceres.
   */
  template <int Options = 0>
  static Eigen::Matrix<double, -1, -1, Options> RetractJacobian(size_t n) {
    assert(n > 0);
    const size_t n2 = n * n, dim = Dimension(n);
    Eigen::Matrix<double, -1, -1, Options> G(n2, dim);
    for (size_t j = 0; j < dim; j++) {
      const auto X = Hat(Vector::Unit(dim, j));
      G.col(j) = Eigen::Map<const Matrix>(X.data(), n2, 1);
    }
    return G;
  }

  /// @}
}; // SOn

} // namespace shonan

namespace ceres {
// define ceres traits for SOn
template <> struct traits<shonan::SOn> {
  /// Ambient dimensions = n^2
  static size_t AmbientDim(const shonan::SOn &t) {
    size_t n = t.matrix().rows();
    return n * n;
  }

  /// Vectorize to doubles in pre-allocated block
  static void Vec(const shonan::SOn &t, double *const block) {
    size_t n = t.matrix().rows();
    Eigen::Map<shonan::Matrix>(block, n, n) = t.matrix();
  }

  /// initialize from vectorized storage
  static shonan::SOn Unvec(size_t nn, const double *const block) {
    using std::floor;
    using std::sqrt;
    size_t n = static_cast<size_t>(floor(sqrt(static_cast<float>(nn))));
    assert(n * n == nn);
    return shonan::SOn(Eigen::Map<const shonan::Matrix>(block, n, n));
  }
};

} // namespace ceres
