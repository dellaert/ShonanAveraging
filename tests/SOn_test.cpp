/**
 * Test optimizing with Ceres
 */

#include "SOn.h"
#include "gtest/gtest.h"

using namespace shonan;

namespace {

TEST(SOn, Hat) {
  Vector v(10);
  v << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;

  Matrix expected2(2, 2);
  expected2 << 0, -1, 1, 0;
  const auto actual2 = SOn::Hat(v.head<1>());
  ASSERT_TRUE(expected2.isApprox(actual2));

  Matrix expected3(3, 3);
  expected3 << 0, -3, 2, //
      3, 0, -1,          //
      -2, 1, 0;
  const auto actual3 = SOn::Hat(v.head<3>());
  ASSERT_TRUE(expected3.isApprox(actual3));

  Matrix expected4(4, 4);
  expected4 << 0, -6, 5, 3, //
      6, 0, -4, -2,         //
      -5, 4, 0, 1,          //
      -3, 2, -1, 0;
  const auto actual4 = SOn::Hat(v.head<6>());
  ASSERT_TRUE(expected4.isApprox(actual4));

  Matrix expected5(5, 5);
  expected5 << 0, -10, 9, 7, -4, //
      10, 0, -8, -6, 3,          //
      -9, 8, 0, 5, -2,           //
      -7, 6, -5, 0, 1,           //
      4, -3, 2, -1, 0;
  const auto actual5 = SOn::Hat(v);
  ASSERT_TRUE(expected5.isApprox(actual5));
}

TEST(SOn, RetractJacobian) {
  RowMajorMatrix expected(9, 3);
  expected << 0., 0., 0., //
      0., 0., 1.,         //
      0., -1, 0.,         //
      0., 0., -1,         //
      0., 0., 0.,         //
      1., 0., 0.,         //
      0., 1., 0.,         //
      -1, 0., 0.,         //
      0., 0., 0.;
  RowMajorMatrix actual = SOn::RetractJacobian<Eigen::RowMajor>(3);
  ASSERT_TRUE(expected.isApprox(actual));
}

} // namespace