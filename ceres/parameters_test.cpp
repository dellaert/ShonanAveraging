/**
 * @file parameters_test.h
 * @brief Tests for Paremerts classes.
 * @author Frank Dellaert
 * @date August 2020
 */

#include "parameters.h"
#include "SOn.h"

#include "ceres/ceres.h"

#include "gtest/gtest.h"

#include <iostream>
#include <string>

using namespace std;
using namespace shonan;

using Matrix = ceres::Matrix;
using Vector = ceres::Vector;

TEST(ParameterBuffer, SOn) {
    // Create buffer and add SO(4) object
    ceres::ParameterBuffer<SOn> parameters;
    parameters.PushBack(SOn(4));

    // Check At
    Matrix expected = Matrix::Identity(4,4);
    EXPECT_TRUE(expected.isApprox(parameters.At(0).matrix()));

    // Check that we can access unsafely
    double* unsafe = parameters.Unsafe(0);
    Eigen::Map<Matrix> M(unsafe, 4, 4);
    EXPECT_TRUE(expected.isApprox(M));

    // and change
    expected(0,0) = 2;
    M(0,0) = 2;
    SOn changed = parameters.At(0);
    EXPECT_TRUE(expected.isApprox(changed.matrix()));
}

TEST(ParameterMap, SOn) {
    // Create buffer and add SO(4) object
    ceres::ParameterMap<string> parameters;
    parameters.Insert("key", SOn(4));

    // Check At
    Matrix expected = Matrix::Identity(4,4);
    EXPECT_TRUE(expected.isApprox(parameters.At<SOn>("key").matrix()));

    // Check that we can access unsafely
    double* unsafe = parameters.Unsafe("key");
    Eigen::Map<Matrix> M(unsafe, 4, 4);
    EXPECT_TRUE(expected.isApprox(M));

    // and change
    expected(0,0) = 2;
    M(0,0) = 2;
    SOn changed = parameters.At<SOn>("key");
    EXPECT_TRUE(expected.isApprox(changed.matrix()));
}
