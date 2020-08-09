/**
 * @file traits.h
 * @brief Traits to have a type-checked interface to parameters in Ceres.
 * @author Frank Dellaert
 * @date August 2020
 */

#pragma once

namespace ceres {

/**
 * Traits used in presenting a type-checked API to Ceres
 *
 * Required traits for Parameters:
 *
 *      // Ambient dimension:
 *      static size_t AmbientDim(const T &t);
 *
 *      // Vectorize to doubles in pre-allocated block:
 *      static void Vec(const T &t, double *const block);
 *
 *      // Initialize from vectorized storage:
 *      static T Unvec(size_t n, const double *const block);
 */
template <typename T> struct traits {};

} // namespace ceres
