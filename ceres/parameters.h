/**
 * @file parameters.h
 * @brief Present a type-checked interface to parameters in Ceres.
 * @author Frank Dellaert
 * @date August 2020
 */

#pragma once

#include "traits.h"

#include <iostream>

namespace ceres {

/**
 * Interface to present a type-checked interface to parameters in Ceres
 */
template <typename Key = size_t> class Parameters {

public:
  // Return unsafe pointer to block with given key
  virtual double *Unsafe(Key index) = 0;

  // Fill an array with unsafe pointers to blocks at given keys
  virtual std::vector<double *> Unsafe(std::vector<Key> indices) = 0;
};

/**
 * Version that uses contiguous memory for an array of T values
 *
 * Required traits for Parameters:
 *
 *      // Get unsafe pointer:
 *      static double* Unsafe(const T &t);
 */
template <typename T> class ParameterBuffer : public Parameters<size_t> {
  std::vector<T> array_;

public:
  ParameterBuffer() {}

  ParameterBuffer(size_t size):array_(size) {}

  void Reserve(size_t size) {
    array_.reserve(size);
  }

  void PushBack(const T &t) {
    array_.push_back(t);
  }

  // Return T value from block with given key
  T At(size_t index) {
    return array_[index];
  }

  // Return unsafe pointer to block with given key
  double *Unsafe(size_t index) override { return traits<T>::Unsafe(array_[index]); }

  // Fill an array with unsafe pointers to blocks at given keys
  std::vector<double *> Unsafe(std::vector<size_t> indices) override {
    std::vector<double *> result;
    result.reserve(indices.size());
    for (size_t i : indices) {
      result.push_back(this->Unsafe(i));
    }
    return result;
  }
};

/**
 * Version that uses a map with arbitrary keys
 *
 * Required traits:
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
template <typename Key = size_t> class ParameterMap : public Parameters<Key> {
  using Block = std::vector<double>;
  using Blocks = std::map<Key, Block>;
  std::map<Key, Block> blocks_;

public:
  ParameterMap() {}

  // Allocate and initialize a block at given key
  template <typename T> bool Insert(Key key, const T &t) {
    typename Blocks::iterator it;
    bool success;
    size_t n = traits<T>::AmbientDim(t);
    std::tie(it, success) = blocks_.emplace(key, Block(n));
    if (success) {
      traits<T>::Vec(t, it->second.data());
    }
    return success;
  }

  // Return T value from block with given key
  template <typename T> T At(Key key) {
    const Block &block = blocks_[key];
    return traits<T>::Unvec(block.size(), block.data());
  }

  // Return unsafe pointer to block with given key
  double *Unsafe(Key key) override { return blocks_[key].data(); }

  // Fill an array with unsafe pointers to blocks at given keys
  std::vector<double *> Unsafe(std::vector<Key> keys) override {
    std::vector<double *> result;
    result.reserve(keys.size());
    for (Key key : keys) {
      result.push_back(this->Unsafe(key));
    }
    return result;
  }
};

} // namespace ceres
