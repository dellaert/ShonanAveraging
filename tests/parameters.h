/**
 * @file parameters.h
 * @brief Present a type-checked interface to parameters in Ceres.
 * @author Frank Dellaert
 * @date August 2020
 */

#pragma once

#include "traits.h"

namespace ceres {

/**
 * Present a type-checked interface to parameters in Ceres
 */
template <typename Key = size_t> class Parameters {
  using Block = std::vector<double>;
  using Blocks = std::map<Key, Block>;
  std::map<Key, Block> blocks_;

public:
  Parameters() {}

  // Allocate and initialize a block at given key
  template <typename T> bool Insert(Key key, const T &t) {
    typename Blocks::iterator it;
    bool success;
    std::tie(it, success) =
        blocks_.emplace(key, Block(traits<T>::AmbientDim(t)));
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
  double *Unsafe(Key key) { return blocks_[key].data(); }

  // Fill an array with unsafe pointers to blocks at given keys
  std::vector<double *> Unsafe(std::vector<Key> keys) {
    std::vector<double *> result;
    result.reserve(keys.size());
    for (Key key : keys) {
      result.push_back(this->Unsafe(key));
    }
    return result;
  }
};

} // namespace ceres
