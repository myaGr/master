/**
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_STATIC_BUFFER_H
#define SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_STATIC_BUFFER_H

#include <array>

namespace swift {
namespace internal {

template <typename Type, std::size_t Capacity>
class StaticBuffer {
 public:
  /**
   * @brief Returns number of items in the collection, between [0, Capacity].
   */
  std::size_t size() const { return size_; }

  /**
   * @brief Place an item at the end of the container.
   * If it is already full, the first item is dropped.
   */
  void insert(const Type &item) {
    const std::size_t index = (oldest_item_idx_ + size_) % Capacity;
    array_[index] = item;
    if (size_ < Capacity) {
      ++size_;
    } else {
      ++oldest_item_idx_ %= Capacity;
    }
  };

  /**
   * @brief Access item, between index [0, size-1].
   * Requested index 0 returns the oldest item in the container by internally
   * shift the index according to the value of `oldest_item_idx_`.
   */
  Type &operator[](const std::size_t requested_index) {
    const std::size_t index = (oldest_item_idx_ + requested_index) % Capacity;
    return array_[index];
  };

  /**
   * @brief Access item, between index [0, size-1].
   * Requested index 0 returns the oldest item in the container by internally
   * shift the index according to the value of `oldest_item_idx_`.
   */
  const Type &operator[](const std::size_t requested_index) const {
    const std::size_t index = (oldest_item_idx_ + requested_index) % Capacity;
    return array_[index];
  };

  /**
   * @brief Remove all items from the container.
   */
  void clear() {
    size_ = 0;
    oldest_item_idx_ = 0;
  }

 private:
  std::array<Type, Capacity> array_;
  std::size_t size_ = 0;
  std::size_t oldest_item_idx_ = 0;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_STATIC_BUFFER_H
