//===----------------------------------------------------------------------===//
//
//                         BusTub
//
// lru_replacer.cpp
//
// Identification: src/buffer/lru_replacer.cpp
//
// Copyright (c) 2015-2019, Carnegie Mellon University Database Group
//
//===----------------------------------------------------------------------===//

#include "buffer/lru_replacer.h"

namespace bustub {

LRUReplacer::LRUReplacer(size_t num_pages) : capacity_{num_pages} {}

LRUReplacer::~LRUReplacer() = default;

auto LRUReplacer::Victim(frame_id_t *frame_id) -> bool {

  std::lock_guard<std::mutex> guard(lru_replacer_mutex_);

  if (frame_list_.empty()) {
    return false;
  }

  auto front = frame_list_.front();
  frame_list_.pop_front();
  auto find = frame_id_to_iter_map_.find(front);
  if (find != frame_id_to_iter_map_.end()) {
    *frame_id = find->first;
    frame_id_to_iter_map_.erase(find);
    return true;
  }
  return false;
}

void LRUReplacer::Pin(frame_id_t frame_id) {
  std::lock_guard<std::mutex> guard(lru_replacer_mutex_);

  auto find = frame_id_to_iter_map_.find(frame_id);
  if (find != frame_id_to_iter_map_.end()) {
    frame_list_.erase(find->second);
    frame_id_to_iter_map_.erase(find);
  }
}

void LRUReplacer::Unpin(frame_id_t frame_id) {
  std::lock_guard<std::mutex> guard(lru_replacer_mutex_);
  if (frame_list_.size() >= capacity_) {
    return;
  }

  auto find = frame_id_to_iter_map_.find(frame_id);
  if (find != frame_id_to_iter_map_.end()) {
    return;
  }

  frame_list_.push_back(frame_id);
  frame_id_to_iter_map_.emplace(frame_id, std::prev(frame_list_.end(), 1));
}

auto LRUReplacer::Size() -> size_t {
  std::lock_guard<std::mutex> guard(lru_replacer_mutex_);

  return frame_list_.size();
}
}  // namespace bustub
