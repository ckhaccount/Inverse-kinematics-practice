/**
 ******************************************************************************
 * @file    fifo_buffer.cpp/h
 * @brief   FIFO data buffer. FIFO数据缓冲
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "fifo_buffer.h"
#include <string.h>

// Add data to buffer, remove overflow data
// 添加数据到缓冲区，超出缓冲区容量部分按FIFO形式移除
void FIFOBuffer::append(uint8_t* data, uint32_t size) {
  if (size_ + size <= max_size_) {
    // 添加后缓冲区数据量 <= 缓冲区容量
    memcpy(pdata_ + size_, data, size);
    size_ += size;
  } else if (size < max_size_) {
    // 添加后缓冲区数据量 > 缓冲区容量 && data大小 < 缓冲区容量
    memmove(pdata_, pdata_ + size_ + size - max_size_, max_size_ - size);
    memcpy(pdata_ + max_size_ - size, data, size);
    size_ = max_size_;
  } else {
    // data大小 >= 缓冲区容量
    memcpy(pdata_, data + size - max_size_, max_size_);
    size_ = max_size_;
  }
}

// Remove data from buffer
// 从缓冲区开头移除数据
void FIFOBuffer::remove(uint32_t size) {
  if (size_ > size) {
    // size < 当前缓冲区数据量
    memmove(pdata_, pdata_ + size, size_ - size);
    size_ -= size;
  } else {
    // size >= 当前缓冲区数据量
    size_ = 0;
  }
}

// Clear buffer
// 清空缓冲区数据
void FIFOBuffer::clear(void) {
  size_ = 0;
}

// Find data, return first result index / size(can't find)
// 查找数据，返回首个结果的序号，未找到则返回size
uint32_t FIFOBuffer::find(uint8_t data) {
  for (uint8_t* ptr = pdata_; ptr < pdata_ + size_; ptr++) {
    if (*ptr == data) {
      return ptr - pdata_;
    }
  }
  return size_;
}
