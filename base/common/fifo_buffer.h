/**
 ******************************************************************************
 * @file    fifo_buffer.cpp/h
 * @brief   FIFO data buffer. FIFO数据缓冲简单实现
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef FIFO_BUFFER_H
#define FIFO_BUFFER_H

#include <stdint.h>

// Basic FIFO data buffer class
// FIFO数据缓冲类(基础功能实现)
class FIFOBuffer {
 public:
  // @param pdata: 数据储存数组地址
  // @param max_size: 缓冲区容量
  FIFOBuffer(uint8_t* pdata, uint32_t max_size)
      : pdata_(pdata), size_(0), max_size_(max_size) {}

  // Add data to buffer, remove overflow data
  // 添加数据到缓冲区，超出缓冲区容量部分按FIFO形式移除
  void append(uint8_t* data, uint32_t size);

  // Remove data from buffer
  // 从缓冲区开头移除数据
  void remove(uint32_t size);

  // Clear buffer
  // 清空缓冲区数据
  void clear(void);

  // Find data, return first result index / size(can't find)
  // 查找数据，返回首个结果的序号，未找到则返回size
  uint32_t find(uint8_t data);

  uint32_t size(void) { return size_; }
  uint32_t maxSize(void) { return max_size_; }

 private:
  uint8_t* pdata_;
  uint32_t size_;
  uint32_t max_size_;
};

#endif  // FIFO_BUFFER_H
