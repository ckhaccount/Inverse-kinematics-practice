/**
 ******************************************************************************
 * @file    lqr.cpp/h
 * @brief   LQR algorithm. LQR控制算法实现
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef LQR_H
#define LQR_H

#include "matrix.h"

class LQR {
 public:
  // Initialize vectors/matrices, allocate storage space, set initial
  // vector/constant matrices 初始化矩阵/向量，分配内存，设置初值
  LQR(uint16_t x_size, uint16_t u_size);

  // Free storage ptr. 释放内存
  ~LQR();

  // Control calculate. 状态反馈控制计算
  void calc(float* xd, float* x, float* u, float* K, uint16_t x_size, uint16_t u_size);

 private:
  // vector size
  uint16_t x_size_;
  uint16_t u_size_;
  // ARM matrix
  struct {
    Matrix xd, x;  // state vector x
    Matrix u;      // control vector u=K(xd-x)
    Matrix K;      // state feedback matrix K
    Matrix tmp;    // temporary matrix (xd-x)
  } mat_;
  // Matrix data storage ptr
  struct {
    float *xd, *x;
    float* u;
    float* K;
    float* tmp;
  } data_;
  arm_status mat_status_;
};

#endif  // LQR_H