/**
 ******************************************************************************
 * @file    lqr.cpp/h
 * @brief   LQR algorithm. LQR控制算法实现
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "lqr.h"

// Initialize vectors/matrices, allocate storage space, set initial
// vector/constant matrices 初始化矩阵/向量，分配内存，设置初值
LQR::LQR(uint16_t x_size, uint16_t u_size) : x_size_(x_size), u_size_(u_size) {
  data_.xd = new float[x_size_];
  MatrixInit(&mat_.xd, x_size_, 1, (float*)data_.xd);
  data_.x = new float[x_size_];
  MatrixInit(&mat_.x, x_size_, 1, (float*)data_.x);
  data_.u = new float[u_size_];
  MatrixInit(&mat_.u, u_size_, 1, (float*)data_.u);
  data_.K = new float[u_size_ * x_size_];
  MatrixInit(&mat_.K, u_size_, x_size_, (float*)data_.K);
  data_.tmp = new float[x_size_];
  MatrixInit(&mat_.tmp, x_size_, 1, (float*)data_.tmp);
}

// Free storage ptr. 释放内存
LQR::~LQR() {
  delete[] data_.xd;
  data_.xd = nullptr;
  delete[] data_.x;
  data_.x = nullptr;
  delete[] data_.u;
  data_.u = nullptr;
  delete[] data_.K;
  data_.K = nullptr;
  delete[] data_.tmp;
  data_.tmp = nullptr;
}

// Control calculate. LQR控制律计算
void LQR::calc(float* xd, float* x, float* u, float* K, uint16_t x_size, uint16_t u_size) {
  if (x_size != x_size_ || u_size != u_size_) {
    return;
  }
  memcpy(data_.xd, xd, x_size_ * sizeof(float));
  memcpy(data_.x, x, x_size_ * sizeof(float));
  memcpy(data_.u, u, u_size_ * sizeof(float));
  memcpy(data_.K, K, u_size_ * x_size_ * sizeof(float));
  mat_status_ = MatrixSub(&mat_.xd, &mat_.x, &mat_.tmp);
  mat_status_ = MatrixMult(&mat_.K, &mat_.tmp, &mat_.u);
  memcpy(u, data_.u, u_size_ * sizeof(float));
}
