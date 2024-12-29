/**
 ******************************************************************************
 * @file    servo.cpp/h
 * @brief   Servo driver. 舵机驱动
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "base/servo/servo.h"
#include <stdio.h>
#include <string.h>
#include "base/common/math.h"

ServoPwm::ServoPwm(TIM_HandleTypeDef* htim, uint32_t tim_channel)
    : htim_(htim), tim_channel_(tim_channel) {}

// init PWM channel
void ServoPwm::init(void) {
  HAL_TIM_PWM_Start(htim_, tim_channel_);
}

// set PMW to control angle
void ServoPwm::setPwm(const uint16_t& pwm) {
  pwm_ = pwm;
  __HAL_TIM_SetCompare(htim_, tim_channel_, pwm_);
}

// config uart port
ServoZX361D::ServoZX361D(UART_HandleTypeDef* huart) : huart_(huart) {}

// init servo, set mode to passive
// 初始化舵机，设置为被动模式
void ServoZX361D::init(void) {
  mode_ = ServoZX361D::COMPLIANCE;
  tx_.len = sizeof("#000PULK!");
  memcpy(tx_.buf, (uint8_t*)"#000PULK!", tx_.len);
  if (huart_ != nullptr) {
    HAL_UART_Transmit_IT(huart_, tx_.buf, tx_.len);
  }
}

// set mode(normal/passive)
// 设置舵机模式（主动控制/被动模式-可手扳动舵机旋转）
bool ServoZX361D::setMode(const ServoZX361D::Mode_e& mode) {
  if (mode_ != mode && HAL_GetTick() - last_tx_tick_ > 100) {
    mode_ = mode;
    if (mode_ == ServoZX361D::NORMAL) {
      tx_.len = sprintf((char*)tx_.buf, "#000PULR!");
      if (huart_ != nullptr) {
        HAL_UART_Transmit_IT(huart_, tx_.buf, tx_.len);
      }
    } else if (mode_ == ServoZX361D::COMPLIANCE) {
      tx_.len = sprintf((char*)tx_.buf, "#000PULK!");
      if (huart_ != nullptr) {
        HAL_UART_Transmit_IT(huart_, tx_.buf, tx_.len);
      }
    }
    last_tx_tick_ = HAL_GetTick();
    return true;
  }
  return false;
}

// set pwm and time(ms)
// 设置舵机pwm和时间(单位ms)
bool ServoZX361D::setPwmTime(const uint16_t& pwm, const uint16_t& time_ms) {
  pwm_ = math::limit(pwm, 500, 2500);
  time_ms_ = math::limit(time_ms, 0, 9999);

  if (HAL_GetTick() - last_tx_tick_ > 10) {
    tx_.len = sprintf((char*)tx_.buf, "#000P");
    tx_.len += sprintf((char*)tx_.buf + tx_.len, "%.4d", pwm_);
    tx_.len += sprintf((char*)tx_.buf + tx_.len, "T");
    tx_.len += sprintf((char*)tx_.buf + tx_.len, "%.4d", time_ms_);
    tx_.len += sprintf((char*)tx_.buf + tx_.len, "!");
    if (huart_ != nullptr) {
      HAL_UART_Transmit_IT(huart_, tx_.buf, tx_.len);
    }
    last_tx_tick_ = HAL_GetTick();
    return true;
  }
  return false;
}
