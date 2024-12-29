/**
 ******************************************************************************
 * @file    servo.cpp/h
 * @brief   Servo driver. 舵机驱动
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef SERVO_H
#define SERVO_H

#include "tim.h"
#include "usart.h"

// PWM舵机
class ServoPwm {
 public:
  // config pwm channel
  ServoPwm(TIM_HandleTypeDef* htim, uint32_t tim_channel);

  // init pwm channel
  void init(void);
  // set pwm to control angle
  void setPwm(const uint16_t& pwm);

 private:
  TIM_HandleTypeDef* htim_;
  uint32_t tim_channel_;
  uint16_t pwm_;
};

// 串口舵机
class ServoZX361D {
 public:
  typedef enum Mode {
    NORMAL,
    COMPLIANCE,
  } Mode_e;

 public:
  // config uart port
  ServoZX361D(UART_HandleTypeDef* huart = nullptr);

  // init servo, set mode to passive
  // 初始化舵机，设置为被动模式
  void init(void);
  // set mode(normal/passive)
  // 设置舵机模式(主动控制/被动模式-可手扳动舵机旋转)
  bool setMode(const ServoZX361D::Mode_e& mode);
  // set pwm and time(ms)
  // 设置舵机pwm和时间(单位ms)
  bool setPwmTime(const uint16_t& pwm, const uint16_t& time_ms);

  ServoZX361D::Mode_e getMode(void) { return mode_; }

 private:
  UART_HandleTypeDef* huart_;
  ServoZX361D::Mode_e mode_;
  uint16_t pwm_;
  uint16_t time_ms_;
  struct Tx_t {
    uint8_t buf[32];
    uint8_t len;
  } tx_;
  uint32_t last_tx_tick_;
};

#endif  // SERVO_H