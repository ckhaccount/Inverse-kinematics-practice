/**
 ******************************************************************************
 * @file    gimbal.cpp/h
 * @brief   Gimbal control. 云台控制
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef GIMBAL_H
#define GIMBAL_H

#include "base/imu/imu.h"
#include "base/motor/motor.h"

// 云台反馈模式(imu/编码器)
typedef enum GimbalFdbMode {
  IMU_MODE,
  ENCODER_MODE,
} GimbalFdbMode_e;

// 云台运动状态
typedef struct GimbalStatus {
  float yaw;
  float pitch;
  float yaw_speed;
  float pitch_speed;
} GimbalStatus_t;

// 云台类
class Gimbal {
 public:
  Gimbal(Motor* gm_yaw, Motor* gm_pitch, IMU* imu);

  // Initialize gimbal, reset init flag
  // 云台初始化，重置回正标记
  void init(void);

  // 云台初始化判断
  bool initFinish(void) {
    return init_status_.yaw_finish && init_status_.pitch_finish;
  }

  // Set gimbal angle and speed
  // 设置云台角度&速度(deg, dps)
  void setAngleSpeed(const float& yaw, const float& pitch,
                     const float& yaw_speed = 0, const float& pitch_speed = 0);

  // Set gimbal angle
  // 设置云台角度(deg)
  void setAngle(const float& yaw, const float& pitch);

  // Add gimbal angle
  // 设置云台角度增量(deg)
  void addAngle(const float& d_yaw, const float& d_pitch);

  // Set gimbal mode(imu)
  // 设置云台模式(imu反馈或编码器反馈)
  void setMode(GimbalFdbMode_e mode);

  // Send target status to motor and update feedback
  // 设置电机目标状态，更新反馈数据
  void handle(void);

 private:
  // pitch轴力矩补偿(作为力矩前馈输入)
  float pitchCompensate(const float& pitch);

 public:
  Motor *gm_yaw_, *gm_pitch_;  // 电机指针
  IMU* imu_;                   // imu指针

  // 目标状态数据
  GimbalStatus_t ref_;
  // 反馈状态数据
  GimbalStatus_t fdb_;
  // 控制模式
  GimbalFdbMode_e mode_;
  // pitch轴补偿
  float pitch_compensate_;
  // 角度限位(deg)
  struct GimbalAngleLimit_t {
    float yaw_min;
    float yaw_max;
    float pitch_min;
    float pitch_max;
  } limit_;

  // 上电按编码器初始化相关参数
  struct InitStatus_t {
    bool yaw_finish;
    bool pitch_finish;
  } init_status_;
};

extern const float yaw_zero_ecd, pitch_zero_ecd;

#endif  // GIMBAL_H