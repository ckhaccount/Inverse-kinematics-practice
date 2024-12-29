/**
 ******************************************************************************
 * @file    gimbal.cpp/h
 * @brief   Gimbal control. 云台控制
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/gimbal.h"
#include "base/common/math.h"
#include "lib/arm_math/arm_math.h"
#include <cmath>
// Encoder value of zero point. 零点编码器值
const float yaw_zero_ecd = math::ecd2deg(995, 8192);
const float pitch_zero_ecd = math::ecd2deg(6400, 8192);

// Speed limit for initialization(dps). 云台初始化限速(dps)
const float gimbal_init_speed_max = 180;
const float gimbal_speed_max = 1800;
// Gimbal init finish threshold. 云台初始化完成角度阈值
const float gimbal_init_angle_thres = 5;

// Gimbal angle limit 云台角度限位
// const float yaw_min = -60;
// const float yaw_max = 60;
const float pitch_min = -40;
const float pitch_max = 25;

Gimbal::Gimbal(Motor* gm_yaw, Motor* gm_pitch, IMU* imu)
    : gm_yaw_(gm_yaw), gm_pitch_(gm_pitch), imu_(imu) {
  init_status_.yaw_finish = false;
  init_status_.pitch_finish = false;
  // limit_.yaw_min = yaw_min;
  // limit_.yaw_max = yaw_max;
  limit_.pitch_min = pitch_min;
  limit_.pitch_max = pitch_max;
  setMode(ENCODER_MODE);
}

// Initialize gimbal, reset init flag
// 云台初始化，重置回正标记
void Gimbal::init(void) {
  init_status_.yaw_finish = false;
  init_status_.pitch_finish = false;

  // 初始化过程中设置目标角度为0，反馈角度为编码器单圈角度
  ref_.yaw = 0;
  ref_.pitch = 0;
  gm_yaw_->motor_data_.angle = math::degNormalize180(
      (gm_yaw_->motor_data_.ecd_angle - yaw_zero_ecd) / gm_yaw_->ratio_);
  gm_yaw_->resetFeedbackAngle(gm_yaw_->motor_data_.angle);
  gm_pitch_->motor_data_.angle = math::degNormalize180(
      (gm_pitch_->motor_data_.ecd_angle - pitch_zero_ecd) / gm_pitch_->ratio_);
  gm_pitch_->resetFeedbackAngle(gm_pitch_->motor_data_.angle);
}

// Set gimbal angle and speed
// 设置云台角度&速度(deg, dps)
void Gimbal::setAngleSpeed(const float& yaw, const float& pitch,
                           const float& yaw_speed, const float& pitch_speed) {
  // 设置目标状态
  ref_.yaw = yaw;
  ref_.pitch = math::limit(pitch,
                           gm_pitch_->control_data_.fdb_angle -
                               gm_pitch_->motor_data_.angle + limit_.pitch_min,
                           gm_pitch_->control_data_.fdb_angle -
                               gm_pitch_->motor_data_.angle + limit_.pitch_max);
  ref_.yaw_speed = yaw_speed;
  ref_.pitch_speed = pitch_speed;
}

// Set gimbal angle
// 设置云台角度(deg)
void Gimbal::setAngle(const float& yaw, const float& pitch) {
  setAngleSpeed(yaw, pitch, 0, 0);
}

// Add gimbal angle
// 设置云台角度增量(deg)
void Gimbal::addAngle(const float& d_yaw, const float& d_pitch) {
  // 设置目标状态
  ref_.yaw += d_yaw;
  ref_.pitch = math::limit(ref_.pitch + d_pitch,
                           gm_pitch_->control_data_.fdb_angle -
                               gm_pitch_->motor_data_.angle + limit_.pitch_min,
                           gm_pitch_->control_data_.fdb_angle -
                               gm_pitch_->motor_data_.angle + limit_.pitch_max);
}

// Set gimbal mode(imu)
// 设置云台模式(imu反馈或编码器反馈)
void Gimbal::setMode(GimbalFdbMode_e mode) {
  if (mode_ != mode && mode == IMU_MODE) {
    gm_yaw_->setFdbSrc(&imu_->yaw(), &imu_->wzWorld());
    gm_pitch_->setFdbSrc(&imu_->pitch(), &imu_->wySensor());
  } else if (mode_ != mode && mode == ENCODER_MODE) {
    gm_yaw_->setFdbSrc(&gm_yaw_->motor_data_.angle, &imu_->wzWorld());
    gm_pitch_->setFdbSrc(&gm_pitch_->motor_data_.angle, &imu_->wySensor());
  }
  mode_ = mode;
}

// Send target status to motor and update feedback
// 设置电机目标状态，更新反馈数据
void Gimbal::handle(void) {
  // 云台初始化慢速回正
  if (!init_status_.yaw_finish) {  // yaw初始化未完成
    // 判断复位是否完成
    init_status_.yaw_finish =
        gm_yaw_->connect_.check() &&
        (fabs(gm_yaw_->motor_data_.angle) < gimbal_init_angle_thres);
    if (init_status_.yaw_finish) {
      // 检测到初始化完成，速度限制恢复正常
      gm_yaw_->ppid_.out_max_ = gimbal_speed_max;
    } else {
      // 设置初始化速度限制
      gm_yaw_->ppid_.out_max_ = gimbal_init_speed_max;
    }
  }
  if (!init_status_.pitch_finish) {  // pitch初始化未完成
    // 判断复位是否完成
    init_status_.pitch_finish =
        gm_pitch_->connect_.check() &&
        (fabs(gm_pitch_->motor_data_.angle) < gimbal_init_angle_thres);
    if (init_status_.pitch_finish) {
      // 检测到初始化完成，速度限制恢复正常
      gm_pitch_->ppid_.out_max_ = gimbal_speed_max;
    } else {
      // 设置初始化速度限制
      gm_pitch_->ppid_.out_max_ = gimbal_init_speed_max;
    }
  }

  // 云台电机离线处理
  if (!gm_yaw_->connect_.check() && !gm_pitch_->connect_.check()) {
    init();
  }

  // 读取电机控制反馈
  fdb_.yaw = gm_yaw_->control_data_.fdb_angle;
  fdb_.pitch = gm_pitch_->control_data_.fdb_angle;
  fdb_.yaw_speed = gm_yaw_->control_data_.fdb_speed;
  fdb_.pitch_speed = gm_pitch_->control_data_.fdb_speed;

  // 设置电机控制目标角度 & 角速度前馈 & 力矩前馈(补偿)
  gm_yaw_->setAngleSpeed(ref_.yaw, ref_.yaw_speed);
  gm_pitch_->setAngleSpeed(ref_.pitch, ref_.pitch_speed,
                           pitchCompensate(fdb_.pitch));
}

// pitch轴力矩补偿(作为力矩前馈输入)
float Gimbal::pitchCompensate(const float& pitch) {
  pitch_compensate_ = 0;
  return pitch_compensate_;
}
