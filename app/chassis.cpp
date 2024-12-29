/**
 ******************************************************************************
 * @file    chassis.cpp/h
 * @brief   Chassis control. 底盘控制
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/chassis.h"
#include "base/common/math.h"
#include "base/common/pid.h"
#include "lib/arm_math/arm_math.h"
#include <cmath>
// 底盘物理参数
const float wheel_radius = 0.0763f;   // 轮半径(m)
const float half_track_width = 0.2f;  // 1/2轴距(m)
const float half_wheel_base = 0.19f;  // 1/2轮距(m)
const float x_bias = 0;  // x方向(底盘前方为正方向)偏移(m)
const float y_bias = 0;  // y方向(底盘左方为正方向)偏移(m)

MecanumChassis::MecanumChassis(Motor* cmfl, Motor* cmfr, Motor* cmbl,
                               Motor* cmbr, PID angle_pid,
                               LowPassFilter speed_filter)
    : cmfl_(cmfl),
      cmfr_(cmfr),
      cmbl_(cmbl),
      cmbr_(cmbr),
      mode_(NORMAL),
      lock_(false),
      angle_pid_(angle_pid),
      vx_filter_(speed_filter),
      vy_filter_(speed_filter) {}

// Set speed, relative angle, rotate speed feedforward(for follow/twist mode)
// 设置速度，相对角度和角速度前馈(跟随/扭腰模式)
// vx+前-后 vy+左-右 angle/wz+逆-顺
void MecanumChassis::setAngleSpeed(float vx, float vy, float angle,
                                   float ff_wz) {
  ref_.vx = vx_filter_.update(vx);
  ref_.vy = vy_filter_.update(vy);
  ref_.angle = angle;
  feedforward_wz_ = ff_wz;
}

// Set speed, rotate speed(for chassis only/gyro mode)
// 设置速度，角速度(纯底盘/陀螺模式)
// vx+前-后 vy+左-右 wz+逆-顺
void MecanumChassis::setSpeed(float vx, float vy, float wz) {
  ref_.vx = vx_filter_.update(vx);
  ref_.vy = vy_filter_.update(vy);
  ref_.wz = wz;
}

// Handle rotate speed
// 处理不同模式下底盘旋转速度
// fdb_angle: 反馈角度，用于底盘跟随等
void MecanumChassis::rotateHandle(float fdb_angle) {
  fdb_.angle = fdb_angle;
  if (mode_ == NORMAL) {
  } else if (mode_ == FOLLOW) {
    ref_.wz = math::deadBand(angle_pid_.calc(ref_.angle, fdb_.angle), -10, 10) +
              feedforward_wz_;
  } else if (mode_ == GYRO) {
  }
}

// Update feedback and send target status to motor
// 更新反馈数据，设置电机目标状态
void MecanumChassis::handle(void) {
  // 更新反馈数据
  fdb_.wheel_speed.fl = cmfl_->realSpeed();
  fdb_.wheel_speed.fr = cmfr_->realSpeed();
  fdb_.wheel_speed.bl = cmbl_->realSpeed();
  fdb_.wheel_speed.br = cmbr_->realSpeed();
  chassis_fdb_.wheel_speed = fdb_.wheel_speed;

  // 正运动学解算当前底盘反馈状态
  fkine();

  // 逆运动学解算目标轮速
  ikine();

  // 设置电机速度
  if (!lock_) {
    cmfl_->setSpeed(ref_.wheel_speed.fl);
    cmfr_->setSpeed(ref_.wheel_speed.fr);
    cmbl_->setSpeed(ref_.wheel_speed.bl);
    cmbr_->setSpeed(ref_.wheel_speed.br);
  } else {
    cmfl_->setSpeed(0);
    cmfr_->setSpeed(0);
    cmbl_->setSpeed(0);
    cmbr_->setSpeed(0);
  }
}

// 正运动学，轮速->底盘状态
void MecanumChassis::fkine(void) {
  // 电机转速->底盘坐标系反馈状态
  float rv_fl = fdb_.wheel_speed.fl;
  float rv_fr = fdb_.wheel_speed.fr;
  float rv_bl = fdb_.wheel_speed.bl;
  float rv_br = fdb_.wheel_speed.br;
  chassis_fdb_.vx =
      0.25f * math::dps2radps(rv_fl - rv_fr + rv_bl - rv_br) * wheel_radius;
  chassis_fdb_.vy =
      0.25f * math::dps2radps(-rv_fl - rv_fr + rv_bl + rv_br) * wheel_radius;
  chassis_fdb_.wz = 0.25f * (-rv_fl - rv_fr - rv_bl - rv_br) * wheel_radius /
                    (half_track_width + half_wheel_base);
  // 底盘坐标系反馈状态->机器人坐标系反馈状态
  fdb_.vx = (chassis_fdb_.vx - math::dps2radps(chassis_fdb_.wz) * y_bias) *
                cosf(math::deg2rad(fdb_.angle)) -
            (chassis_fdb_.vy - math::dps2radps(chassis_fdb_.wz) * x_bias) *
                sinf(math::deg2rad(fdb_.angle));
  fdb_.vy = (chassis_fdb_.vx - math::dps2radps(chassis_fdb_.wz) * y_bias) *
                sinf(math::deg2rad(fdb_.angle)) +
            (chassis_fdb_.vy - math::dps2radps(chassis_fdb_.wz) * x_bias) *
                cosf(math::deg2rad(fdb_.angle));
  fdb_.wz = chassis_fdb_.wz;
}

// 逆运动学，底盘状态->轮速
void MecanumChassis::ikine(void) {
  // 机器人坐标系目标状态->底盘坐标系目标状态
  chassis_ref_.vx = ref_.vx * cosf(math::deg2rad(fdb_.angle)) +
                    ref_.vy * sinf(math::deg2rad(fdb_.angle)) +
                    math::dps2radps(ref_.wz) * y_bias;
  chassis_ref_.vy = -ref_.vx * sinf(math::deg2rad(fdb_.angle)) +
                    ref_.vy * cosf(math::deg2rad(fdb_.angle)) -
                    math::dps2radps(ref_.wz) * x_bias;
  chassis_ref_.wz = ref_.wz;
  // 底盘坐标系目标状态->电机转速
  float vx = chassis_ref_.vx;
  float vy = chassis_ref_.vy;
  float wv =
      math::dps2radps(chassis_ref_.wz) * (half_track_width + half_wheel_base);
  chassis_ref_.wheel_speed.fl = math::radps2dps((-wv + vx - vy) / wheel_radius);
  chassis_ref_.wheel_speed.fr = math::radps2dps((-wv - vx - vy) / wheel_radius);
  chassis_ref_.wheel_speed.bl = math::radps2dps((-wv + vx + vy) / wheel_radius);
  chassis_ref_.wheel_speed.br = math::radps2dps((-wv - vx + vy) / wheel_radius);
  ref_.wheel_speed = chassis_ref_.wheel_speed;
}
