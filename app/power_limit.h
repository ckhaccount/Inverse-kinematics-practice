/**
 ******************************************************************************
 * @file    power_limit.cpp/h
 * @brief   Power limitation. 功率限制
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef POWER_LIMIT_H
#define POWER_LIMIT_H

#include "app/chassis.h"
#include "base/cap_comm/cap_comm.h"
#include "base/common/filter.h"
#include "base/referee_comm/referee_comm.h"

namespace powerlimit {
// 电机功率计算
float motorPower(const Motor::Type_e& type, const float& i, const float& w);
}  // namespace powerlimit

class MecanumChassisPower {
 public:
  // 电机功率
  typedef struct MotorPower {
    float fl, fr, bl, br;
  } MotorPower_t;

 public:
  MecanumChassisPower(MecanumChassis* chassis, RefereeComm* referee,
                      CapComm* cap);

  // 功率限制初始化
  void init(void);
  // 功率限制处理
  void handle(float extra_power_max = 0);

 private:
  // 功率限制方程求解，返回求解功率限制方程得到的实际功率限制
  void solve(const float& p_ref);

 public:
  MecanumChassis* chassis_;
  RefereeComm* referee_;
  CapComm* ultra_cap_;

  // 程序设置功率上限
  float ref_power_limit_;

  // 减速比例
  float speed_rate_;
  LowPassFilter speed_rate_filter_;

  // 电机功率反馈（估计值）
  MotorPower_t motor_power_fdb_;
  // 底盘功率反馈（估计值）
  float chassis_power_fdb_;
  LowPassFilter chassis_power_fdb_filter_;

  // 电机预估功率
  MotorPower_t motor_power_est_;
  // 底盘预估功率
  float chassis_power_est_;

  // 功率方程
  struct PowerLimitEquation {
    float kp, rm;   // kp-速度环，rm-减速比
    float k[4];     // 电机功率相关常数
    float a, b, c;  // 方程系数
    float delta;    // Δ
    float r;        // 速度比例
  } eq_;
};

#endif  // POWER_LIMIT_H