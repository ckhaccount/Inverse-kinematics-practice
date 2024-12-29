/**
 ******************************************************************************
 * @file    chassis.cpp/h
 * @brief   Chassis control. 底盘控制
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef CHASSIS_H
#define CHASSIS_H

#include "base/common/filter.h"
#include "base/common/pid.h"
#include "base/motor/motor.h"

// chassis locomotion status
// 底盘运动状态
typedef struct ChassisStatus {
  float vx = 0;  // x方向速度(m/s)
  float vy = 0;  // y方向速度(m/s)
  float wz = 0;  // 旋转角速度(dps)
  float angle = 0;  // 底盘角度(deg)(yaw电机角度-相对，陀螺仪角度-绝对)

  struct WheelSpeed {  // 车轮转速(dps)
    float fl = 0;
    float fr = 0;
    float bl = 0;
    float br = 0;
  } wheel_speed;
} ChassisStatus_t;

// Mecanum chassis class
// 麦克纳姆轮底盘类
class MecanumChassis {
 public:
  MecanumChassis(Motor* cmfl, Motor* cmfr, Motor* cmbl, Motor* cmbr,
                 PID angle_pid, LowPassFilter speed_filter);

  // Set speed, relative angle, rotate speed feedforward(for follow/twist mode)
  // 设置速度，相对角度和角速度前馈(跟随/扭腰模式)
  // vx+前-后 vy+左-右 angle/wz+逆-顺
  void setAngleSpeed(float vx, float vy, float angle, float ff_wz = 0);

  // Set speed, rotate speed(for chassis only/gyro mode)
  // 设置速度，角速度(纯底盘/陀螺模式)
  // vx+前-后 vy+左-右 wz+逆-顺
  void setSpeed(float vx, float vy, float wz);

  // Set feedback angle (for chassis follow)
  // 设置反馈角度(用于底盘跟随)
  void setFdbAngle(float angle) {}

  // Handle rotate speed
  // 处理底盘旋转速度
  // fdb_angle: 反馈角度，用于底盘跟随等
  void rotateHandle(float fdb_angle);

  // Update feedback and send target status to motor
  // 更新反馈数据，设置电机目标状态
  // fdb_angle: 反馈角度，用于底盘跟随等
  void handle(void);

  // 正运动学，轮速->底盘状态
  void fkine(void);
  // 逆运动学，底盘状态->轮速
  void ikine(void);

 public:
  // 电机指针
  Motor *cmfl_, *cmfr_, *cmbl_, *cmbr_;

  // 底盘模式
  enum ChassisMode_e {
    NORMAL,  // 正常
    FOLLOW,  // 跟随
    GYRO,    // 陀螺
  } mode_;

  bool lock_;  // 底盘锁定

  // 目标状态(机器人坐标系)
  ChassisStatus_t ref_;
  // 反馈状态(机器人坐标系)
  ChassisStatus_t fdb_;
  // 目标状态(底盘坐标系)
  ChassisStatus_t chassis_ref_;
  // 反馈状态(底盘坐标系)
  ChassisStatus_t chassis_fdb_;

  // 底盘角度pid
  PID angle_pid_;
  // 底盘角速度前馈
  float feedforward_wz_;

  // 目标速度滤波(逐渐加速)
  LowPassFilter vx_filter_;
  LowPassFilter vy_filter_;
};

#endif  // CHASSIS_H