/**
 ******************************************************************************
 * @file    autoaim.cpp/h
 * @brief   Autoaim gimbal & shoot control. 自瞄云台&发射控制
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef AUTOAIM_H
#define AUTOAIM_H

#include <stdint.h>

#define GIMBAL_ANGLE_RECORD_SIZE 64

class Autoaim {
 public:
  Autoaim(void) {}

  // 自瞄数据处理，电机控制
  void handle(void);

  // 自瞄状态设置
  void setState(bool aim_state, bool shoot_state);

 public:
  // 自瞄开关
  bool aim_state_;
  // 自动射击开关
  bool shoot_state_;
  // 自瞄数据
  struct AutoaimStatus_t {
    float relative_yaw;    // yaw相对角度(deg)
    float relative_pitch;  // pitch相对角度(deg)
    float dist;            // 距离(m)

    float absolute_yaw_ref;    // yaw绝对角度目标(deg)
    float absolute_pitch_ref;  // pitch绝对角度目标(deg)
    float yaw_speed_ref;       // yaw目标速度(dps)
    float pitch_speed_ref;     // pitch目标速度(dps)

    float absolute_yaw_fdb;    // yaw绝对角度反馈
    float absolute_pitch_fdb;  // pitch绝对角度反馈
    float yaw_speed_fdb;       // yaw速度反馈
    float pitch_speed_fdb;     // pitch速度反馈

    bool shoot_flag;
    bool idle_flag;
    uint8_t enemy_id;  // 瞄准目标id
  } status_;
  // 角度补偿
  struct AutoaimOffset_t {
    float yaw;
    float pitch;
    float energy_yaw;
    float energy_pitch;
  } offset_;
  // 云台角度记录
  struct GimbalAngleRecord_t {
    float yaw;
    float pitch;
  } gimbal_angle_rcd[GIMBAL_ANGLE_RECORD_SIZE];
};

#endif  // AUTOAIM_H