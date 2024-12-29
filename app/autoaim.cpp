/**
 ******************************************************************************
 * @file    autoaim.cpp/h
 * @brief   Autoaim gimbal & shoot control. 自瞄云台&发射控制
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/autoaim.h"
#include "app/chassis.h"
#include "app/gimbal.h"
#include "app/shoot.h"
#include "base/cv_comm/cv_comm.h"
#include "base/referee_comm/referee_comm.h"

// extern MecanumChassis chassis;
extern Gimbal gimbal;
extern Shoot shoot;
extern CVComm cv_comm;
extern RefereeComm referee;

const uint32_t gimbal_cv_delay = 22;  // ms，云台角度反馈与视觉指令的延时

// 自瞄数据处理，电机控制
void Autoaim::handle(void) {
  // 更新反馈数据
  status_.absolute_yaw_fdb = gimbal.fdb_.yaw;         // GMY.realAngle()
  status_.absolute_pitch_fdb = gimbal.fdb_.pitch;     // GMP.realAngle()
  status_.yaw_speed_fdb = gimbal.fdb_.yaw_speed;      // GMY.realSpeed()
  status_.pitch_speed_fdb = gimbal.fdb_.pitch_speed;  // GMP.realSpeed()

  // 更新反馈通信数据包
  cv_comm.aim_shoot_board2pc_msg_.yaw_angle = status_.absolute_yaw_fdb;
  cv_comm.aim_shoot_board2pc_msg_.pitch_angle = status_.absolute_pitch_fdb;
  cv_comm.aim_shoot_board2pc_msg_.yaw_speed = status_.yaw_speed_fdb;
  cv_comm.aim_shoot_board2pc_msg_.pitch_speed = status_.pitch_speed_fdb;
  if (cv_comm.mode_ == CVMode::AUTOAIM || cv_comm.mode_ == CVMode::DECISION) {
    cv_comm.aim_shoot_board2pc_msg_.yaw_offset = offset_.yaw;
    cv_comm.aim_shoot_board2pc_msg_.pitch_offset = offset_.pitch;
  } else if (cv_comm.mode_ == CVMode::ENERGY ||
             cv_comm.mode_ == CVMode::ENERGY_DISTURB) {
    cv_comm.aim_shoot_board2pc_msg_.yaw_offset = offset_.energy_yaw;
    cv_comm.aim_shoot_board2pc_msg_.pitch_offset = offset_.energy_pitch;
  }
  cv_comm.aim_shoot_board2pc_msg_.dist = 10;
  cv_comm.aim_shoot_board2pc_msg_.shoot_speed = shoot.getBulletSpeed();
  cv_comm.aim_shoot_board2pc_msg_.shoot_delay = shoot.getShootDelay();
  // cv_comm.aim_shoot_board2pc_msg_.chassis_vx = chassis.fdb_.vx;
  // cv_comm.aim_shoot_board2pc_msg_.chassis_vy = chassis.fdb_.vy;
  if (referee.connect_.check()) {
    // 连接到裁判系统更新自身id
    cv_comm.aim_shoot_board2pc_msg_.self_id =
        referee.robot_status_.robot_id;
    // 能量机关类别判断
    if (referee.game_status_.game_progress == 4) {
      if (referee.game_status_.stage_remain_time > 200) {
        // 小符
        cv_comm.aim_shoot_board2pc_msg_.is_big_energy = 0;
      } else {
        // 大符
        cv_comm.aim_shoot_board2pc_msg_.is_big_energy = 1;
      }
    } else {
      // 非比赛阶段(todo)
    }
  } else {
    // 未连接裁判系统(todo)
  }

  // 更新自瞄数据
  status_.relative_yaw = cv_comm.aim_shoot_pc2board_msg_.yaw_angle;
  status_.relative_pitch = cv_comm.aim_shoot_pc2board_msg_.pitch_angle;
  status_.yaw_speed_ref = cv_comm.aim_shoot_pc2board_msg_.yaw_speed;
  status_.pitch_speed_ref = cv_comm.aim_shoot_pc2board_msg_.pitch_speed;
  status_.dist = cv_comm.aim_shoot_pc2board_msg_.dist;
  status_.shoot_flag = cv_comm.aim_shoot_pc2board_msg_.shoot_flag == 1;
  status_.idle_flag = cv_comm.aim_shoot_pc2board_msg_.shoot_flag == 2;
  status_.enemy_id = cv_comm.aim_shoot_pc2board_msg_.enemy_id;

  // 记录云台历史角度，处理云台角度与视觉指令的时间同步
  uint32_t gimbal_rcd_index = HAL_GetTick() % GIMBAL_ANGLE_RECORD_SIZE;
  gimbal_angle_rcd[gimbal_rcd_index].yaw = gimbal.fdb_.yaw;
  gimbal_angle_rcd[gimbal_rcd_index].pitch = gimbal.fdb_.pitch;

  // 根据云台历史角度设置云台当前目标角度
  gimbal_rcd_index =
      (HAL_GetTick() - gimbal_cv_delay) % GIMBAL_ANGLE_RECORD_SIZE;
  if (cv_comm.mode_ == CVMode::ENERGY ||
      cv_comm.mode_ == CVMode::ENERGY_DISTURB) {
    status_.absolute_yaw_ref =
        gimbal_angle_rcd[gimbal_rcd_index].yaw + status_.relative_yaw;
    status_.absolute_pitch_ref =
        gimbal_angle_rcd[gimbal_rcd_index].pitch + status_.relative_pitch;
  } else if (cv_comm.mode_ == CVMode::AUTOAIM ||
             cv_comm.mode_ == CVMode::DECISION) {
    status_.absolute_yaw_ref = gimbal_angle_rcd[gimbal_rcd_index].yaw +
                               status_.relative_yaw + offset_.yaw;
    status_.absolute_pitch_ref = gimbal_angle_rcd[gimbal_rcd_index].pitch +
                                 status_.relative_pitch + offset_.pitch;
  }

  // 云台控制
  if (aim_state_ && cv_comm.aim_shoot_connect_.check() && !status_.idle_flag) {
    gimbal.setAngleSpeed(status_.absolute_yaw_ref, status_.absolute_pitch_ref,
                         status_.yaw_speed_ref, status_.pitch_speed_ref);
  }
  // 发射控制
  if (shoot_state_ && status_.shoot_flag &&
      cv_comm.aim_shoot_connect_.check() && !status_.idle_flag) {
    if (shoot.shootOneBullet()) {
      // 成功发射修改反馈时间戳
      cv_comm.aim_shoot_board2pc_msg_.shoot_id_fdb =
          cv_comm.aim_shoot_pc2board_msg_.shoot_id;
    }
  }
}

// 自瞄状态设置
void Autoaim::setState(bool aim_state, bool shoot_state) {
  aim_state_ = aim_state;
  shoot_state_ = shoot_state;
}
