/**
 ******************************************************************************
 * @file    motor_monitor.cpp/h
 * @brief   Motor parameter, id config and management. 电机参数id配置和统一管理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/motor_monitor.h"
#include "lib/arm_math/arm_math.h"
#include <cmath>
// Motor parameter config
// 电机参数配置

// J456电机参数
namespace m4310 {
// position limit(rad)
const float p_min = -12.5;
const float p_max = 12.5;
// velocity limit(rad/s)
const float v_min = -30;
const float v_max = 30;
// PD param limit
const float kp_min = 0;
const float kp_max = 500;
const float kv_min = 0;
const float kv_max = 5;
// feedforward torque limit(N·m)
const float t_ff_min = -10;
const float t_ff_max = 10;
// feedback torque limit(N·m)
const float t_min = -10;
const float t_max = 10;
}  // namespace m4310

// J123电机参数
namespace m4310_A {
// position limit(rad)
const float p_min = -12.5;
const float p_max = 12.5;
// velocity limit(rad/s)
const float v_min = -18;
const float v_max = 18;
// PD param limit
const float kp_min = 0;
const float kp_max = 500;
const float kv_min = 0;
const float kv_max = 50;
// feedforward torque limit(N·m)
const float t_ff_min = -30;
const float t_ff_max = 30;
// feedback torque limit(N·m)
const float t_min = -30;
const float t_max = 30;
}  // namespace m4310

// Chassis motor 底盘电机
const PID chassis_wheel_spid(40, 1, 10, 1000, 16384);
Motor CMFL(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
           PID(), PID(chassis_wheel_spid));             // ppid, spid
Motor CMFR(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
           PID(), PID(chassis_wheel_spid));             // ppid, spid
Motor CMBL(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
           PID(), PID(chassis_wheel_spid));             // ppid, spid
Motor CMBR(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
           PID(), PID(chassis_wheel_spid));             // ppid, spid

// Gimbal motor 云台电机
Motor GMY(Motor::GM6020, 1, Motor::POSITION_SPEED,   // type, ratio, method
          PID(25, 0.1, 10, 10, 1800),                // ppid
          PID(400, 2, 0, 500, 30000),                // spid
          true);                                     // use kf
Motor GMP(Motor::GM6020, -1, Motor::POSITION_SPEED,  // type, ratio, method
          PID(25, 0.1, 10, 10, 1800),                // ppid
          PID(400, 2, 0, 500, 30000),                // spid
          true);                                     // use kf

// Friction wheel motors
const PID fric_spid(6, 0.05, 2, 200, 16384);
Motor FRICL(Motor::M3508, 1, Motor::SPEED,  // type, ratio, method
            PID(), PID(fric_spid));         // ppid, spid
Motor FRICR(Motor::M3508, 1, Motor::SPEED,  // type, ratio, method
            PID(), PID(fric_spid));         // ppid, spid

// Stir motor
Motor STIR(Motor::M2006, 36, Motor::POSITION_SPEED,  // type, ratio, method
           PID(20, 0.1, 10, 10, 2500),               // ppid
           PID(60, 0.1, 200, 1000, 10000));          // spid

// Other motors
Motor m1(Motor::M3508, 1, Motor::POSITION_SPEED,  // type, ratio, method
         PID(50, 0.02, 50, 200, 36000),           // ppid
         PID(20, 0.02, 50, 200, 16384));          // spid

Motor JM6(Motor::MIT, 1, Motor::POSITION_SPEED,          // type, ratio, method
          PID(20, 0.1, 10, 100, 360),                    // ppid
          PID(6e-3, 0, 1.5e-2, 0, 3),                    // spid
          true, Motor::KFParam_t(2, 1e4, 1, 0.75, 50));  // kf
Motor JM5(Motor::MIT, 1, Motor::POSITION_SPEED,          // type, ratio, method
          PID(20, 0.1, 10, 100, 360),                    // ppid
          PID(6e-3, 0, 1.5e-2, 0, 3),                    // spid
          true, Motor::KFParam_t(2, 1e4, 1, 0.75, 50));  // kf
Motor JM4(Motor::MIT, 1, Motor::POSITION_SPEED,          // type, ratio, method
          PID(45, 0, 25, 100, 360),                    // ppid
          PID(8e-3, 0, 3e-2, 0, 10),                    // spid
          true, Motor::KFParam_t(2, 1e4, 1, 0.75, 50));  // kf

Motor JM3(Motor::OD, 1, Motor::POSITION_SPEED,          // type, ratio, method
          PID(30, 0, 10, 100, 360),                    // ppid
          PID(4e-2, 0, 2e-2, 0, 10),                    // spid
          true, Motor::KFParam_t(2, 1e4, 1, 0.75, 50));  // kf
Motor JM2(Motor::OD, 1, Motor::POSITION_SPEED,          // type, ratio, method
          PID(40, 0.1, 10, 100, 360),                    // ppid
          PID(6e-2, 0, 1.5e-2, 0, 20),                    // spid
          true, Motor::KFParam_t(2, 1e4, 1, 0.75, 50));  // kf
Motor JM1(Motor::OD, 1, Motor::POSITION_SPEED,          // type, ratio, method
          PID(20, 0.1, 10, 100, 360),                    // ppid
          PID(3e-2, 0, 1.5e-2, 0, 5),                    // spid
          true, Motor::KFParam_t(2, 1e4, 1, 0.75, 50));  // kf

// 气泵电机
const PID pump_spid(5, 0, 5, 1000, 16384);
Motor PUMP(Motor::M3508, 1, Motor::SPEED,                 // type, ratio, method
          PID(), pump_spid,                              // ppid, spid
          true, Motor::KFParam_t(2, 1e4, 1, 0.75, 50));  // kf


// DJI Motor id config, M3508/M2006: 1~8, GM6020: 5~11
// DJI电机ID配置，M3508，M2006可配置范围为1~8，GM6020可配置范围为5~11
Motor* can1_dji_motor[11] = {
    &PUMP,   // id:1
    nullptr,   // id:2
    nullptr,  // id:3
    nullptr,  // id:4
    nullptr,    // id:5
    nullptr,     // id:6
    nullptr,  // id:7
    nullptr,      // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};
Motor* can2_dji_motor[11] = {
    nullptr,    // id:1
    nullptr,    // id:2
    nullptr,    // id:3
    nullptr,    // id:4
    nullptr,     // id:5
    nullptr,  // id:6
    nullptr,  // id:7
    nullptr,  // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};

DJIMotorDriver dji_motor_driver(can1_dji_motor, can2_dji_motor);

Motor* motor_OD[4] = {
    &JM1,       // id:1
    &JM2,       // id:2
    nullptr,    // id:3
    &JM3,       // id:4
};

MITMotorDriver mit_motor_driver[] = {
    MITMotorDriver(&JM6, &hcan1, 0x00, 0x08, m4310::p_min, m4310::p_max,
                   m4310::v_min, m4310::v_max, m4310::kp_min, m4310::kp_max,
                   m4310::kv_min, m4310::kv_max, m4310::t_ff_min,
                   m4310::t_ff_max, m4310::t_min, m4310::t_max),
    MITMotorDriver(&JM5, &hcan1, 0x00, 0x06, m4310::p_min, m4310::p_max,
                   m4310::v_min, m4310::v_max, m4310::kp_min, m4310::kp_max,
                   m4310::kv_min, m4310::kv_max, m4310::t_ff_min,
                   m4310::t_ff_max, m4310::t_min, m4310::t_max),
    MITMotorDriver(&JM4, &hcan1, 0x00, 0x05, m4310::p_min, m4310::p_max,
                   m4310::v_min, m4310::v_max, m4310::kp_min, m4310::kp_max,
                   m4310::kv_min, m4310::kv_max, m4310::t_ff_min,
                   m4310::t_ff_max, m4310::t_min, m4310::t_max),

    MITMotorDriver(&JM3, &hcan1, 0x04, 0x04, m4310_A::p_min, m4310_A::p_max,
                   m4310_A::v_min, m4310_A::v_max, m4310_A::kp_min, m4310_A::kp_max,
                   m4310_A::kv_min, m4310_A::kv_max, m4310_A::t_ff_min,
                   m4310_A::t_ff_max, m4310_A::t_min, m4310_A::t_max),
    MITMotorDriver(&JM2, &hcan1, 0x02, 0x02, m4310_A::p_min, m4310_A::p_max,
                   m4310_A::v_min, m4310_A::v_max, m4310_A::kp_min, m4310_A::kp_max,
                   m4310_A::kv_min, m4310_A::kv_max, m4310_A::t_ff_min,
                   m4310_A::t_ff_max, m4310_A::t_min, m4310_A::t_max),
    MITMotorDriver(&JM1, &hcan1, 0x01, 0x01, m4310_A::p_min, m4310_A::p_max,
                   m4310_A::v_min, m4310_A::v_max, m4310_A::kp_min, m4310_A::kp_max,
                   m4310_A::kv_min, m4310_A::kv_max, m4310_A::t_ff_min,
                   m4310_A::t_ff_max, m4310_A::t_min, m4310_A::t_max),
};

// 电机停转速度上限(dps)
const float motor_stop_rotate_speed_thres = 1200;

// Stop and shut off all motors
// 所有电机先停转再断电
void allMotorsStopShutoff(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can1_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can1_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can1_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can2_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can2_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can2_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
  }
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->mode_ = Motor::POWEROFF;
  }
}

// Shut off all motors
// 所有电机断电
void allMotorsShutOff(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
  }
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->mode_ = Motor::POWEROFF;
  }
}

// Stop all motors
// 所有电机停转
void allMotorsStop(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::STOP;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::STOP;
    }
  }
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->mode_ = Motor::STOP;
  }
}

// Startup all motors
// 所有电机启动
void allMotorsOn(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::WORKING;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::WORKING;
    }
  }
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->mode_ = Motor::WORKING;
  }
}

// Reset all motors.
// 电机统一初始化
void allMotorsInit(void) {
  dji_motor_driver.idConfig();
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->reset();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->reset();
    }
  }
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->reset();
    mit_motor_driver[i].setCmd(mitmotor::CmdType_e ::MOTOR_MODE);
  }
}

// Handle all motors. Called in motorTask
// 电机统一处理，在motorTask中调用
void allMotorsHandle(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->handle();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->handle();
    }
  }
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->handle();
    if (!mit_motor_driver[i].motor_->connect_.check()) {
      mit_motor_driver[i].setCmd(mitmotor::CmdType_e::MOTOR_MODE);
    }
  }
}

// Check CAN channel and id of received CAN message
// 校验接收信息的CAN通道和ID，调用对应回调函数
void motorsCanRxMsgHandle(CAN_HandleTypeDef* hcan,
                          CAN_RxHeaderTypeDef rx_header, uint8_t* rx_data) {
  if (dji_motor_driver.canRxMsgCheck(hcan, rx_header)) {
    dji_motor_driver.canRxMsgCallback(hcan, rx_header, rx_data);
  }

  if (mit_motor_driver[0].canRxMsgCheck(hcan, rx_header)) {
    mit_motor_driver[0].canRxMsgCallback(hcan, rx_header, rx_data);
  }
  if (mit_motor_driver[1].canRxMsgCheck(hcan, rx_header)) {
    mit_motor_driver[1].canRxMsgCallback(hcan, rx_header, rx_data);
  }
  if (mit_motor_driver[2].canRxMsgCheck(hcan, rx_header)) {
    mit_motor_driver[2].canRxMsgCallback(hcan, rx_header, rx_data);
  }

  if (mit_motor_driver[3].canRxMsgCheck_OD_(hcan, rx_header)) {
    mit_motor_driver[3].canRxMsgCallback_OD_(hcan, rx_header, rx_data);
  }
  if (mit_motor_driver[4].canRxMsgCheck_OD_(hcan, rx_header)) {
    mit_motor_driver[4].canRxMsgCallback_OD_(hcan, rx_header, rx_data);
  }
  if (mit_motor_driver[5].canRxMsgCheck_OD_(hcan, rx_header)) {
    mit_motor_driver[5].canRxMsgCallback_OD_(hcan, rx_header, rx_data);
  }
}
