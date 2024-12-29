/**
 ******************************************************************************
 * @file    serial_tool.cpp/h
 * @brief   Serial tool message package. 串口工具(Serial tool)debug信息封装
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/serial_tool.h"
#include <stdio.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

extern IMU board_imu;
extern Gimbal gimbal;
extern MecanumChassis chassis;
extern MecanumChassisPower power_limit;
extern Autoaim autoaim;

// 数据发送封装，在robot.cpp中调用
void SerialStudio::handle(void) {
  // txIMUData(board_imu);
  // txMotorData(m1);
  // txGimbalData(gimbal);
  // txChassisData(chassis);
  txPowerLimitData(power_limit);
  // txAutoaimData(autoaim);

#ifdef DEBUG_USB
  // USB auto reconnect in suspended state
  // USB挂起状态下重新初始化自动重连
  if (hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED) {
    if (HAL_GetTick() - usb_init_tick > 100) {
      MX_USB_DEVICE_Init();
      usb_init_tick = HAL_GetTick();
    }
  }
#endif  // DEBUG_USB
}

// USB接收回调
void SerialStudio::usbRxCallback(uint8_t* buf, uint32_t len) {
  rx_.len = MIN(len, sizeof(rx_.buf));
  memcpy(rx_.buf, buf, rx_.len);
}

// UART接收回调
void SerialStudio::uartRxCallback(void) {
  // todo
}

// 数据打包发送
void SerialStudio::txData(float* data, uint16_t data_len) {
  tx_.len = sprintf((char*)tx_.buf, "%s", tx_.frame_start);
  for (int i = 0; i < data_len; i++) {
    tx_.len += sprintf((char*)tx_.buf + tx_.len, "%.3f,", data[i]);
  }
  tx_.len += sprintf((char*)tx_.buf + tx_.len, "%s", tx_.frame_end);

#ifdef DEBUG_USB
  CDC_Transmit_FS((uint8_t*)tx_.buf, tx_.len);
#endif  // DEBUG_USB
  if (huart_ != nullptr) {
    HAL_UART_Transmit_IT(huart_, (uint8_t*)tx_.buf, tx_.len);
  }
}

// IMU数据可视化/采集
void SerialStudio::txIMUData(IMU& imu) {
  sys_time_ = (float)HAL_GetTick() * 1e-3f;
  float txdata_pack[] = {
      sys_time_,       // %1, time
      imu.yaw(),       // %2, yaw
      imu.pitch(),     // %3, pitch
      imu.roll(),      // %4, roll
      imu.wxSensor(),  // %5, wx
      imu.wySensor(),  // %6, wy
      imu.wzSensor(),  // %7, wz
      imu.axSensor(),  // %8, ax(sensor)
      imu.aySensor(),  // %9, ay(sensor)
      imu.azSensor(),  // %10, az(sensor)
      imu.axWorld(),   // %11, ax(-g, world)
      imu.ayWorld(),   // %12, ay(-g, world)
      imu.azWorld(),   // %13, az(-g, world)
      imu.quat(0),     // %14, q0
      imu.quat(1),     // %15, q1
      imu.quat(2),     // %16, q2
      imu.quat(3),     // %17, q3
  };
  txData(txdata_pack, sizeof(txdata_pack) / sizeof(float));
}

// 电机数据可视化/采集
void SerialStudio::txMotorData(Motor& motor) {
  sys_time_ = (float)HAL_GetTick() * 1e-3f;
  float txdata_pack[] = {
      sys_time_,                       // %1
      motor.targetAngle(),             // %2
      motor.realAngle(),               // %3
      motor.targetSpeed(),             // %4
      motor.realSpeed(),               // %5
      motor.motor_data_.angle,         // %6
      motor.kf_data_.x[0],             // %7
      motor.motor_data_.rotate_speed,  // %8
      motor.kf_data_.x[1],             // %9
      (float)motor.intensity_,         // %10
      motor.motor_data_.current,       // %11
  };
  txData(txdata_pack, sizeof(txdata_pack) / sizeof(float));
}

// 云台数据可视化/采集
void SerialStudio::txGimbalData(Gimbal& gimbal) {
  sys_time_ = (float)HAL_GetTick() * 1e-3f;
  float txdata_pack[] = {
      sys_time_,                            // %1
      gimbal.ref_.yaw,                      // %2
      gimbal.fdb_.yaw,                      // %3
      gimbal.ref_.pitch,                    // %4
      gimbal.fdb_.pitch,                    // %5
      gimbal.ref_.yaw_speed,                // %6
      gimbal.fdb_.yaw_speed,                // %7
      gimbal.ref_.pitch_speed,              // %8
      gimbal.fdb_.pitch_speed,              // %9
      (float)gimbal.gm_yaw_->intensity_,    // %10
      (float)gimbal.gm_pitch_->intensity_,  // %11
  };
  txData(txdata_pack, sizeof(txdata_pack) / sizeof(float));
}

// 麦轮底盘数据可视化/采集
void SerialStudio::txChassisData(MecanumChassis& chassis) {
  sys_time_ = (float)HAL_GetTick() * 1e-3f;
  float txdata_pack[] = {
      sys_time_,                    // %1
      chassis.ref_.vx,              // %2
      chassis.fdb_.vx,              // %3
      chassis.ref_.vy,              // %4
      chassis.fdb_.vy,              // %5
      chassis.ref_.wz,              // %6
      chassis.fdb_.wz,              // %7
      chassis.ref_.angle,           // %8
      chassis.fdb_.angle,           // %9
      chassis.ref_.wheel_speed.fl,  // %10
      chassis.fdb_.wheel_speed.fl,  // %11
      chassis.ref_.wheel_speed.fr,  // %12
      chassis.fdb_.wheel_speed.fr,  // %13
      chassis.ref_.wheel_speed.bl,  // %14
      chassis.fdb_.wheel_speed.bl,  // %15
      chassis.ref_.wheel_speed.br,  // %16
      chassis.fdb_.wheel_speed.br,  // %17
  };
  txData(txdata_pack, sizeof(txdata_pack) / sizeof(float));
}

// 发送功率限制数据
void SerialStudio::txPowerLimitData(MecanumChassisPower& power) {
  sys_time_ = (float)HAL_GetTick() * 1e-3f;
  float txdata_pack[] = {
      sys_time_,                                                       // %1
      power_limit.motor_power_fdb_.fl,                                 // %2
      power_limit.motor_power_fdb_.fr,                                 // %3
      power_limit.motor_power_fdb_.bl,                                 // %4
      power_limit.motor_power_fdb_.br,                                 // %5
      power_limit.ref_power_limit_,                                    // %6
      power_limit.chassis_power_fdb_,                                  // %7
      power_limit.chassis_power_est_,                                  // %8
      power_limit.referee_->power_heat_data_.chassis_power,            // %9
      (float)power_limit.referee_->robot_status_.chassis_power_limit,  // %10
      power_limit.ultra_cap_->rx_msg_.chassis_power,                   // %11
      (float)power_limit.referee_->power_heat_data_.buffer_energy,     // %12
      power_limit.ultra_cap_->rx_msg_.cap_voltage * 1e-3f,             // %13
      power_limit.speed_rate_,                                         // %14
      power_limit.eq_.r,                                               // %15
      power_limit.chassis_->ref_.vx,                                   // %16
      power_limit.chassis_->ref_.vy,                                   // %17
      power_limit.chassis_->ref_.wz,                                   // %18
  };
  txData(txdata_pack, sizeof(txdata_pack) / sizeof(float));
}

// 发送自瞄数据
void SerialStudio::txAutoaimData(Autoaim& autoaim) {
  sys_time_ = (float)HAL_GetTick() * 1e-3f;
  float txdata_pack[] = {
      sys_time_,                           // %1
      autoaim.status_.relative_yaw,        // %2
      autoaim.status_.relative_pitch,      // %3
      autoaim.status_.absolute_yaw_ref,    // %4
      autoaim.status_.absolute_yaw_fdb,    // %5
      autoaim.status_.absolute_pitch_ref,  // %6
      autoaim.status_.absolute_pitch_fdb,  // %7
      autoaim.status_.yaw_speed_ref,       // %8
      autoaim.status_.yaw_speed_fdb,       // %9
      autoaim.status_.pitch_speed_ref,     // %10
      autoaim.status_.pitch_speed_fdb,     // %11
      autoaim.offset_.yaw,                 // %12
      autoaim.offset_.pitch,               // %13
  };
  txData(txdata_pack, sizeof(txdata_pack) / sizeof(float));
}
