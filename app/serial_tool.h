/**
 ******************************************************************************
 * @file    serial_tool.cpp/h
 * @brief   Serial tool message package. 串口工具debug信息封装
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef SERIAL_TOOL_H
#define SERIAL_TOOL_H

#include <stdint.h>

#include "hardware_config.h"
#include "usart.h"

#include "app/autoaim.h"
#include "app/chassis.h"
#include "app/gimbal.h"
#include "app/imu_monitor.h"
#include "app/motor_monitor.h"
#include "app/power_limit.h"

class SerialStudio {
 public:
  // USB烧写程序会断连，如UART端口足够推荐使用UART
  SerialStudio(UART_HandleTypeDef* huart = nullptr) : huart_(huart) {}

  // USB接收回调
  void usbRxCallback(uint8_t* buf, uint32_t len);
  // UART接收回调
  void uartRxCallback(void);
  // UART端口校验
  bool uartCheck(UART_HandleTypeDef* huart) { return huart == huart_; }
  // 数据发送封装，在robot.cpp中调用
  void handle(void);

 private:
  // 数据打包发送
  void txData(float* data, uint16_t data_len);

  // 发送IMU数据
  void txIMUData(IMU& imu);
  // 发送电机数据
  void txMotorData(Motor& motor);
  // 发送云台数据
  void txGimbalData(Gimbal& gimbal);
  // 发送麦轮底盘数据
  void txChassisData(MecanumChassis& chassis);
  // 发送功率限制数据
  void txPowerLimitData(MecanumChassisPower& power);
  // 发送自瞄数据
  void txAutoaimData(Autoaim& autoaim);

 private:
  UART_HandleTypeDef* huart_;

  uint32_t usb_init_tick;

  struct Tx_t {
    uint8_t buf[256];  // 约50个%.3f数据
    uint32_t len;
    const char* frame_start = "/*";
    const char* frame_end = "*/\r\n";
  } tx_;

  struct Rx_t {
    uint8_t buf[64];
    uint32_t len;
  } rx_;

  float sys_time_;
};

#endif  // SERIAL_TOOL_H
