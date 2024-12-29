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

#ifndef MOTOR_MONITOR_H
#define MOTOR_MONITOR_H

#include "base/motor/driver/dji_motor_driver.h"
#include "base/motor/driver/mit_motor_driver.h"
#include "base/motor/motor.h"

// Manage all motors' mode. Called in controlTask.
// 统一操作电机模式，在controlTask中调用
void allMotorsStopShutoff(void);
void allMotorsShutOff(void);
void allMotorsStop(void);
void allMotorsOn(void);

// Reset all motors.
// 电机统一初始化
void allMotorsInit(void);

// Handle all motors. Called in motorTask
// 电机统一处理，在motorTask中调用
void allMotorsHandle(void);

// Check CAN channel and id of received CAN message
// 校验接收信息的CAN通道和ID，调用对应回调函数
void motorsCanRxMsgHandle(CAN_HandleTypeDef* hcan,
                          CAN_RxHeaderTypeDef rx_header, uint8_t* rx_data);

extern Motor CMFL, CMFR, CMBL, CMBR, GMY, GMP, FRICL, FRICR, STIR, m1;
extern Motor JM1, JM2, JM3, JM4, JM5, JM6;
extern Motor PUMP;
extern Motor* can1_dji_motor[11];
extern Motor* can2_dji_motor[11];

#endif  // MOTOR_MONITOR_H
