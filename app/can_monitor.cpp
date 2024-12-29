/**
 ******************************************************************************
 * @file    can_monitor.cpp/h
 * @brief   CAN communication transmit manage. CAN通信发送管理
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/can_monitor.h"

#include "can.h"
#include "cmsis_os.h"

#include "app/board_comm.h"
#include "app/motor_monitor.h"
#include "base/cap_comm/cap_comm.h"

extern DJIMotorDriver dji_motor_driver;
extern MITMotorDriver mit_motor_driver[];

// CAN filter初始化
void canFilterInit(void) {
  CAN_FilterTypeDef filter;
  filter.FilterActivation = ENABLE;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.SlaveStartFilterBank = 14;

  filter.FilterBank = 0;
  HAL_CAN_ConfigFilter(&hcan1, &filter);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  filter.FilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan2, &filter);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// CAN通信发送管理
void canTxMonitor(void) {
  // note: 每个通道一次只能发送3个包

  // CAN1
  if (HAL_GetTick() % 2 == 0) {
    dji_motor_driver.canTxMsg(1, djimotor::ID_1_4);
  }

  if (HAL_GetTick() % 3 == 0) {
    mit_motor_driver[0].canTxMsg();
    mit_motor_driver[1].canTxMsg();
  } else if (HAL_GetTick() % 3 == 1) {
    mit_motor_driver[2].canTxMsg();
    mit_motor_driver[3].canTxMsg_OD_();
  } else if (HAL_GetTick() % 3 == 2) {
    mit_motor_driver[4].canTxMsg_OD_();
    mit_motor_driver[5].canTxMsg_OD_();
  }

  osDelay(1);

}
