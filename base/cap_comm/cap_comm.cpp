/**
 ******************************************************************************
 * @file    cap_comm.cpp/h
 * @brief   Capacity communication(CAN). 超级电容通信(CAN)
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "base/cap_comm/cap_comm.h"
#include <string.h>

const uint32_t capacity_timeout = 1000;

CapComm::CapComm(CAN_HandleTypeDef* hcan)
    : connect_(capacity_timeout), hcan_(hcan) {}

void CapComm::handle(void) {
  connect_.check();
}

// Transmit data to capacity
// 向电容控制板发送数据(未使用)
void CapComm::canTxMsg(void) {
  can_tx_header_.IDE = CAN_ID_STD;
  can_tx_header_.RTR = CAN_RTR_DATA;
  can_tx_header_.DLC = 8;
  can_tx_header_.StdId = board_id_;
  memcpy(can_tx_data_, &tx_msg_, 8);
  // transmit
  HAL_CAN_AddTxMessage(hcan_, &can_tx_header_, can_tx_data_, &can_tx_mail_box_);
}

// Check CAN channel and id of received CAN message
// 校验接收信息的CAN通道和ID
bool CapComm::canRxMsgCheck(CAN_HandleTypeDef* hcan,
                             CAN_RxHeaderTypeDef rx_header) {
  return hcan == hcan_ && rx_header.StdId == capacity_id_;
}

// Receive feedback data message callback. Called in
// HAL_CAN_RxFifo0MsgPendingCallback()
// 电容信息接收回调，在HAL_CAN_RxFifo0MsgPendingCallback中调用
void CapComm::canRxMsgCallback(CAN_HandleTypeDef* hcan,
                                CAN_RxHeaderTypeDef rx_header,
                                uint8_t rx_data[8]) {
  memcpy(&rx_msg_, rx_data, 8);
  connect_.refresh();
}
