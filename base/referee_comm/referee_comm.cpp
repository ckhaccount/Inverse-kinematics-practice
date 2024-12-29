/**
 ******************************************************************************
 * @file    referee_comm.cpp/h
 * @brief   Referee communication(UART). 裁判系统通信(UART)
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "base/referee_comm/referee_comm.h"
#include "base/common/crc.h"
#include "base/referee_comm/referee_ui.h"

const uint32_t referee_comm_timout = 1000;  // ms

RefereeComm::RefereeComm(UART_HandleTypeDef* huart)
    : huart_(huart), connect_(referee_comm_timout), unpack_step_(WAIT) {}

// Init UART receive 初始化，打开UART接收
void RefereeComm::init(void) {
  if (huart_ != nullptr) {
    HAL_UART_Receive_IT(huart_, rx_.byte, 1);
  }
}

// Handle data, check connection 处理数据，检查连接状态
void RefereeComm::handle(void) {
  connect_.check();
  // txMsg();
}

// // Data transmit 数据发送
void RefereeComm::txMsg(void) {
  if (huart_ != nullptr) {
    HAL_UART_Transmit_IT(huart_, tx_.buf, tx_.pack_size);
  }
}

// Data receive callback 接收中断
void RefereeComm::rxCallback(void) {
  // 将接收到的1byte数据存入fifo缓冲区
  rx_.fifo.append(rx_.byte, 1);

  // 数据解包
  // 等待阶段
  if (unpack_step_ == WAIT) {
    uint32_t sof_index = rx_.fifo.find(referee_comm_sof);
    if (sof_index != rx_.fifo.size()) {
      // 检测到SOF标志，清除缓冲区无效数据，进入data_len阶段
      rx_.fifo.remove(sof_index);
      rx_.expect_size =
          sizeof(rx_.frame.header.sof) + sizeof(rx_.frame.header.data_len);
      unpack_step_ = DATA_LEN;
    } else {
      // 未检测到SOF标志，清空缓冲区
      rx_.fifo.clear();
    }
  }
  // data_len阶段
  if (unpack_step_ == DATA_LEN) {
    if (rx_.fifo.size() >= rx_.expect_size) {
      // 记录data_len，进入seq阶段
      memcpy(&rx_.frame.header.data_len, rx_.buf + sizeof(rx_.frame.header.sof),
             sizeof(rx_.frame.header.data_len));
      if (rx_.frame.header.data_len + sizeof(rx_.frame) > rx_.fifo.maxSize()) {
        // 数据长度错误
        rx_.fifo.clear();
        unpack_error_ = DATA_LEN_OUTRANGE;
        unpack_step_ = WAIT;
      } else {
        rx_.expect_size += sizeof(rx_.frame.header.seq);
        unpack_step_ = SEQ;
      }
    }
  }
  // seq阶段
  if (unpack_step_ == SEQ) {
    if (rx_.fifo.size() >= rx_.expect_size) {
      // 记录seq，进入帧头CRC8校验阶段
      memcpy(&rx_.frame.header.seq,
             rx_.buf + sizeof(rx_.frame.header.sof) +
                 sizeof(rx_.frame.header.data_len),
             sizeof(rx_.frame.header.seq));
      rx_.expect_size += sizeof(rx_.frame.header.crc8);
      unpack_step_ = HEADER_CRC8;
    }
  }
  // 帧头CRC8校验
  if (unpack_step_ == HEADER_CRC8) {
    if (rx_.fifo.size() >= rx_.expect_size) {
      rx_.frame.header.crc8 = CRC8_Calc(rx_.buf, rx_.expect_size);
      if (CRC8_Verify(rx_.buf, rx_.expect_size)) {
        // CRC8校验成功
        rx_.expect_size += sizeof(rx_.frame.cmd_id);
        unpack_step_ = CMD_ID;
      } else {
        // CRC8校验失败
        rx_.fifo.clear();
        unpack_error_ = HEADER_CRC_FAIL;
        unpack_step_ = WAIT;
      }
    }
  }
  // cmd id
  if (unpack_step_ == CMD_ID) {
    if (rx_.fifo.size() >= rx_.expect_size) {
      // 记录cmd_id，进入data阶段
      memcpy(&rx_.frame.cmd_id, rx_.buf + sizeof(rx_.frame.header),
             sizeof(rx_.frame.cmd_id));
      rx_.expect_size += rx_.frame.header.data_len;
      unpack_step_ = DATA;
    }
  }
  // data阶段，接收数据量足够后进入帧尾校验阶段
  if (unpack_step_ == DATA) {
    if (rx_.fifo.size() >= rx_.expect_size) {
      // 进入tail阶段
      rx_.expect_size += sizeof(rx_.frame.crc16);
      unpack_step_ = PACK_CRC16;
    }
  }
  // 帧尾CRC16校验阶段，接收完帧尾后进行校验
  if (unpack_step_ == PACK_CRC16) {
    if (rx_.fifo.size() >= rx_.expect_size) {
      rx_.frame.crc16 = CRC16_Calc(rx_.buf, rx_.expect_size);
      if (CRC16_Verify(rx_.buf, rx_.expect_size)) {
        // CRC校验成功
        unpack_step_ = READ_DATA;
      } else {
        // CRC校验失败
        rx_.fifo.clear();
        unpack_error_ = PACK_CRC_FAIL;
        unpack_step_ = WAIT;
      }
    }
  }
  // read_data阶段，从缓冲区读取数据
  if (unpack_step_ == READ_DATA) {
    const static uint8_t data_offset =
        sizeof(rx_.frame.header) + sizeof(rx_.frame.cmd_id);
    unpack_error_ = NO_ERROR;
    if (rx_.frame.cmd_id == GAME_STATUS_ID) {
      memcpy(&game_status_, rx_.buf + data_offset, rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == GAME_RESULT_ID) {
      memcpy(&game_result_, rx_.buf + data_offset, rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == GAME_ROBOT_HP_ID) {
      memcpy(&game_robot_HP_, rx_.buf + data_offset, rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == EVENT_DATA_ID) {
      memcpy(&event_data_, rx_.buf + data_offset, rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == SUPPLY_PROJECTILE_ACTION_ID) {
      memcpy(&supply_projectile_action_, rx_.buf + data_offset,
             rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == REFEREE_WARNING_ID) {
      memcpy(&referee_warning_, rx_.buf + data_offset,
             rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == DART_REMAINING_TIME_ID) {
      memcpy(&dart_remaining_time_, rx_.buf + data_offset,
             rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == ROBOT_STATUS_ID) {
      memcpy(&robot_status_, rx_.buf + data_offset, rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == POWER_HEAT_DATA_ID) {
      memcpy(&power_heat_data_, rx_.buf + data_offset,
             rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == ROBOT_POS_ID) {
      memcpy(&robot_pos_, rx_.buf + data_offset, rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == BUFF_ID) {
      memcpy(&buff_, rx_.buf + data_offset, rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == AERIAL_SUPPORT_DATA_ID) {
      memcpy(&air_support_data_, rx_.buf + data_offset,
             rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == HURT_DATA_ID) {
      memcpy(&hurt_data_, rx_.buf + data_offset, rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == SHOOT_DATA_ID) {
      memcpy(&shoot_data_, rx_.buf + data_offset, rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == PROJECTILE_ALLOWANCE_ID) {
      memcpy(&projectile_allowance_, rx_.buf + data_offset,
             rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == RFID_STATUS_ID) {
      memcpy(&rfid_status_, rx_.buf + data_offset, rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == DART_CLIENT_CMD_ID) {
      memcpy(&dart_client_cmd_, rx_.buf + data_offset,
             rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == GROUND_ROBOT_POSITION_ID) {
      memcpy(&ground_robot_position_, rx_.buf + data_offset,
             rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == RADAR_MARK_DATA_ID) {
      memcpy(&radar_mark_data_, rx_.buf + data_offset,
             rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == MAP_COMMAND_ID) {
      memcpy(&map_command_, rx_.buf + data_offset, rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == REMOTE_CONTROL_ID) {
      memcpy(&remote_control_, rx_.buf + data_offset,
             rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == MAP_ROBOT_DATA_ID) {
      memcpy(&map_robot_data_, rx_.buf + data_offset,
             rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == CUSTOM_CLIENT_DATA_ID) {
      memcpy(&custom_client_data_, rx_.buf + data_offset,
             rx_.frame.header.data_len);
    } else if (rx_.frame.cmd_id == MAP_SENTRY_DATA_ID) {
      memcpy(&map_sentry_data_, rx_.buf + data_offset,
             rx_.frame.header.data_len);
    } else {
      unpack_error_ = ID_UNDEFINED;
    }
    rx_.fifo.remove(rx_.expect_size);  // 从缓冲区移除该帧数据
    unpack_step_ = WAIT;               // 重置解包状态
    // 刷新连接状态
    if (unpack_error_ == NO_ERROR) {
      connect_.refresh();
    }
  }

  // 重新打开串口接收
  if (huart_ != nullptr) {
    HAL_UART_Receive_IT(huart_, rx_.byte, 1);
  }
}

UART_HandleTypeDef* RefereeComm::getHuart() const {
  return huart_;
}
