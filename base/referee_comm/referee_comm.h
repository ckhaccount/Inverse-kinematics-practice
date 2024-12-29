/**
 ******************************************************************************
 * @file    referee_comm.cpp/h
 * @brief   Referee communication(UART). 裁判系统通信(UART)
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef REFEREE_COMM_H
#define REFEREE_COMM_H

#include "base/common/connect.h"
#include "base/common/fifo_buffer.h"
#include "base/referee_comm/referee_protocol.h"
#include "usart.h"

#define REFEREE_TX_BUF_SIZE REFEREE_COMM_FRAME_MAX_SIZE
#define REFEREE_RX_BUF_SIZE REFEREE_COMM_FRAME_MAX_SIZE

// Referee communication class 裁判系统通信类
class RefereeComm {
 public:
  RefereeComm(UART_HandleTypeDef* huart = nullptr);

  // Init UART receive 初始化，打开UART接收
  void init(void);
  // Handle data, check connection 处理数据，检查连接状态
  void handle(void);

  // // Data transmit 数据发送
  void txMsg(void);

  // Data receive callback 接收中断
  void rxCallback(void);
  // UART端口校验
  bool uartCheck(UART_HandleTypeDef* huart) { return huart == huart_; }

  UART_HandleTypeDef* getHuart() const;

 public:
  Connect connect_;

  game_status_t game_status_;      // 比赛状态数据
  game_result_t game_result_;      // 比赛结果数据
  game_robot_HP_t game_robot_HP_;  // 机器人血量数据

  event_data_t event_data_;                              // 场地事件数据
  supply_projectile_action_t supply_projectile_action_;  // 补给站动作标识
  referee_warning_t referee_warning_;                    // 裁判警告信息
  dart_remaining_time_t dart_remaining_time_;  // 飞镖发射口倒计时

  robot_status_t robot_status_;                  // 比赛机器人状态
  power_heat_data_t power_heat_data_;            // 实时功率热量数据
  robot_pos_t robot_pos_;                        // 机器人位置
  buff_t buff_;                                  // 机器人增益
  air_support_data_t air_support_data_;          // 空中支援时间数据
  hurt_data_t hurt_data_;                        // 伤害状态数据
  shoot_data_t shoot_data_;                      // 实时射击数据
  projectile_allowance_t projectile_allowance_;  // 允许发弹量
  rfid_status_t rfid_status_;                    // 机器人RFID状态
  dart_client_cmd_t dart_client_cmd_;            // 客户端飞镖指令数据
  ground_robot_position_t ground_robot_position_;  // 地面机器人位置数据
  radar_mark_data_t radar_mark_data_;              // 雷达标记进度数据

  // todo: interactive data
  custom_robot_data_t custom_robot_data_;  // 自定义控制器交互数据(另外实现)
  map_command_t map_command_;              // 选手端小地图交互数据
  remote_control_t remote_control_;  // 键鼠遥控数据(todo，文档混乱)
  map_robot_data_t map_robot_data_;  // 选手端小地图接收雷达数据
  custom_client_data_t
      custom_client_data_;  // 自定义控制器与选手端交互数据(todo，文档混乱)
  map_sentry_data_t map_sentry_data_;  // 选手端小地图接收哨兵数据

 private:
  UART_HandleTypeDef* huart_;

  struct Tx_t {
    uint8_t buf[REFEREE_TX_BUF_SIZE];
    RefereeCommFrame_t frame;
    uint32_t pack_size;
  } tx_;

  struct Rx_t {
    Rx_t(void) : fifo(buf, REFEREE_RX_BUF_SIZE) {}

    uint8_t byte[1];
    uint8_t buf[REFEREE_RX_BUF_SIZE];
    FIFOBuffer fifo;
    RefereeCommFrame_t frame;
    uint32_t expect_size;
  } rx_;

  enum MessageUnpackStep_e {
    WAIT,
    DATA_LEN,
    SEQ,
    HEADER_CRC8,
    CMD_ID,
    DATA,
    PACK_CRC16,
    READ_DATA,
  } unpack_step_;

  enum UnpackError_e {
    NO_ERROR,
    DATA_LEN_OUTRANGE,
    HEADER_CRC_FAIL,
    ID_UNDEFINED,
    PACK_CRC_FAIL,
  } unpack_error_;
};

#endif  // REFEREE_COMM_H