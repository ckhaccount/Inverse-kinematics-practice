/**
 ******************************************************************************
 * @file    mit_motor_driver.cpp/h
 * @brief   Driver program for motor use MIT protocol. MIT协议电机驱动
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef MIT_MOTOR_DRIVER_H
#define MIT_MOTOR_DRIVER_H

#include "base/motor/motor.h"
#include "can.h"

namespace mitmotor {

// motor mode command
typedef enum CmdType {
  NO_CMD = 0,
  MOTOR_MODE = 1,
  RESET_MODE = 2,
  ZERO_POSITION = 3,
} CmdType_e;

// feedback data
typedef struct Feedback {
  uint8_t id;
  float position;  // rad
  float velocity;  // rad/s
  float torq;      // N·m
} Feedback_t;

};  // namespace mitmotor

//
class MITMotorDriver {

 public:
  // motor mode command
  typedef enum Cmd {
    NO_CMD = 0,
    MOTOR_MODE = 1,
    RESET_MODE = 2,
    ZERO_POSITION = 3,
  } Cmd_e;

 public:
  MITMotorDriver(Motor* motor, CAN_HandleTypeDef* hcan, uint32_t master_id,
                 uint32_t slave_id, float p_min, float p_max, float v_min,
                 float v_max, float kp_min, float kp_max, float kv_min,
                 float kv_max, float t_ff_min, float t_ff_max, float t_min,
                 float t_max);

  // set motor mode command
  bool setCmd(mitmotor::CmdType_e cmd);

  // set motor control parameter
  // p: target position(rad)
  // v: target velocity(rad/s)
  // kp/kv: control gain
  // t: feedforward torque
  // T = kp*(p-p_fdb)+kv*(v-v_fdb)+t
  void setControlParam(float p, float v, float kp, float kv, float t_ff);

  // set torque
  void setTorque(const float& torque);

  // 查询电机ID(J23)
  void MotorGetID();

  // 查询电机通信模式，0x01：自动报文，0x02：问答(J123)
  void MotorGetCommMode();

  // 设置电机通信模式，0x01：自动报文，0x02：问答(J123)
  void MotorCommModeSet();

  // Transmit CAN message
  // J456
  void canTxMsg(void);
  // J123问答通信模式+力位混控控制模式
  void canTxMsg_OD_();

  // Check CAN channel and CAN message header id
  // J456
  bool canRxMsgCheck(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header);
  // J123问答模式
  bool canRxMsgCheck_OD_(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header);

  // Receive feedback data message callback. Called in
  // HAL_CAN_RxFifo0MsgPendingCallback()
  // J456
  void canRxMsgCallback(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header,
                        uint8_t rx_data[8]);
  // J123问答模式
  void canRxMsgCallback_OD_(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header,
                           uint8_t rx_data[8]);

 public:
  // motor pointer
  Motor* motor_;
  // Motor pointer group
  Motor* motor_OD_[4];

  // transmit data pack
  struct TxData {
    uint16_t p;
    uint16_t v;
    uint16_t kp;
    uint16_t kv;
    uint16_t t_ff;
  } tx_data_;

  // receive data pack
  struct RxData {
    uint8_t id;
    float position;  // rad
    float velocity;  // rad/s
    float torq;      // N·m
    float temperature;
    float error;
  } rx_data_;

 private:
  CAN_HandleTypeDef* hcan_;
  uint32_t master_id_;  // motor to board
  uint32_t slave_id_;   // board to motor

  // can transmit
  CAN_TxHeaderTypeDef can_tx_header_;
  uint8_t can_tx_data_[8];
  uint32_t can_tx_mail_box_;

  // motor command
  struct Command_t {
    uint32_t tick;
    const uint32_t interval = 100;
    mitmotor::CmdType_e list[5] = {mitmotor::NO_CMD};
    uint8_t cnt = 0;
  } cmd_;

  // motor parameters
  struct Param_t {
    // position limit(rad)
    float p_min;
    float p_max;
    // velocity limit(rad/s)
    float v_min;
    float v_max;
    // PD param limit
    float kp_min;
    float kp_max;
    float kv_min;
    float kv_max;
    // feedforward torque limit(N·m)
    float t_ff_min;
    float t_ff_max;
    // feedback torque limit(N·m)
    float t_min;
    float t_max;
  } param_;
};

#endif  // MIT_MOTOR_DRIVER_H
