/**
 ******************************************************************************
 * @file    shoot.cpp/h
 * @brief   Shoot control. 发射控制
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef SHOOT_H
#define SHOOT_H

#include "base/motor/motor.h"
#include "base/servo/servo.h"

class Shoot {
 public:
  Shoot(Motor* fric_l, Motor* fric_r, Motor* stir);

  // 发射一发弹丸(发射-true，未发射-false)
  bool shootOneBullet(void);
  // 发射数据处理
  void handle(void);
  // 设置射击参数
  void setShootParam(const float& speed_limit, const float& heat_limit,
                     const float& cooling_rate);
  // 设置最小射击间隔(<20ms)
  void setCD(const uint32_t& cd) { cd_ = cd; }

  // 开关摩擦轮
  void fricOn(void) { fric_state_ = true; }
  void fricOff(void) { fric_state_ = false; }

  // 射击速度
  float getBulletSpeed(void);
  // 发射延迟
  uint8_t getShootDelay(void) { return delay_; }

 private:
  // 射速处理
  void speedHandle(void);
  // 卡弹处理
  void blockHandle(void);
  // 热量处理
  void heatHandle(void);

 private:
  Motor *fric_l_, *fric_r_, *stir_;  // 电机指针

  bool shoot_state_;  // true-可以发射，false-不能发射
  bool fric_state_;   // 摩擦轮状态，true-开启，false-停止
  bool block_state_;  // 卡弹状态，true-卡弹，false-未卡弹
  bool heat_state_;   // 热量状态，true-剩余热量大于1发弹丸热量

  float speed_limit_;   // 射速上限
  float heat_limit_;    // 热量上限
  float cooling_rate_;  // 冷却

  float fric_speed_;          // 摩擦轮转速
  float bullet_speed_;        // 弹速
  uint32_t cd_;               // 发射间隔(ms)
  uint32_t last_tick_;        // 发射时间记录(ms)
  uint32_t block_tick_;       // 卡弹时间记录(ms)
  float calc_heat_;           // 计算热量
  const uint8_t delay_ = 20;  // 发弹延迟(控制+机械延迟，ms)

  // // 自适应射速(todo)
  // struct SelfAdaptSpeed_t {
  //   float fric_speed;
  //   float bullet_speed[5];
  //   uint8_t cnt;
  // } self_adapt_;
};

// 弹舱盖
class Gate {
 public:
  typedef enum State {
    COMPLIANCE,
    CLOSE,
    OPEN,
  } State_e;

 public:
  Gate(uint16_t close_pwm, uint16_t open_pwm, ServoZX361D* servo = nullptr);

  void init(void);
  void set(const Gate::State_e& state);

 private:
  ServoZX361D* servo_;
  Gate::State_e state_;
  uint16_t close_pwm_;
  uint16_t open_pwm_;
};

#endif  // SHOOT_H