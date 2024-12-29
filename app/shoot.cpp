/**
 ******************************************************************************
 * @file    shoot.cpp/h
 * @brief   Shoot control. 发射控制
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/shoot.h"
#include "base/common/math.h"
#include "base/referee_comm/referee_comm.h"

extern RefereeComm referee;

const float stir_step_angle = 45;
const float heat_17mm_bullet = 10;

const float stir_block_current = 8000;
const float stir_block_angle = 8 * stir_step_angle;

Shoot::Shoot(Motor* fric_l, Motor* fric_r, Motor* stir)
    : fric_l_(fric_l),
      fric_r_(fric_r),
      stir_(stir),
      fric_state_(false),
      block_state_(false),
      heat_state_(true),
      speed_limit_(15),
      heat_limit_(200),
      cooling_rate_(10),
      cd_(40) {}

// 发射一发弹丸(发射-true，未发射-false)
bool Shoot::shootOneBullet(void) {
  if (shoot_state_) {
    stir_->targetAngle() -= stir_step_angle;
    last_tick_ = HAL_GetTick();
    calc_heat_ += heat_17mm_bullet;
  }
  return shoot_state_;
}

// 发射数据处理
void Shoot::handle(void) {
  if (referee.connect_.check()) {
    // 连接裁判系统(未连接裁判系统可在control中调用setShootParam设置不同模式的射击参数)
    setShootParam(
        referee.robot_status_.shooter_id1_17mm_initial_launching_speed_limit,
        referee.robot_status_.shooter_id1_17mm_barrel_heat_limit,
        referee.robot_status_.shooter_id1_17mm_barrel_cooling_value);
  }
  speedHandle();
  blockHandle();
  heatHandle();
  // 判断是否可发射(摩擦轮开 & 未卡弹 & 下一发不会超热量 & 发射未cd)
  shoot_state_ = fric_state_ && !block_state_ && heat_state_ &&
                 HAL_GetTick() - last_tick_ > cd_;
}

// 设置射击参数
void Shoot::setShootParam(const float& speed_limit, const float& heat_limit,
                          const float& cooling_rate) {
  speed_limit_ = speed_limit;
  heat_limit_ = heat_limit;
  cooling_rate_ = cooling_rate;
}

// 射击速度
float Shoot::getBulletSpeed(void) {
  if (referee.connect_.check() && referee.shoot_data_.initial_speed != 0) {
    bullet_speed_ = referee.shoot_data_.initial_speed;
  } else {
    bullet_speed_ = speed_limit_ - 1.0f;  // todo弹速估计
  }
  return bullet_speed_;
}

// 射速处理
void Shoot::speedHandle(void) {
  if (!fric_state_) {
    // 摩擦轮关闭
    fric_l_->setSpeed(0);
    fric_r_->setSpeed(0);
  } else {
    // 摩擦轮开启
    // 根据射速上限计算摩擦轮转速(dps) todo: 自适应射速调整
    fric_speed_ = 1e3f * speed_limit_ + 1.1e4f;
    fric_l_->setSpeed(-fric_speed_);
    fric_r_->setSpeed(fric_speed_);
  }
}

// 卡弹处理
void Shoot::blockHandle(void) {
  if (!block_state_) {
    if (stir_->motor_data_.current < -stir_block_current &&
        stir_->targetAngle() - stir_->realAngle() < -stir_block_angle) {
      // 检测到卡弹
      block_state_ = true;
      block_tick_ = HAL_GetTick();
      stir_->targetAngle() =
          math::loopLimit(stir_->targetAngle(), stir_->realAngle(),
                          stir_->realAngle() + stir_step_angle);
    }
  } else if (HAL_GetTick() - block_tick_ > 100) {
    // 从卡弹状态下恢复
    block_state_ = false;
  }
}

// 热量处理(裁判系统通信热量实时性不够，需自行计算保证不超热量)
void Shoot::heatHandle(void) {
  if (referee.connect_.check() && HAL_GetTick() - last_tick_ > 200) {
    // 距离上次发射大于200ms，用裁判系统数据更新计算热量
    calc_heat_ = referee.power_heat_data_.shooter_17mm_1_barrel_heat;
  }
  calc_heat_ = math::limitMin(calc_heat_ - cooling_rate_ * 1e-3f, 0);
  // 判断下一发是否会超热量
  if (heat_limit_ - calc_heat_ > heat_17mm_bullet) {
    heat_state_ = true;
  } else {
    heat_state_ = false;
  }
}

// 输入舵机指针
Gate::Gate(uint16_t close_pwm, uint16_t open_pwm, ServoZX361D* servo)
    : servo_(servo),
      state_(Gate::COMPLIANCE),
      close_pwm_(close_pwm),
      open_pwm_(open_pwm) {}

// 舵机初始化，设置为被动模式
void Gate::init(void) {
  servo_->init();
  state_ = Gate::COMPLIANCE;
}

// 设置舱门状态
void Gate::set(const Gate::State_e& state) {
  if (state == Gate::COMPLIANCE) {
    if (servo_->setMode(ServoZX361D::COMPLIANCE)) {
      state_ = state;
    }
  } else if (state == Gate::CLOSE) {
    if (servo_->getMode() != ServoZX361D::NORMAL) {
      servo_->setMode(ServoZX361D::NORMAL);
    } else {
      if (servo_->setPwmTime(close_pwm_, 10)) {
        state_ = state;
      }
    }
  } else if (state == Gate::OPEN) {
    if (servo_->getMode() != ServoZX361D::NORMAL) {
      servo_->setMode(ServoZX361D::NORMAL);
    } else {
      if (servo_->setPwmTime(open_pwm_, 10)) {
        state_ = state;
      }
    }
  }
}
