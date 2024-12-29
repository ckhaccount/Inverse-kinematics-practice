/**
 ******************************************************************************
 * @file    power_limit.cpp/h
 * @brief   Power limitation. 功率限制
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/power_limit.h"
#include "base/common/math.h"
#include <cmath>
namespace power {

namespace m3508 {
const float k_i2 = 2e-7f;
const float k_iw = 4e-7f;
const float k_absw = 1.6e-4f;
}  // namespace m3508

}  // namespace power

float extraPower(const float& cap_volt, const float referee_buffer,
                 const float& max) {
  return math::limit((cap_volt * 1e-3f - 15) * 25.0f, 0, max) +
         math::limit(math::deadBand(referee_buffer - 40.0f, -20, 5) * 5.0f, -20,
                     50);
}

// 电机功率估算
float powerlimit::motorPower(const Motor::Type_e& type, const float& i,
                             const float& w) {
  if (type == Motor::M3508) {
    return power::m3508::k_i2 * i * i + power::m3508::k_iw * i * w +
           power::m3508::k_absw * fabs(w);
  }
  return 0;
}

MecanumChassisPower::MecanumChassisPower(MecanumChassis* chassis,
                                         RefereeComm* referee, CapComm* cap)
    : chassis_(chassis),
      referee_(referee),
      ultra_cap_(cap),
      chassis_power_fdb_filter_(5e-2f),
      speed_rate_(1),
      speed_rate_filter_(5e-2f, 1) {}

// 功率限制初始化
void MecanumChassisPower::init(void) {
  eq_.kp = chassis_->cmfl_->spid_.kp_;
  eq_.rm = fabs(chassis_->cmfl_->ratio_);
  eq_.k[0] = power::m3508::k_i2 * eq_.kp * eq_.kp;
  eq_.k[1] = -2 * power::m3508::k_i2 * eq_.kp * eq_.kp +
             power::m3508::k_iw * eq_.kp * eq_.rm;
  eq_.k[2] = power::m3508::k_i2 * eq_.kp * eq_.kp -
             power::m3508::k_iw * eq_.kp * eq_.rm;
  eq_.k[3] = power::m3508::k_absw * eq_.rm;
}

// 功率限制方程求解，返回求解功率限制方程得到的实际功率限制
void MecanumChassisPower::solve(const float& p_ref) {
  // 底盘逆运动学
  chassis_->ikine();
  // a = k0*Σ(w_ref^2)
  eq_.a = eq_.k[0] *
          (chassis_->ref_.wheel_speed.fl * chassis_->ref_.wheel_speed.fl +
           chassis_->ref_.wheel_speed.fr * chassis_->ref_.wheel_speed.fr +
           chassis_->ref_.wheel_speed.bl * chassis_->ref_.wheel_speed.bl +
           chassis_->ref_.wheel_speed.br * chassis_->ref_.wheel_speed.br);
  if (eq_.a < 100) {
    // 电机目标速度全部几乎为0，速度比例设置为1
    eq_.r = 1;
    return;
  }
  // b = k1*Σ(w_ref*w_fdb)
  eq_.b = eq_.k[1] *
          (chassis_->ref_.wheel_speed.fl * chassis_->fdb_.wheel_speed.fl +
           chassis_->ref_.wheel_speed.fr * chassis_->fdb_.wheel_speed.fr +
           chassis_->ref_.wheel_speed.bl * chassis_->fdb_.wheel_speed.bl +
           chassis_->ref_.wheel_speed.br * chassis_->fdb_.wheel_speed.br);
  // c = k2*Σ(w_fdb^2)+k3*Σ|w_ref|-P_ref
  eq_.c = eq_.k[2] *
              (chassis_->fdb_.wheel_speed.fl * chassis_->fdb_.wheel_speed.fl +
               chassis_->fdb_.wheel_speed.fr * chassis_->fdb_.wheel_speed.fr +
               chassis_->fdb_.wheel_speed.bl * chassis_->fdb_.wheel_speed.bl +
               chassis_->fdb_.wheel_speed.br * chassis_->fdb_.wheel_speed.br) +
          eq_.k[3] * (fabs(chassis_->ref_.wheel_speed.fl) +
                      fabs(chassis_->ref_.wheel_speed.fr) +
                      fabs(chassis_->ref_.wheel_speed.bl) +
                      fabs(chassis_->ref_.wheel_speed.br)) -
          p_ref;
  // Δ = b^2-4*a*c
  eq_.delta = eq_.b * eq_.b - 4.f * eq_.a * eq_.c;
  if (eq_.delta > 0) {
    // r = (-b+sqrt(Δ))/2a (目标功率下上限速度)
    eq_.r = (-eq_.b + sqrtf(eq_.delta)) / (2.f * eq_.a);
  } else {
    // r = (-b+sqrt(Δ))/2a (临界功率对应速度)
    eq_.r = -eq_.b / (2.f * eq_.a);
  }
}

// 麦轮底盘功率限制，在设置底盘速度后，解算底盘电机速度前调用
void MecanumChassisPower::handle(float extra_power_max) {
  // 设定功率限制=裁判系统功率限制+超级电容额外功率
  ref_power_limit_ =
      referee_->robot_status_.chassis_power_limit +
      extraPower(ultra_cap_->rx_msg_.cap_voltage,
                 referee_->power_heat_data_.buffer_energy, extra_power_max);

  // 估算当前底盘功率
  motor_power_fdb_.fl = powerlimit::motorPower(
      chassis_->cmfl_->info_.type, chassis_->cmfl_->motor_data_.current,
      chassis_->cmfl_->realSpeed() * chassis_->cmfl_->ratio_);
  motor_power_fdb_.fr = powerlimit::motorPower(
      chassis_->cmfr_->info_.type, chassis_->cmfr_->motor_data_.current,
      chassis_->cmfr_->realSpeed() * chassis_->cmfr_->ratio_);
  motor_power_fdb_.bl = powerlimit::motorPower(
      chassis_->cmbl_->info_.type, chassis_->cmbl_->motor_data_.current,
      chassis_->cmbl_->realSpeed() * chassis_->cmbl_->ratio_);
  motor_power_fdb_.br = powerlimit::motorPower(
      chassis_->cmbr_->info_.type, chassis_->cmbr_->motor_data_.current,
      chassis_->cmbr_->realSpeed() * chassis_->cmbr_->ratio_);
  chassis_power_fdb_ = motor_power_fdb_.fl + motor_power_fdb_.fr +
                       motor_power_fdb_.bl + motor_power_fdb_.br + 5;
  chassis_power_fdb_ = chassis_power_fdb_filter_.update(chassis_power_fdb_);

  // 计算底盘速度比例(方程求解)
  solve(ref_power_limit_);
  if (math::limit(eq_.r, 0, 1) < speed_rate_) {
    speed_rate_ = math::limit(eq_.r, 0, 1);
    speed_rate_filter_.reset(speed_rate_);
  } else {
    speed_rate_ = speed_rate_filter_.update(math::limit(eq_.r, 0, 1));
  }

  // 设置限制功率后底盘速度
  chassis_->ref_.vx *= speed_rate_;
  chassis_->ref_.vy *= speed_rate_;
  chassis_->ref_.wz *= speed_rate_;

  // 功率限制后的估计功率
  chassis_->ikine();
  motor_power_est_.fl = powerlimit::motorPower(
      chassis_->cmfl_->info_.type,
      eq_.kp * (chassis_->ref_.wheel_speed.fl - chassis_->fdb_.wheel_speed.fl),
      chassis_->fdb_.wheel_speed.fl * chassis_->cmfl_->ratio_);
  motor_power_est_.fr = powerlimit::motorPower(
      chassis_->cmfr_->info_.type,
      eq_.kp * (chassis_->ref_.wheel_speed.fr - chassis_->fdb_.wheel_speed.fr),
      chassis_->fdb_.wheel_speed.fr * chassis_->cmfr_->ratio_);
  motor_power_est_.bl = powerlimit::motorPower(
      chassis_->cmbl_->info_.type,
      eq_.kp * (chassis_->ref_.wheel_speed.bl - chassis_->fdb_.wheel_speed.bl),
      chassis_->fdb_.wheel_speed.bl * chassis_->cmbl_->ratio_);
  motor_power_est_.br = powerlimit::motorPower(
      chassis_->cmbr_->info_.type,
      eq_.kp * (chassis_->ref_.wheel_speed.br - chassis_->fdb_.wheel_speed.br),
      chassis_->fdb_.wheel_speed.br * chassis_->cmbr_->ratio_);
  chassis_power_est_ = motor_power_est_.fl + motor_power_est_.fr +
                       motor_power_est_.bl + motor_power_est_.br + 5;
}
