/**
 ******************************************************************************
 * @file    control.cpp/h
 * @brief   Robot control design. 机器人控制设计（模式/键位）
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/control.h"

#include "iwdg.h"

#include "app/arm_X5.h"
#include "app/autoaim.h"
#include "app/chassis.h"
#include "app/gimbal.h"
#include "app/imu_monitor.h"
#include "app/motor_monitor.h"
#include "app/power_limit.h"
#include "app/shoot.h"
#include "base/bsp/bsp_buzzer.h"
#include "base/bsp/bsp_led.h"
#include "base/cap_comm/cap_comm.h"
#include "base/common/math.h"
#include "base/cv_comm/cv_comm.h"
#include "base/remote/remote.h"
#include "base/remote/wfly.h"
#include "base/servo/servo.h"
#include <cmath>
void iwdgHandler(bool iwdg_refresh_flag);
void robotPowerStateFSM(bool stop_flag);
void robotReset(void);
bool robotStartup(void);
void robotControl(void);
void boardLedHandle(void);
float t=1;
#ifdef RF206S
extern WFLY rc;
#elif defined(DR16)
extern RC rc;
#endif
extern IMU board_imu;
extern CVComm cv_comm;
extern ServoZX361D gate_servo;
extern CapComm ultra_cap;
extern RefereeComm referee;

Gimbal gimbal(&GMY, &GMP, &board_imu);
MecanumChassis chassis(&CMFL, &CMFR, &CMBL, &CMBR, PID(5, 0, 10, 100, 240),
                       LowPassFilter(2e-2f));
MecanumChassisPower power_limit(&chassis, &referee, &ultra_cap);
Shoot shoot(&FRICL, &FRICR, &STIR);
Gate gate(1580, 800, &gate_servo);
Autoaim autoaim;
BoardLed led;
ArmX5 arm(&JM1, &JM2, &JM3, &JM4, &JM5, &JM6);
ArmTask task;

// 上电状态
enum RobotPowerState_e {
  STOP = 0,
  STARTUP = 1,
  WORKING = 2,
} robot_state;
// 初始化标志
bool robot_init_flag = false;
// 遥控器挡位记录
#ifdef RF206S
WFLY::RCSwitch last_rc_switch;
#endif
#ifdef DR16
RC::RCSwitch last_rc_switch;
#endif
// 额外功率
float extra_power_max = 0;

// 遥控器控制
namespace rcctrl {
const float chassis_speed_rate = 6e-3f;
const float chassis_rotate_rate = 0.8f;
const float gimbal_rate = 6e-4f;
const float chassis_follow_ff_rate = 0.3f;
}  // namespace rcctrl

// 键鼠控制
namespace kbctrl {
const float chassis_speed_rate = 6e-3f;
const float chassis_rotate_rate = 0.8f;
const float gimbal_rate = 6e-4f;
const float chassis_follow_ff_rate = 0.3f;
}  // namespace kbctrl

// 弹舱参数
namespace gateparam {
const uint16_t pwm_close = 1530;
const uint16_t pwm_open = 600;
const uint16_t time = 10;
}  // namespace gateparam

// 控制初始化
void controlInit(void) {
  *RC_UART.Init.BaudRate = 100000;
  *RC_UART.Init.WordLength = UART_WORDLENGTH_9B;
#ifdef RF206S
  *RC_UART.Init.StopBits = UART_STOPBITS_2;
#endif
#ifdef DR16
  *RC_UART.Init.StopBits = UART_STOPBITS_1;
#endif
  *RC_UART.Init.Parity = UART_PARITY_EVEN;
  *RC_UART.Init.Mode = UART_MODE_RX;
  *RC_UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  *RC_UART.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(RC_UART) != HAL_OK)
  {
      Error_Handler();
  }
  led.init();
  gate.init();
  power_limit.init();
  robot_state = STOP;
}

// 控制主循环
void controlLoop(void) {
  iwdgHandler(rc.connect_.check());
  robotPowerStateFSM(!rc.connect_.check() || rc.switch_.r == RC::DOWN);

  if (robot_state == STOP) {
    allMotorsStopShutoff();
    robotReset();
  } else if (robot_state == STARTUP) {
    allMotorsOn();                     // 电机上电
    robotStartup();
    robot_init_flag = robotStartup();  // 开机状态判断
  } else if (robot_state == WORKING) {
    allMotorsOn();   // 电机上电
    robotControl();  // 机器人控制
  }
  chassis.rotateHandle(math::degNormalize180(
      -(GMY.motor_data_.ecd_angle - yaw_zero_ecd) / GMY.ratio_));
  power_limit.handle(extra_power_max);
  chassis.handle();
  gimbal.handle();
  shoot.handle();
  autoaim.handle();
  cv_comm.handle();
  boardLedHandle();
  arm.handle();
}

// IWDG处理，true持续刷新，false进入STOP状态并停止刷新
void iwdgHandler(bool iwdg_refresh_flag) {
  if (!iwdg_refresh_flag) {
    robot_state = STOP;
  } else {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

// STOP(断电，安全模式)/开机/正常运行 状态机
void robotPowerStateFSM(bool stop_flag) {
  if (robot_state == STOP) {
    if (!stop_flag) {
      robot_state = STARTUP;
    }
  } else if (robot_state == STARTUP) {
    if (stop_flag) {
      robot_state = STOP;
    } else if (robot_init_flag) {
      // 初始化/复位完成
      robot_state = WORKING;
    }
  } else if (robot_state == WORKING) {
    if (stop_flag) {
      robot_state = STOP;
    }
  }
}

// 重置各功能状态
void robotReset(void) {
  robot_init_flag = false;

  arm.mode_ = ArmX5::STOP;
  gimbal.setMode(IMU_MODE);
  shoot.fricOff();
  autoaim.setState(false, false);
  cv_comm.mode_ = CVMode::AUTOAIM;

  last_rc_switch = rc.switch_;
  task.reset();
}

// 开机上电启动处理
bool robotStartup(void) {
  bool flag = true;

  arm.ref_.x = arm.fdb_.x;
  arm.ref_.y = arm.fdb_.y;
  arm.ref_.z = arm.fdb_.z;
  arm.ref_.yaw = 0;
  arm.ref_.row = 0;

  JM1.control_data_.target_angle = 0;
  JM2.control_data_.target_angle = 0;

  JM4.control_data_.target_angle = 0;
  JM6.control_data_.target_angle = 0;
  if (fabs(JM4.control_data_.target_angle - JM4.realAngle()) > JM_ANGLE_THRESHOLD) {
    flag = false;
  } else {
    JM5.control_data_.target_angle = 0;
    JM3.control_data_.target_angle = 0;
    arm.ref_.pitch = JM4.realAngle();
  }
  if (fabs(JM1.control_data_.target_angle - JM1.realAngle()) > JM_ANGLE_THRESHOLD) {
    flag = false;
  }
  if (fabs(JM2.control_data_.target_angle - JM2.realAngle()) > JM_ANGLE_THRESHOLD) {
    flag = false;
  }
  if (fabs(JM3.control_data_.target_angle - JM3.realAngle()) > JM_ANGLE_THRESHOLD) {
    flag = false;
  }
  if (fabs(JM5.control_data_.target_angle - JM5.realAngle()) > JM_ANGLE_THRESHOLD) {
    flag = false;
  }
  if (fabs(JM6.control_data_.target_angle - JM6.realAngle()) > JM_ANGLE_THRESHOLD) {
    flag = false;
  }
  return flag;
}

// 机器人控制
void robotControl(void) {
  // 遥控器挡位左上右上
  if (rc.switch_.l == RC::UP && rc.switch_.r == RC::UP) {
    // 模式: 各轴分别控制
    arm.mode_ = ArmX5::JOINT;

    if (rc.channel_.l_col > 400) {
      JM4.control_data_.target_angle += 8e-2;
    } else if (rc.channel_.l_col < -400) {
      JM4.control_data_.target_angle -= 8e-2;
    }
    if (rc.channel_.l_row > 400) {
      JM5.control_data_.target_angle += 8e-2;
    } else if (rc.channel_.l_row < -400) {
      JM5.control_data_.target_angle -= 8e-2;
    }

    if (rc.channel_.r_row > 400) {
      JM6.control_data_.target_angle += 8e-2;
    } else if (rc.channel_.r_row < -400) {
      JM6.control_data_.target_angle -= 8e-2;
    }

    // 开关泵
    if (rc.channel_.dial_wheel > 100) {
      PUMP.setSpeed(0);
    } else if (rc.channel_.dial_wheel < -100) {
      PUMP.setSpeed(10000);
    }
  }
  // 遥控器挡位左中右上
  else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::UP) {
    // 模式: 直接控制末端位置
    arm.mode_ = ArmX5::MANIPULATION;

    if (rc.channel_.l_col > 400) {
      arm.ref_.z += 2e-2;
      //t+=15e-2;
    } else if (rc.channel_.l_col < -400) {
      arm.ref_.z -= 2e-2;
      //t-=15e-2;
    }
    if (rc.channel_.l_row > 400) {
      arm.ref_.y -= 2e-2;
    } else if (rc.channel_.l_row < -400) {
      arm.ref_.y += 2e-2;
    }

    if (rc.channel_.r_col > 400) {
      arm.ref_.x += 2e-2;
    } else if (rc.channel_.r_col < -400) {
      arm.ref_.x -= 2e-2;
    }
    if (rc.channel_.r_row > 400) {
      arm.ref_.y -= 2e-2;
    } else if (rc.channel_.r_row < -400) {
      arm.ref_.y += 2e-2;
    }

    // 开关泵
    if (rc.channel_.dial_wheel > 100) {
      PUMP.setSpeed(0);
    } else if (rc.channel_.dial_wheel < -100) {
      PUMP.setSpeed(10000);
    }
  }
  // 遥控器挡位左下右上
  else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::UP) {
    //任务模式
    arm.mode_ = ArmX5::MANIPULATION;
    if (rc.channel_.l_col > 500){
      task.switchMode(ArmTask::Mode_e::Normal);
    } else
    if (rc.channel_.r_col > 500){
      task.switchMode(ArmTask::Mode_e::Pick);
    }
    task.handle();
    // 开关泵
    if (rc.channel_.dial_wheel > 100) {
        PUMP.setSpeed(0);
    } else if (rc.channel_.dial_wheel < -100) {
        PUMP.setSpeed(10000);
    }
  }
  // 遥控器挡位左上右中
  else if (rc.switch_.l == RC::UP && rc.switch_.r == RC::MID) {
    // 模式: 各轴分别控制
    arm.mode_ = ArmX5::JOINT;

    if (rc.channel_.l_col > 400) {
      JM2.control_data_.target_angle += 1e-2;
    } else if (rc.channel_.l_col < -400) {
      JM2.control_data_.target_angle -= 1e-2;
    }
    if (rc.channel_.l_row > 400) {
      JM1.control_data_.target_angle -= 2e-2;
    } else if (rc.channel_.l_row < -400) {
      JM1.control_data_.target_angle += 2e-2;
    }

    if (rc.channel_.r_col > 400) {
      JM3.control_data_.target_angle += 8e-3;
    } else if (rc.channel_.r_col < -400) {
      JM3.control_data_.target_angle -= 8e-3;
    }

    // 开关泵
    if (rc.channel_.dial_wheel > 100) {
      PUMP.setSpeed(0);
    } else if (rc.channel_.dial_wheel < -100) {
      PUMP.setSpeed(10000);
    }
  }
  // 遥控器挡位左中右中
  else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::MID) {
    // 模式: 直接控制末端位置
    arm.mode_ = ArmX5::MANIPULATION;

    if (rc.channel_.l_col > 400) {
      arm.ref_.pitch += 1e-2;
    } else if (rc.channel_.l_col < -400) {
      arm.ref_.pitch -= 1e-2;
    }
    if (rc.channel_.l_row > 400) {
      arm.ref_.yaw += 2e-2;
    } else if (rc.channel_.l_row < -400) {
      arm.ref_.yaw -= 2e-2;
    }

    if (rc.channel_.r_col > 400) {
      arm.ref_.pitch += 1e-2;
    } else if (rc.channel_.r_col < -400) {
      arm.ref_.pitch -= 1e-2;
    }
    if (rc.channel_.r_row > 400) {
      arm.ref_.row -= 2e-2;
    } else if (rc.channel_.r_row < -400) {
      arm.ref_.row += 2e-2;
    }

    // 开关泵
    if (rc.channel_.dial_wheel > 100) {
      PUMP.setSpeed(0);
    } else if (rc.channel_.dial_wheel < -100) {
      PUMP.setSpeed(10000);
    }
  }
  // 遥控器挡位左下右中
  else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::MID) {
    arm.mode_ = ArmX5::STOP;
  }

  // 记录遥控器挡位状态
  last_rc_switch = rc.switch_;
}

// 板载LED指示灯效果
void boardLedHandle(void) {
#ifdef DBC
  if (robot_state == STOP) {
    led.setColor(255, 0, 0);  // red
    led.setModeOn();
  } else if (robot_state == STARTUP) {
    led.setColor(150, 150, 0);  // yellow
    led.setModeBreath();
  } else if (robot_state == WORKING) {
    led.setColor(0, 0, 255);  // blue
    if (rc.switch_.l == RC::UP && rc.switch_.r == RC::UP) {
      // 遥控器挡位左上右上
      led.setModeBlink(1);
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::UP) {
      // 遥控器挡位左中右上
      led.setModeBlink(2);
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::UP) {
      // 遥控器挡位左下右上
      led.setModeBlink(3);
    } else if (rc.switch_.l == RC::UP && rc.switch_.r == RC::MID) {
      // 遥控器挡位左上右中
      led.setModeBlink(4);
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::MID) {
      // 遥控器挡位左中右中
      led.setModeBlink(5);
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::MID) {
      // 遥控器挡位左下右中
      led.setModeBlink(6);
    }
  }
  led.handle();
#elif defined DBA
  if (robot_state == STOP) {
    led.setLED(true, false, 0);  // red
  } else if (robot_state == STARTUP) {
    led.setLED(true, true, 0);  // red+green
  } else if (robot_state == WORKING) {
    if (rc.switch_.l == RC::UP && rc.switch_.r == RC::UP) {
      // 遥控器挡位左上右上
      led.setLED(false, true, 1);  // green
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::UP) {
      // 遥控器挡位左中右上
      led.setLED(false, true, 2);  // green
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::UP) {
      // 遥控器挡位左下右上
      led.setLED(false, true, 3);  // green
    } else if (rc.switch_.l == RC::UP && rc.switch_.r == RC::MID) {
      // 遥控器挡位左上右中
      led.setLED(false, true, 4);  // green
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::MID) {
      // 遥控器挡位左中右中
      led.setLED(false, true, 5);  // green
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::MID) {
      // 遥控器挡位左下右中
      led.setLED(false, true, 6);  // green
    }
  }
#endif
}

void ArmTask::reset(void) {
  mode_ = Mode::Normal;

  Normal_.state = TaskState::IDLE;
  Pick_.state = TaskState::IDLE;

  Normal_.step = Normal_t::PREPARE;
  Pick_.step = Pick_t::PREPARE;

  Normal_.finish_tick = &finish_tick_;
  Pick_.finish_tick = &finish_tick_;
}

void ArmTask::switchMode(ArmTask::Mode_e mode) {
  if (mode == Normal){
    mode_ = mode;
    Normal_.state = WORKING;
    Normal_.step = Normal_t::PREPARE;
    finish_tick_ = HAL_GetTick() + 10;
    //重置任务状态
    //Normal_.state = IDLE;
    Pick_.state = IDLE;
  } else
  if (mode == Pick){
    mode_ = mode;
    Pick_.state = WORKING;
    Pick_.step = Pick_t::PREPARE;
    finish_tick_ = HAL_GetTick() + 10;
    //重置任务状态
    Normal_.state = IDLE;
    //Pick_.state = IDLE;
  }
}

void ArmTask::abort(void) {
  Normal_.state = IDLE;
  Pick_.state = IDLE;
}

void ArmTask::handle(void) {
  arm.mode_ = ArmX5::Mode_e::MANIPULATION;
  Normal_.handle();
  Pick_.handle();
}

void ArmTask::Normal_t::handle() {
  if (state == IDLE){
    return;
  }
  if (step == PREPARE){
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_RELAY;
      arm.traj_.method = ArmX5::Traj_t::JOINT;
      arm.trajSet(q_relay[0], q_relay[1],
                  q_relay[2], q_relay[3], false);
      *finish_tick = HAL_GetTick() + arm.trajStart();
    }
  } else
  if (step == TRAJ_RELAY){
    if (HAL_GetTick() > *finish_tick){
      step = TRAJ_RETRACT;
      arm.traj_.method = ArmX5::Traj_t::JOINT;
      arm.trajSet(q_retract[0], q_retract[1],
                  q_retract[2], q_retract[3], false);
      *finish_tick = HAL_GetTick() + arm.trajStart();
    }
  } else
  if (step == TRAJ_RETRACT){
    if (HAL_GetTick() > *finish_tick){
      arm.trajAbort();
    }
  }
}

void ArmTask::Pick_t::handle(void) {
  if (state == IDLE){
    return;
  }
  // TODO: 取矿状态机
}
