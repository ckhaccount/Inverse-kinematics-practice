/**
 ******************************************************************************
 * @file    control.cpp/h
 * @brief   Robot control design. 机器人控制设计（模式/键位）
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <cstdint>

// 控制初始化
void controlInit(void);
// 控制主循环
void controlLoop(void);

class ArmTask{
 public:
  //状态机种类
  typedef enum Mode {
    Normal,
    Pick,
  } Mode_e;

  //任务工作状态
  typedef enum TaskState {
    IDLE,
    WORKING,
  } TaskState_e;

 public:
  // 构造函数
  ArmTask(void) {}

  // 初始化
  void reset(void);

  // 切换任务模式
  void switchMode(ArmTask::Mode_e mode);

  // 中止任务
  void abort(void);

  // 任务处理
  void handle(void);

 public:
  //任务状态
  Mode_e mode_;

  //步骤切换时间
  uint32_t finish_tick_;

  //通常（移动）
  struct Normal_t {
    // 状态
    TaskState_e state;
    uint32_t* finish_tick;
    // 步骤
    enum Step_e {
      PREPARE,       // 准备
      TRAJ_RELAY,    // 机械臂中间点
      TRAJ_RETRACT,  // 机械臂收回
    } step;

    uint8_t flag;
    // 类型
    uint8_t type;
    // 机械臂中间点关节角度
    float q_relay[4] = {0, 17, 18, 0};
    // 机械臂收回关节角度
    float q_retract[4] = {0, 4, 4, 0};

    // 处理函数
    void handle(void);
  } Normal_;

  //取矿
  struct Pick_t {
    // 状态
    TaskState_e state;
    uint32_t* finish_tick;
    // 步骤
    enum Step_e {
      PREPARE,       // 准备
      RAISE,         // 抬升
      FORWARD_1,     // 第一段前移
      PUMP_ON,       // 开气泵
      FORWARD_2,     // 第二段前移
      RAISE_MINE,    // 抬升矿
      ADJUST_MINE,     // 下降矿
      PULL_BACK,     // 回拉
      GO_DOWN,       // 手臂下降
    } step;
    // 类型
    uint8_t type;
    // 机械臂中间点关节角度
    float q_raise[4] = {0, 31, 41, 16};
    // 机械臂收回关节角度
    float q_forward_1[4] = {8.73, 77.5, 62.6, 12.08};
    float q_forward_2[4] = {8.73, 94.15, 82, 9.9};
    float q_raise_mine[4] = {8.73, 93.38, 95.12, -5};
    float t_raise_mine[3] = {353.2, 54.2, 374.03};
    float q_adjust_mine[4] = {8.73, 93.38, 94.12, -5};
    float q_pull_back[4] = {8.73, 31, 46.35, -17.5};
    float q_go_down[4] = {0, 31, 41, 16};
    // 反馈电流阈值
    float curr_fwd = 3.f;
    float curr_up = 2.f;
    // 处理函数
    void handle(void);
  } Pick_;
};

#endif  // CONTROL_H