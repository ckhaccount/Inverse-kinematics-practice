//
// Created by 69333 on 2024/1/25.
//

#ifndef RM_FRAME_ARM_X5_H
#define RM_FRAME_ARM_X5_H

#include "base/motor/motor.h"

const float JM_ANGLE_THRESHOLD = 2.5;

class ArmX5 {
 public:
  // 构造函数
  ArmX5(Motor* jm1, Motor* jm2, Motor* jm3, Motor* jm4, Motor* jm5, Motor* jm6);

  void handle();

  // 逆运动学
  void ikine();
  // 正运动学
  void fkine(float theta1, float theta2, float theta3, float theta4,
             float& X_, float& Y_, float& Z_);

  //轨迹规划
  void trajPlanner(void);
  //开始轨迹
  uint32_t trajStart(void);
  //设置轨迹终点
  void trajSet(float theta1, float theta2, float theta3, float theta4, bool if_raise);
  //终止轨迹
  void trajAbort();

  // 目标值停留在当前位置
  void ref_stay();

 public:
  Motor *jm1_, *jm2_, *jm3_, *jm4_, *jm5_, *jm6_;

  // 工作模式
  enum Mode_e {
    STOP,
    MANIPULATION,
    JOINT,
  } mode_;

  // 目标位置
  struct ref {
    float x;
    float y;
    float z;
    float pitch;
    float yaw;
    float row;
  } ref_;

  // 实际位置(根据实际角度正运动学得出)
  struct fdb {
    float x;
    float y;
    float z;
    // deg
    float theta1;
    float theta2;
    float theta3;
    float theta4;
    // rad
    float theta1_;
    float theta2_;
    float theta3_;
    float theta4_;
  } fdb_;

  // 用于debug 逆运动学中间过程量
  struct inter {
    float R_;
    float theta0;
    float x;
    float y;
    float cos_theta3;
    float k1;
    float k2;
    bool theta2_out_of_range;
    bool theta4_out_of_range;
    bool theta1_out_of_range;
    bool xyz_out_of_range;
  } inter_;

  // 用于debug 逆运动学解算结果
  struct ik_result {
    float theta1;
    float theta2;
    float theta3;
    float theta4;
  } ik_result_;

  // 用于debug 发给电机的目标角度
  struct target_angle {
    float theta1_;
    float theta2_;
    float theta3_;
    float theta4_;
    float theta5_;
    float theta6_;
  } target_angle_;

  struct Traj_t {
    // 轨迹规划运行状态
    bool state;

    // 轨迹规划方法(工作空间/关节空间插值)
    enum Method_e {
      MANIPULATION,
      JOINT,
    } method;

    // 轨迹起点
    struct Start_t {
      float x, y, z;           // m
      float yaw, pitch, roll;  // rad
      float theta1, theta2, theta3, theta4; //rad
      uint32_t tick;
    } start;

    // 轨迹终点
    struct End_t {
      float x, y, z;           // m
      float yaw, pitch, roll;  // rad
      float theta1, theta2, theta3, theta4; //rad
      uint32_t tick;
    } end;

    // 速度
    float speed;         // 速度(m/s)
    float rotate_speed;  // 角速度(rad/s)
    float q_D1[4];  // 关节角速度(rad/s)

    // 时间相关变量
    uint32_t ticks;
    float sigma;

  } traj_;

  int user = 0;
};

#endif  // RM_FRAME_ARM_X5_H
