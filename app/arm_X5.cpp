//
// Created by 69333 on 2024/1/25.
//

#include "arm_X5.h"
#include "base/common/math.h"
#include "stm32f4xx_hal.h"
#include <cmath>
const float d1 = 128;
const float a1 = 20;
const float a2 = 270;
const float a3 = (float)sqrt(245 * 245 + 56 * 56);
const float a4 = 66.8;
float s=0;
float r=0;
float theta=atan2f(56,245);
extern float t;
extern ArmX5 arm;

float arm_traj_q_D1[4] = {80, 80, 80, 80};  // dps
float arm_traj_q_D_raise[4] = {60, 60, 60, 60};

// 构造函数
ArmX5::ArmX5(Motor* jm1, Motor* jm2, Motor* jm3, Motor* jm4, Motor* jm5,
             Motor* jm6)
    : jm1_(jm1),
      jm2_(jm2),
      jm3_(jm3),
      jm4_(jm4),
      jm5_(jm5),
      jm6_(jm6),
      mode_(STOP) {
  inter_.theta2_out_of_range = false;
  inter_.theta4_out_of_range = false;
}

void ArmX5::handle() {
  fkine(jm1_->control_data_.fdb_angle, jm2_->control_data_.fdb_angle,
        jm3_->control_data_.fdb_angle, jm4_->control_data_.fdb_angle,
        fdb_.x, fdb_.y, fdb_.z);
  if (mode_ == MANIPULATION){
    //trajPlanner();
  }
  ikine();
  if (mode_ != MANIPULATION) {
    // 更新目标位置

    ref_.x = fdb_.x;
    ref_.y = fdb_.y;
    ref_.z = fdb_.z;
  }
}

// 逆运动学
void ArmX5::ikine() {
  //检测x，y，z的合法性
  if ((ref_.x-a1)*(ref_.x-a1)+(ref_.y)*(ref_.y)+(ref_.z-d1)*(ref_.z-d1) <= pow(a2+a3,2)) {
    inter_.xyz_out_of_range = false;
  }
  else{
    inter_.xyz_out_of_range = true;
    ref_stay();
    return;
  }
  if (ref_.z<d1+56-3) {
    ref_stay();
    return;
  }
  //算坐标系中的theta1
  ik_result_.theta1=atan2f(ref_.y,ref_.x);//但是这个坐标系算出来的theta1范围和原本的一样，范围都是-150-180
  if (!(ik_result_.theta1*180/PI>=-150 && ik_result_.theta1*180/PI<=180)) {
    inter_.theta1_out_of_range=true;
    ref_stay();
    return;
  }
  else {
    inter_.theta1_out_of_range=false;
  }
  //算坐标系中的theta3
  inter_.R_=sqrtf(ref_.x * ref_.x + ref_.y * ref_.y);
  inter_.x=inter_.R_-a4*cosf(ref_.pitch*PI/180)-a1;
  inter_.y=ref_.z-a4*sinf(ref_.pitch*PI/180)-d1;
  inter_.cos_theta3=(inter_.x*inter_.x+inter_.y*inter_.y-a2*a2-a3*a3)/(2*a2*a3);
  if (std::fabs(inter_.cos_theta3)>1+0.01) {
    ref_stay();
    return;
  }
  else if (std::fabs(inter_.cos_theta3)<=1+0.005 && std::fabs(inter_.cos_theta3)>1) {
    if (inter_.cos_theta3>0) inter_.cos_theta3=1;
    else inter_.cos_theta3=-1;
  }
  ik_result_.theta3=-math::limit(fabsf(acosf(inter_.cos_theta3)),0,PI);//acos范围0-180，但是这个坐标系中的theta3范围是-180-0.
  //算坐标系中的theta2
  s=inter_.x*a3*sinf(ik_result_.theta3)-inter_.y*(a2+a3*cosf(ik_result_.theta3));
  r=inter_.x*(a2+a3*cosf(ik_result_.theta3))+inter_.y*a3*sinf(ik_result_.theta3);
  // if (fabs(r)<1) {
  //   r=-fabs(r);
  // }
  if (fabs(s)<0.1*fabs(r)){
    s=-fabs(s);
  }//同理，这个坐标系中的theta2是-180到0

  ik_result_.theta2=atan2f(s,r);
  if (!(ik_result_.theta2<=0 && ik_result_.theta2>=-PI)) {
    inter_.theta2_out_of_range=true;
    ref_stay();
    return;
  }
  else {
    inter_.theta2_out_of_range=false;
  }
  //算坐标系中的theta4
  ik_result_.theta4=ref_.pitch*PI/180+ik_result_.theta2-ik_result_.theta3;//同理，这个坐标系中的theta4是-90-90
  if (!(ik_result_.theta4<=PI/2+0.03-theta && ik_result_.theta4>=-PI/2-0.03-theta)) {
    inter_.theta4_out_of_range=true;
    ref_stay();
    return;
  }
  else {
    inter_.theta4_out_of_range=false;
  }

  ik_result_.theta1*=180/PI;
  ik_result_.theta2*=180/PI;
  ik_result_.theta3*=180/PI;
  ik_result_.theta4*=180/PI;

  target_angle_.theta1_=ik_result_.theta1;
  target_angle_.theta2_=ik_result_.theta2+180;
  target_angle_.theta3_=ik_result_.theta3+180-theta*180/PI;
  target_angle_.theta4_=ik_result_.theta4+theta*180/PI;
  target_angle_.theta5_=ref_.yaw;
  target_angle_.theta6_=ref_.row;


  // MANIPULATION模式时，更新电机目标角度
  if (mode_ == MANIPULATION) {
    jm1_->control_data_.target_angle = target_angle_.theta1_;
    jm2_->control_data_.target_angle = target_angle_.theta2_;
    jm3_->control_data_.target_angle = target_angle_.theta3_;
    jm4_->control_data_.target_angle = target_angle_.theta4_;
    jm5_->control_data_.target_angle = target_angle_.theta5_;
    jm6_->control_data_.target_angle = target_angle_.theta6_;
  }
}
// 正运动学
void ArmX5::fkine(float theta1, float theta2, float theta3, float theta4,
                  float& X_, float& Y_, float& Z_) {
  fdb_.theta1=theta1;
  fdb_.theta2=theta2-180;
  fdb_.theta3=theta3-180+theta*180/PI;
  fdb_.theta4=theta4-theta*180/PI;
  fdb_.theta1_=(fdb_.theta1/180.0*PI);
  fdb_.theta2_=(fdb_.theta2/180.0*PI);
  fdb_.theta3_=(fdb_.theta3/180.0*PI);
  fdb_.theta4_=(fdb_.theta4/180.0*PI);
  X_=cosf(fdb_.theta1_)*(a4*cosf(fdb_.theta4_+fdb_.theta3_-fdb_.theta2_)+a2*cosf(fdb_.theta2_)+a1+a3*cosf(fdb_.theta3_-fdb_.theta2_));
  Y_=sinf(fdb_.theta1_)*(a4*cosf(fdb_.theta4_+fdb_.theta3_-fdb_.theta2_)+a2*cosf(fdb_.theta2_)+a1+a3*cosf(fdb_.theta3_-fdb_.theta2_));
  Z_=a4*sinf(fdb_.theta4_+fdb_.theta3_-fdb_.theta2_)+a3*sinf(math::limit(fdb_.theta3_,-PI,PI)-math::limit(fdb_.theta2_,-PI,PI))-a2*sinf(fdb_.theta2_)+d1;
}
void ArmX5::ref_stay() {
  ref_.x = fdb_.x;
  ref_.y = fdb_.y;
  ref_.z = fdb_.z;
}

void ArmX5::trajPlanner(void) {
  if (traj_.state){
    float sigma = 1;
    if (traj_.ticks > 1){
      sigma = (float)(HAL_GetTick() - traj_.start.tick) / (float)traj_.ticks;
    }
    traj_.sigma = math::limit(sigma, 0, 1);
    if (arm.mode_ == ArmX5::MANIPULATION){
      if (traj_.method == ArmX5::Traj_t::MANIPULATION){
        ref_.x = traj_.sigma * traj_.end.x + (1 - traj_.sigma) * traj_.start.x;
        ref_.y = traj_.sigma * traj_.end.y + (1 - traj_.sigma) * traj_.start.y;
        ref_.z = traj_.sigma * traj_.end.z + (1 - traj_.sigma) * traj_.start.z;
        //这里gyh用了slerp插值，这里没有矩阵运算导致运算复杂，也对末端三轴使用线性插值
        ref_.yaw = traj_.sigma * traj_.end.yaw + (1 - traj_.sigma) * traj_.start.yaw;
        ref_.pitch = traj_.sigma * traj_.end.pitch + (1 - traj_.sigma) * traj_.start.pitch;
        ref_.row = traj_.sigma * traj_.end.roll + (1 - traj_.sigma) * traj_.start.roll;
      } else
      if (traj_.method == ArmX5::Traj_t::JOINT){
        float X_,Y_,Z_;
        float theta1 = traj_.sigma * traj_.end.theta1 + (1 - traj_.sigma) * traj_.start.theta1;
        float theta2 = traj_.sigma * traj_.end.theta2 + (1 - traj_.sigma) * traj_.start.theta2;
        float theta3 = traj_.sigma * traj_.end.theta3 + (1 - traj_.sigma) * traj_.start.theta3;
        float theta4 = traj_.sigma * traj_.end.theta4 + (1 - traj_.sigma) * traj_.start.theta4;
        fkine(theta1, theta2, theta3, theta4, X_, Y_, Z_);
        ref_.x = X_;
        ref_.y = Y_;
        ref_.z = Z_;
      }
    }
  }
}

uint32_t ArmX5::trajStart(void) {
  //设置起点位置
  traj_.start.theta1 = jm1_->control_data_.fdb_angle;
  traj_.start.theta2 = jm2_->control_data_.fdb_angle;
  traj_.start.theta3 = jm3_->control_data_.fdb_angle;
  traj_.start.theta4 = jm4_->control_data_.fdb_angle;
  traj_.start.x = fdb_.x;
  traj_.start.y = fdb_.y;
  traj_.start.z = fdb_.z;

  //设置开始时间
  traj_.start.tick = HAL_GetTick();

  if (traj_.method == ArmX5::Traj_t::JOINT) {
    // 关节运动时间
    float d_theta1 = traj_.end.theta1 - traj_.start.theta1;
    float d_theta2 = traj_.end.theta2 - traj_.start.theta2;
    float d_theta3 = traj_.end.theta3 - traj_.start.theta3;
    float d_theta4 = traj_.end.theta4 - traj_.start.theta4;
    if (traj_.q_D1[0] == 0 || traj_.q_D1[1] == 0 ||
        traj_.q_D1[2] == 0 || traj_.q_D1[3] == 0){
      return 0;
    }
    traj_.ticks = 0;
    traj_.ticks = fmax(traj_.ticks, fabs(d_theta1) / traj_.q_D1[0] * 1e3f);
    traj_.ticks = fmax(traj_.ticks, fabs(d_theta2) / traj_.q_D1[1] * 1e3f);
    traj_.ticks = fmax(traj_.ticks, fabs(d_theta3) / traj_.q_D1[2] * 1e3f);
    traj_.ticks = fmax(traj_.ticks, fabs(d_theta4) / traj_.q_D1[3] * 1e3f);
    traj_.end.tick = traj_.start.tick + traj_.ticks;
  }

  traj_.state = true;
  return traj_.ticks;
}

void ArmX5::trajSet(float theta1, float theta2, float theta3, float theta4, bool if_raise) {
  //设置终点位姿
  traj_.end.theta1 = theta1;
  traj_.end.theta2 = theta2;
  traj_.end.theta3 = theta3;
  traj_.end.theta4 = theta4;
  //设置关节角速度
  if (!if_raise){
    for (int i = 0; i < 4; i++){
      traj_.q_D1[i] = fmaxf(fabs(arm_traj_q_D1[i]), 1e-6f);
    }
  } else {
    for (int i = 0; i < 4; i++){
      traj_.q_D1[i] = fmaxf(fabs(arm_traj_q_D_raise[i]), 1e-6f);
    }
  }
}

void ArmX5::trajAbort() {
  traj_.state = false;
}
