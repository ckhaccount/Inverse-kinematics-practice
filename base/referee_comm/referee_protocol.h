/**
 ******************************************************************************
 * @file    referee_protocol.cpp/h
 * @brief   Referee communication(UART) protocol. 裁判系统通信协议
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef REFEREE_PROTOCOL_H
#define REFEREE_PROTOCOL_H

#include <stdint.h>
#include <string.h>

#ifndef __packed
#define __packed __attribute__((packed))
#endif

#define REFEREE_COMM_FRAME_MAX_SIZE 128

const uint8_t referee_comm_sof = 0xA5;

// 裁判系统通信帧头
typedef struct RefereeCommFrameHeader {
  uint8_t sof;
  uint16_t data_len;
  uint8_t seq;
  uint8_t crc8;
} __packed RefereeCommFrameHeader_t;

// 裁判系统通信一帧数据中data以外的内容
typedef struct RefereeCommFrame {
  RefereeCommFrameHeader_t header;
  uint16_t cmd_id;
  // uint8_t data[n];
  uint16_t crc16;
} __packed RefereeCommFrame_t;

// cmd_id
typedef enum RefereeCMDID {
  GAME_STATUS_ID = 0x0001,
  GAME_RESULT_ID = 0x0002,
  GAME_ROBOT_HP_ID = 0x0003,
  EVENT_DATA_ID = 0x0101,
  SUPPLY_PROJECTILE_ACTION_ID = 0x0102,
  REFEREE_WARNING_ID = 0x0104,
  DART_REMAINING_TIME_ID = 0x0105,
  ROBOT_STATUS_ID = 0x0201,
  POWER_HEAT_DATA_ID = 0x0202,
  ROBOT_POS_ID = 0x0203,
  BUFF_ID = 0x0204,
  AERIAL_SUPPORT_DATA_ID = 0x0205,
  HURT_DATA_ID = 0x0206,
  SHOOT_DATA_ID = 0x0207,
  PROJECTILE_ALLOWANCE_ID = 0x0208,
  RFID_STATUS_ID = 0x0209,
  DART_CLIENT_CMD_ID = 0x020A,
  GROUND_ROBOT_POSITION_ID = 0x20B,
  RADAR_MARK_DATA_ID = 0x20C,
  STUDENT_INTERACTIVE_DATA_ID = 0x0301,
  MAP_COMMAND_ID = 0x303,
  REMOTE_CONTROL_ID = 0x304,
  MAP_ROBOT_DATA_ID = 0x305,
  CUSTOM_CLIENT_DATA_ID = 0x306,
  MAP_SENTRY_DATA_ID = 0x307,
} RefereeCMDID_e;

// 0x0001 11 比赛状态数据，固定3Hz 频率发送 服务器→全体机器人
typedef struct {
  uint8_t game_type : 4;
  uint8_t game_progress : 4;
  uint16_t stage_remain_time;
  uint64_t SyncTimeStamp;
} __packed game_status_t;

// 0x0002 1 比赛结果数据，比赛结束触发发送 服务器→全体机器人
typedef struct {
  uint8_t winner;
} __packed game_result_t;

// 0x0003 32 机器人血量数据，固定3Hz 频率发送 服务器→全体机器人
typedef struct {
  uint16_t red_1_robot_HP;
  uint16_t red_2_robot_HP;
  uint16_t red_3_robot_HP;
  uint16_t red_4_robot_HP;
  uint16_t red_5_robot_HP;
  uint16_t red_7_robot_HP;
  uint16_t red_outpost_HP;
  uint16_t red_base_HP;
  uint16_t blue_1_robot_HP;
  uint16_t blue_2_robot_HP;
  uint16_t blue_3_robot_HP;
  uint16_t blue_4_robot_HP;
  uint16_t blue_5_robot_HP;
  uint16_t blue_7_robot_HP;
  uint16_t blue_outpost_HP;
  uint16_t blue_base_HP;
} __packed game_robot_HP_t;

// 0x0101 4 场地事件数据，固定3Hz 频率发送 服务器→己方全体机器人
// 0：未占领/未激活 1：已占领/已激活
// bit 0-2：
// • bit 0：己方补给站前补血点的占领状态，1 为已占领
// • bit 1：己方补给站左侧（面向补给站）补血点的占领状态，1 为已占领
// • bit 2：己方补给站右侧（面向补给站）的占领状态，1 为已占领
// bit 3-5：己方能量机关状态：
// • bit 3：己方能量机关激活点的占领状态，1 为已占领
// • bit 4：己方小能量机关的激活状态，1 为已激活
// • bit 5：己方大能量机关的激活状态，1 为已激活
// bit 6：己方2 号环形高地的占领状态，1 为已占领
// bit 7：己方3 号梯形高地的占领状态，1 为已占领
// bit 8：己方4 号梯形高地的占领状态，1 为已占领
// bit9-16：己方基地虚拟护盾的值（0-250）
// bit17-27：己方前哨站的血量（0-1500）
// bit28：哨兵此时是否在己方巡逻区内
// bit29-31：保留
typedef struct {
  uint32_t event_data;
} __packed event_data_t;

// 0x0102 4 补给站动作标识数据，补给站弹丸释放时触发发送 服务器→己方全体机器人
typedef struct {
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id;
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;
} __packed supply_projectile_action_t;

// 0x0104 2 裁判警告数据，己方判罚/判负时触发发送 服务器→被处罚方全体机器人
typedef struct {
  uint8_t level;
  uint8_t offending_robot_id;
} __packed referee_warning_t;

// 0x0201 27 机器人性能体系数据，固定10Hz 频率发送 主控模块→对应机器人
typedef struct {
  uint8_t dart_remaining_time;
} __packed dart_remaining_time_t;

// 比赛机器人状态：0x0201。发送频率：10Hz
typedef struct {
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t current_HP;
  uint16_t maximum_HP;
  uint16_t shooter_id1_17mm_barrel_cooling_value;
  uint16_t shooter_id1_17mm_barrel_heat_limit;
  uint16_t shooter_id1_17mm_initial_launching_speed_limit;
  uint16_t shooter_id2_17mm_barrel_cooling_valuecooling_rate;
  uint16_t shooter_id2_17mm_barrel_heatcooling_limit;
  uint16_t shooter_id2_17mm_initial_launching_speed_limit;
  uint16_t shooter_id1_42mm_barrel_cooling_value;
  uint16_t shooter_id1_42mm_barrel_heat_cooling_limit;
  uint16_t shooter_id1_42mm_initial_launching_speed_limit;
  uint16_t chassis_power_limit;
  uint8_t power_management_gimbal_output : 1;
  uint8_t power_management_chassis_output : 1;
  uint8_t power_management_shooter_output : 1;
} __packed robot_status_t;

// 0x0202 16 实时功率热量数据，固定50Hz 频率发送 主控模块→对应机器人
typedef struct {
  uint16_t chassis_voltage;
  uint16_t chassis_current;
  float chassis_power;
  uint16_t buffer_energy;
  uint16_t shooter_17mm_1_barrel_heat;
  uint16_t shooter_17mm_2_barrel_heat;
  uint16_t shooter_42mm_barrel_heat;
} __packed power_heat_data_t;

// 0x0203 16 机器人位置数据，固定10Hz 频率发送 主控模块→对应机器人
typedef struct {
  float x;
  float y;
  float z;
  float angle;
} __packed robot_pos_t;

// 0x0204 1 机器人增益数据，固定3Hz 频率发送 服务器→对应机器人
// 机器人回血增益（百分比，值为10 意为每秒回复10%最大血量）
// 机器人枪口冷却倍率（直接值，值为5 意味着5 倍冷却）
// 机器人防御增益（百分比，值为50 意为50%防御增益）
// 机器人攻击增益（百分比，值为50 意为50%攻击增益）
typedef struct {
  uint8_t recovery_buff;
  uint8_t cooling_buff;
  uint8_t defence_buff;
  uint16_t attack_buff;
} __packed buff_t;

// 0x0205 1 空中支援时间数据，固定10Hz 频率发送 服务器→己方空中机器人
typedef struct {
  uint8_t airforce_status;
  uint8_t time_remain;
} air_support_data_t;

// 0x0206 1 伤害状态数据，伤害发生后发送 主控模块→对应机器人
// bit 0-3：
//   当血量变化类型为装甲伤害，代表装甲ID，其中数值为0-4号代表机器人的五个装甲片，其他血量变化类型，该变量数值为0。
// bit 4-7：血量变化类型
//   0x0 装甲伤害扣血；
//   0x1 模块掉线扣血；
//   0x2 超射速扣血；
//   0x3 超枪口热量扣血；
//   0x4 超底盘功率扣血；
//   0x5 装甲撞击扣血
typedef struct {
  uint8_t armor_id : 4;
  uint8_t HP_deduction_reason : 4;
} __packed hurt_data_t;

// 0x0207 7 实时射击数据，弹丸发射后发送 主控模块→对应机器人
typedef struct {
  uint8_t bullet_type;
  uint8_t shooter_number;
  uint8_t launching_frequency;
  float initial_speed;
} __packed shoot_data_t;

// 0x0208 允许发弹量，固定10Hz 频率发送 服务器→己方英雄、步兵、哨兵、空中机器人
typedef struct {
  uint16_t projectile_allowance_17mm;
  uint16_t projectile_allowance_42mm;
  uint16_t remaining_gold_coin;
} __packed projectile_allowance_t;

// 0x0209 4 机器人RFID 状态，固定3Hz 频率发送 服务器→己方装有RFID模块的机器人
// bit 位值为1/0 的含义：是否已检测到该增益点RFID
// • bit0：己方基地增益点
// • bit1：己方环形高地增益点
// • bit2：对方环形高地增益点
// • bit3：己方R3/B3 梯形高地增益点
// • bit4：对方R3/B3 梯形高地增益点
// • bit5：己方R4/B4 梯形高地增益点
// • bit6：对方R4/B4 梯形高地增益点
// • bit7：己方能量机关激活点
// • bit8：己方飞坡增益点（靠近己方一侧飞坡前）
// • bit9：己方飞坡增益点（靠近己方一侧飞坡后）
// • bit10：对方飞坡增益点（靠近对方一侧飞坡前）
// • bit11：对方飞坡增益点（靠近对方一侧飞坡后）
// • bit12：己方前哨站增益点
// • bit13：己方补血点（检测到任一均视为激活）
// • bit14：己方哨兵巡逻区
// • bit15：对方哨兵巡逻区
// • bit16：己方大资源岛增益点
// • bit17：对方大资源岛增益点
// • bit18：己方控制区
// • bit19：对方控制区
// • bit20-31：保留
// 注：基地增益点，高地增益点，飞坡增益点，前哨站增益点，资源岛增益点，
// 补血点，控制区和哨兵巡逻区的RFID 仅在赛内才生效，即在赛外，即使检
// 测到对应的RFID 卡，对应值也将为0。
typedef struct {
  uint32_t rfid_status;
} __packed rfid_status_t;

// 0x020A 飞镖选手端指令数据，飞镖闸门上线后固定10Hz 频率发送
//   服务器→己方飞镖机器人
// 当前飞镖发射口的状态
// 1：关闭；
// 2：正在开启或者关闭中
// 0：已经开启
// 飞镖的打击目标，默认为前哨站；
// 0：前哨站；
// 1：基地。
typedef struct {
  uint8_t dart_launch_opening_status;
  uint8_t dart_attack_target;
  uint16_t target_change_time;
  uint16_t latest_launch_cmd_time;
} __packed dart_client_cmd_t;

// 0x020B 40 地面机器人位置数据，固定1Hz 频率发送 服务器→己方哨兵机器人
typedef struct {
  float hero_x;
  float hero_y;
  float engineer_x;
  float engineer_y;
  float standard_3_x;
  float standard_3_y;
  float standard_4_x;
  float standard_4_y;
  float standard_5_x;
  float standard_5_y;
} __packed ground_robot_position_t;

// 0x020C 6 雷达标记进度数据，固定1Hz 频率发送 服务器→己方雷达机器人
typedef struct {
  uint8_t mark_hero_progress;
  uint8_t mark_engineer_progress;
  uint8_t mark_standard_3_progress;
  uint8_t mark_standard_4_progress;
  uint8_t mark_standard_5_progress;
  uint8_t mark_sentry_progress;
} __packed radar_mark_data_t;

// 机器人间交互数据

// 0x0301 128 机器人交互数据，发送方触发发送，频率上限为10Hz
typedef struct {
  uint16_t data_cmd_id;
  uint16_t sender_id;
  uint16_t receiver_id;
  // uint8_t user_data[x];
} __packed robot_interaction_data_t;

// 机器人ID（机器人->机器人）
typedef enum RobotID {
  RED_HERO = 0x0001,
  RED_ENGINEER = 0x0002,
  RED_STANDARD3 = 0x0003,
  RED_STANDARD4 = 0x0004,
  RED_STANDARD5 = 0x0005,
  RED_AERIAL = 0x0006,
  RED_SENTRY = 0x0007,
  RED_DART = 0x0008,
  RED_RADAR = 0x0009,
  BLUE_HERO = 0x0101,
  BLUE_ENGINEER = 0x0102,
  BLUE_STANDARD3 = 0x0103,
  BLUE_STANDARD4 = 0x0104,
  BLUE_STANDARD5 = 0x0105,
  BLUE_AERIAL = 0x0106,
  BLUE_SENTRY = 0x0107,
  BLUE_DART = 0x0108,
  BLUE_RADAR = 0x0109,
} RobotID_e;

// 客户端ID（机器人->ui）
typedef enum ClientID {
  RED_HERO_CLIENT = 0x0101,
  RED_ENGINEER_CLIENT = 0x0102,
  RED_STANDARD3_CLIENT = 0x1003,
  RED_STANDARD4_CLIENT = 0x1004,
  RED_STANDARD5_CLIENT = 0x1005,
  RED_AERIAL_CLIENT = 0x0106,
  BLUE_HERO_CLIENT = 0x0165,
  BLUE_ENGINEER_CLIENT = 0x0166,
  BLUE_STANDARD3_CLIENT = 0x0167,
  BLUE_STANDARD4_CLIENT = 0x0168,
  BLUE_STANDARD5_CLIENT = 0x0169,
  BLUE_AERIAL_CLIENT = 0x016A,
} ClientID_e;

// UI

// 客户端删除图形
typedef struct {
  uint8_t operate_tpye;
  uint8_t layer;
} __packed interaction_layer_delete_t;

// 图形数据
typedef struct {
  uint8_t figure_name[3];
  uint32_t operate_type : 3;
  uint32_t figure_type : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t details_a : 9;
  uint32_t details_b : 9;
  uint32_t width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t details_c : 10;
  uint32_t details_d : 11;
  uint32_t details_e : 11;
} __packed interaction_figure_t;

// 客户端绘制二个图形
typedef struct {
  interaction_figure_t interaction_figure[2];
} __packed interaction_figure_2_t;

// 客户端绘制五个图形
typedef struct {
  interaction_figure_t interaction_figure[5];
} __packed interaction_figure_3_t;

// 客户端绘制七个图形
typedef struct {
  interaction_figure_t interaction_figure[7];
} __packed interaction_figure_4_t;

// 客户端绘制字符
typedef struct {
  interaction_figure_t grapic_data_struct;
  uint8_t data[30];
} __packed client_custom_character_t;

// 0x0302 30 自定义控制器与机器人交互数据，发送方触发发送，频率上限为30Hz
// 自定义控制器→选手端图传连接的机器人
typedef struct {
  // uint8_t data[x];
} __packed custom_robot_data_t;

// 0x0303 15 选手端小地图交互数据，选手端触发发送
// 选手端点击→服务器→发送方选择的己方机器人。
typedef struct {
  float target_position_x;
  float target_position_y;
  float target_position_z;
  uint8_t commd_keyboard;
  uint16_t target_robot_id;
} __packed map_command_t;

// 0x0304 12 键鼠遥控数据，固定30Hz 频率发送 客户端→选手端图传连接的机器人
typedef struct {
  uint16_t key_value;
  uint16_t x_position : 12;
  uint16_t mouse_left : 4;
  uint16_t y_position : 12;
  uint16_t mouse_right : 4;
  uint16_t reserved;
} __packed custom_client_data_t;

// 0x0305 10 选手端小地图接收雷达数据，频率上限为10Hz 雷达→服务器→己方所有选手端
typedef struct {
  uint16_t target_robot_id;
  float target_position_x;
  float target_position_y;
} __packed map_robot_data_t;

// 0x0306 8 自定义控制器与选手端交互数据，发送方触发发送，频率上限为30Hz
// 自定义控制器→选手端
typedef struct {
  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  int8_t left_button_down;
  int8_t right_button_down;
  uint16_t keyboard_value;
  uint16_t reserved;
} __packed remote_control_t;

// 0x0307 103 选手端小地图接收哨兵数据，频率上限为1Hz 哨兵→己方云台手选手端
typedef struct {
  uint8_t intention;
  uint16_t start_position_x;
  uint16_t start_position_y;
  int8_t delta_x[49];
  int8_t delta_y[49];
} __packed map_sentry_data_t;

#endif  // REFEREE_PROTOCOL_H
