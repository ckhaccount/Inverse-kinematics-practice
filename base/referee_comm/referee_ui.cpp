//
// Created by yixin on 2023/2/5.
//

#include "base/referee_comm/referee_ui.h"
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include "base/common/crc.h"
#include "base/referee_comm/referee_comm.h"
#include "base/referee_comm/referee_protocol.h"

#ifdef UI_TEST_ON_COMPUTER
#include "UI.h"
#include "main.h"
#endif

extern UI ui;

//// UI version 3.5

bool compareGraphicData(interaction_figure_t gd1, interaction_figure_t gd2) {
  bool r = true;
  if (gd1.color != gd2.color || gd1.details_a != gd2.details_a ||
      gd1.details_b != gd2.details_b || gd1.width != gd2.width ||
      gd1.start_x != gd2.start_x || gd1.start_y != gd2.start_y ||
      gd1.details_d != gd2.details_d || gd1.details_e != gd2.details_e ||
      gd1.details_c != gd2.details_c || gd1.layer != gd2.layer)
    r = false;
  return r;
}

UIPoint polar2Point(UIPolarVec vec) {
  int16_t x = cosf(vec.theta) * vec.r;
  int16_t y = sinf(vec.theta) * vec.r;
  return {x, y};
};

Circle::Circle(const char figure_name[3], u8 layer, Color color, u8 width,
               u16 center_x, u16 center_y, u16 details_c) {
  graph_data_.figure_name[0] = figure_name[0];
  graph_data_.figure_name[1] = figure_name[1];
  graph_data_.figure_name[2] = figure_name[2];
  graph_data_.operate_type = NO_OPERATE;
  graph_data_.figure_type = CIRCLE;
  graph_data_.layer = layer;
  graph_data_.color = color;
  graph_data_.width = width;
  graph_data_.start_x = center_x;
  graph_data_.start_y = center_y;
  graph_data_.details_c = details_c;
}

Rect::Rect(const char figure_name[3], u8 layer, Color color, u8 width, u16 x1,
           u16 y1, u16 x2, u16 y2) {
  graph_data_.figure_name[0] = figure_name[0];
  graph_data_.figure_name[1] = figure_name[1];
  graph_data_.figure_name[2] = figure_name[2];
  graph_data_.operate_type = NO_OPERATE;
  graph_data_.figure_type = RECT;
  graph_data_.layer = layer;
  graph_data_.color = color;
  graph_data_.width = width;
  graph_data_.start_x = x1;
  graph_data_.start_y = y1;
  graph_data_.details_d = x2;
  graph_data_.details_e = y2;
}

Line::Line(const char figure_name[3], u8 layer, Color color, u8 width, u16 x1,
           u16 y1, u16 x2, u16 y2) {
  graph_data_.figure_name[0] = figure_name[0];
  graph_data_.figure_name[1] = figure_name[1];
  graph_data_.figure_name[2] = figure_name[2];
  graph_data_.operate_type = NO_OPERATE;
  graph_data_.figure_type = LINE;
  graph_data_.layer = layer;
  graph_data_.color = color;
  graph_data_.width = width;
  graph_data_.start_x = x1;
  graph_data_.start_y = y1;
  graph_data_.details_d = x2;
  graph_data_.details_e = y2;
}

Ellipse::Ellipse(const char figure_name[3], u8 layer, Color color, u8 width,
                 u16 center_x, u16 center_y, u16 ax, u16 ay) {
  graph_data_.figure_name[0] = figure_name[0];
  graph_data_.figure_name[1] = figure_name[1];
  graph_data_.figure_name[2] = figure_name[2];
  graph_data_.operate_type = NO_OPERATE;
  graph_data_.figure_type = ELLIPSE;
  graph_data_.layer = layer;
  graph_data_.color = color;
  graph_data_.width = width;
  graph_data_.start_x = center_x;
  graph_data_.start_y = center_y;
  graph_data_.details_d = ax;
  graph_data_.details_e = ay;
}

Arc::Arc(const char figure_name[3], u8 layer, Color color, u8 width,
         u16 center_x, u16 center_y, u16 ax, u16 ay, u16 angle_st,
         u16 angle_ed) {
  graph_data_.figure_name[0] = figure_name[0];
  graph_data_.figure_name[1] = figure_name[1];
  graph_data_.figure_name[2] = figure_name[2];
  graph_data_.operate_type = NO_OPERATE;
  graph_data_.figure_type = ARC;
  graph_data_.layer = layer;
  graph_data_.color = color;
  graph_data_.width = width;
  graph_data_.start_x = center_x;
  graph_data_.start_y = center_y;
  graph_data_.details_d = ax;
  graph_data_.details_e = ay;
  graph_data_.details_a = angle_st;
  graph_data_.details_b = angle_ed;
}

IntNum::IntNum(const char figure_name[3], u8 layer, Color color, u8 width,
               u8 size, u16 x, u16 y, int32_t num) {
  graph_data_.figure_name[0] = figure_name[0];
  graph_data_.figure_name[1] = figure_name[1];
  graph_data_.figure_name[2] = figure_name[2];
  graph_data_.operate_type = NO_OPERATE;
  graph_data_.figure_type = INT;
  graph_data_.layer = layer;
  graph_data_.color = color;
  graph_data_.width = width;
  graph_data_.details_a = size;
  graph_data_.start_x = x;
  graph_data_.start_y = y;
  graph_data_.details_e = num >> 21;
  graph_data_.details_d = (num >> 10) & 0x07ff;
  graph_data_.details_c = num & 0x03ff;
}

FloatNum::FloatNum(const char figure_name[3], u8 layer, Color color, u8 width,
                   u8 size, u8 effective_digit, u16 x, u16 y, float num) {
  auto num_times_1000 = (uint32_t)(num * 1000);
  graph_data_.figure_name[0] = figure_name[0];
  graph_data_.figure_name[1] = figure_name[1];
  graph_data_.figure_name[2] = figure_name[2];
  graph_data_.operate_type = NO_OPERATE;
  graph_data_.figure_type = FLOAT;
  graph_data_.layer = layer;
  graph_data_.color = color;
  graph_data_.width = width;
  graph_data_.details_a = size;
  graph_data_.details_b = effective_digit;
  graph_data_.start_x = x;
  graph_data_.start_y = y;
  graph_data_.details_e = num_times_1000 >> 21;
  graph_data_.details_d = (num_times_1000 >> 10) & 0x07ff;
  graph_data_.details_c = (num_times_1000)&0x03ff;
}

/**
 ******************************************************************************
 * @brief  Draw a string no more than 30 characters
 * @note   This string can only be drawn by data ID "0x0110", though formatted
 *         output is supported, using this function will waste precious 10hz
 *limit
 ******************************************************************************
 **/
Str::Str(const char figure_name[3], u8 layer, Color color, u8 size, u8 len,
         u8 width, u16 x, u16 y, const char* __restrict format, ...) {
  graph_data_.figure_name[0] = figure_name[0];
  graph_data_.figure_name[1] = figure_name[1];
  graph_data_.figure_name[2] = figure_name[2];
  graph_data_.operate_type = NO_OPERATE;
  graph_data_.figure_type = STRING;  // character
  graph_data_.layer = layer;
  graph_data_.color = color;
  graph_data_.details_a = size;
  graph_data_.details_b = len;
  graph_data_.width = width;
  graph_data_.start_x = x;
  graph_data_.start_y = y;
  graph_data_.details_c = 0;
  graph_data_.details_d = 0;
  graph_data_.details_e = 0;
  content_ = new char[30];
  memset(content_, 0, 30);
  va_list args;
  va_start(args, format);
  vsprintf(content_, format, args);
  va_end(args);
}

Graph* Graph::setRelativePos(const char target[3], u16 relative_x,
                             u16 relative_y, bool if_start_point) {
  auto data = UI::getGraphData(target);
  return this->setPos(data.start_x + relative_x, data.start_y + relative_y,
                      if_start_point);
}
Graph* Graph::setRelativePos(const char target[3], uint16_t (*func1)(uint16_t),
                             uint16_t (*func2)(uint16_t), bool if_start_point) {
  auto data = UI::getGraphData(target);
  return this->setPos(func1(data.start_x), func2(data.start_y), if_start_point);
}
Graph* Graph::setFollowColor(const char name[3]) {
  auto data = ui.getGraphData(name);
  this->graph_data_.color = data.color;
  return this;
}

inline bool compareName(const unsigned char* s1, const unsigned char* s2) {
  for (int i = 0; i < 3; ++i) {
    if (s1[i] != s2[i])
      return false;
  }
  return true;
}

UI& UI::add(Graph* graph) {
  auto* node = head_;
  while (node != nullptr) {
    if (compareName(node->graph->getGraphicName(), graph->getGraphicName())) {
      // 此处如果图形原始数据发生改变则直接覆盖，且 Graph 类附带的属性则不会变化
      // 但是如果只是 Graph 类的属性发生了变化（比如改变了 priority
      // 计算规则）但是不修改图形，则此函数不会起作用
      if (compareGraphicData(node->graph->getGraphData(),
                             graph->getGraphData())) {
        // 完全一样的图形
        delete graph;
        return *this;
      }
      if (node->graph->getDrawProperty().drawn) {
        // 画过的图形，而且图形内容发生了变化，需要修改(modify)
        node->graph->setGraphData(graph->getGraphData());
        node->graph->setOperateType(MODIFY);
      }
      delete graph;
      return *this;
    }
    node = node->next;
  }
  // 没画过的图形，添加(add)
  graph->setOperateType(ADD);
  addNode(graph);

  return *this;
}

UI& UI::del(const char figure_name[3]) {
  auto* node = head_;
  while (node != nullptr) {
    uint8_t* it_name = node->graph->getGraphicName();
    for (int i = 0; i < 3; ++i) {
      if (it_name[i] != figure_name[i])
        break;
      if (i == 2) {
        node->graph->setOperateType(DELETE);
        return *this;
      }
    }
    node = node->next;
  }
  return *this;
}

interaction_figure_t UI::getGraphData(const char name[3]) {
  auto* node = ui.fineNode(name);
  if (node == nullptr) {
    return {0};
  }
  return node->graph->getGraphData();
}

/**
 * @brief add the index of elements by one
 * @param graphs
 * @param start_index
 * @param num the overall size of the list, we need this param because the
 * actual size of list is always 7, but may only need less elements note that
 * this param is not the number of element you want to move but the used size of
 * list
 */
inline void moveGraphs(GraphListNode** graphs, uint8_t start_index,
                       uint8_t num) {
  for (uint8_t i = num - 1; i >= start_index; --i) {
    graphs[i] = graphs[i - 1];
  }
}

void UI::graphManagerHandle() {
  if (head_ == nullptr)
    return;

  static auto** first_seven_graphs = new GraphListNode*[7];

  GraphListNode* node = head_;

  while (node->graph->getGraphData().figure_type == STRING &&
         node->next != nullptr)
    node = node->next;  // 防止string把队列卡死

  uint8_t this_draw_nums =
      graph_num_ < draw_nums
          ? graph_num_
          : draw_nums;  // 节约带宽，同时避免删除操作被多次执行

  for (uint8_t i = 0; i < this_draw_nums; ++i) {
    first_seven_graphs[i] = node;
    node = getCircularNextNode(
        node);  // 如果前七个全都是优先级最高的，那排序就会失效，而往往最后加入的优先级很高（最后加入的会变成head）
  }

  node = head_;

  while (node != nullptr) {
    node->graph->calcThisPriority();
    node->graph->addCountSinceLastUpdate();
    if (node->graph->getThisPriority() < 0) {
      node = node->next;
      continue;
    }                                           // priority 小于0不予处理
    for (int i = 0; i < this_draw_nums; ++i) {  // 排序排出前n个，取决于用户设定
      if (compareName(node->graph->getGraphicName(),
                      first_seven_graphs[i]->graph->getGraphicName()))
        break;
      if (node->graph->getThisPriority() >
          first_seven_graphs[i]->graph->getThisPriority()) {
        moveGraphs(first_seven_graphs, i + 1, this_draw_nums);
        first_seven_graphs[i] = node;
        break;
      }
    }
    node = node->next;
  }

  // 打包数据
  for (int i = 0; i < this_draw_nums; ++i) {
    node = first_seven_graphs[i];
    if (node->graph->getGraphData().figure_type == STRING) {
      drawStr(node->graph->getGraphData(), node->graph->getCharContent());
#ifdef UI_TEST_ON_COMPUTER
      log_graph_info(node);
#endif
      node->graph->setDrawn();
      if (node->graph->getGraphData().operate_type == DELETE) {
        delete node->graph;
        delNode(node);
        //// TODO: 防止丢包
      }
      return;
    }
  }

  for (int i = 0; i < this_draw_nums; ++i) {
    node = first_seven_graphs[i];
    //        if(i> 0 && first_seven_graphs[i] == first_seven_graphs[i-1])break;
    addGraph(node->graph->getGraphData());
#ifdef UI_TEST_ON_COMPUTER
    log_graph_info(node);
#endif
    node->graph->setDrawn();
    if (node->graph->getGraphData().operate_type == DELETE) {
      delete node->graph;
      delNode(node);
      //// TODO: 防止丢包
    }
  }
}

void UI::delAll() {
  GraphListNode* node = head_;
  while (node != nullptr) {
    GraphListNode* tmp = node;
    node = node->next;
    delete tmp->graph;
    delete tmp;
  }
  head_ = nullptr;
  graph_num_ = 0;
}

int relativeStaticPriRule(const DrawProperty& state) {
  int addition = 0;
  if (state.drawn) {
    if ((state.static_content))
      return -1;
    //        if (state.content_changed) { addition +=20; }
  } else {
    addition += 20;
  }
  if (state.min_request_refresh_count - state.count_since_last_update < 30)
    // 如果全都挤到一起发送反而也会很卡，这里先分配出去一些
    addition += state.priority + (30 - state.min_request_refresh_count +
                                  state.count_since_last_update) *
                                     2;

  return state.priority + addition + state.count_since_last_update / 10;
}

int defaultPriRule(const DrawProperty& state) {
  int addition = 0;

  if (state.drawn) {
    if ((state.static_content))
      return -1;
    if (state.content_changed)
      addition += 6;
  } else {
    addition += 8;
  }
  if (state.min_request_refresh_count - state.count_since_last_update < 5)
    // 如果全都挤到一起发送反而也会很卡，这里先分配出去一些
    addition += state.priority + (5 - state.min_request_refresh_count +
                                  state.count_since_last_update) *
                                     10;
  if (state.min_request_refresh_count <= state.count_since_last_update)
    addition +=
        (state.count_since_last_update - state.min_request_refresh_count + 1) *
        100000;
  return state.priority + addition + state.count_since_last_update / 20;
}

void UI::init() {
  memset(buf, 0, sizeof(buf));
  memset(char_buf, 0, sizeof(char_buf));
}

void UI::deleteGraph(uint8_t all, uint8_t layer) {
  setHeader(8, 0x0100);

  buf[13] = (all) ? 2 : 1;
  buf[14] = layer;

  CRC16_Append(buf, 17);

#ifndef UI_TEST_ON_COMPUTER
  HAL_UART_Transmit_IT(huart, buf, 17);
#endif
}

void UI::reload() {
  // restart and clear the UI and GraphManager
  ui.deleteGraph(1, 0);
  delAll();
}

void UI::drawGraphs(uint8_t n) {
  static uint16_t cmd_id;

  memset(buf + 13 + 15 * n, 0, 120 - 13 - 15 * n);  // 清空没被图层占满的数据
  if (n >= 3 && n <= 5) {
    n = 5;
    cmd_id = 0x0103;
  } else if (n >= 6) {
    n = 7;
    cmd_id = 0x0104;
  } else
    cmd_id = 0x0100 + n;

  setHeader(15 * n + 15 - 9, cmd_id);
  CRC16_Append(buf, 15 * n + 15);

#ifndef UI_TEST_ON_COMPUTER
  HAL_UART_Transmit_IT(huart, buf, 15 * n + 15);
#endif
}

void UI::setHeader(uint16_t data_len, uint16_t cmd_id) {
  frame_head.SOF = 0xA5;
  frame_head.data_length = data_len;
  frame_head.seq = seq_;

  *(frame_head_t*)buf = frame_head;
  CRC8_Append(buf, 5);

  *(uint16_t*)(buf + 5) = 0x0301;

  data_head.data_cmi_id = cmd_id;

#ifndef UI_TEST_ON_COMPUTER
  data_head.sender_ID = referee->robot_status_.robot_id;
  data_head.receiver_ID = referee->robot_status_.robot_id + 0x0100;
#endif

  *(data_head_t*)(buf + 7) = data_head;
}

void UI::handle() {
#ifndef UI_TEST_ON_COMPUTER
  if (HAL_GetTick() - last_tick_ >= refresh_delay_) {  // 8hz
    last_tick_ = HAL_GetTick();
    // 比赛开始前会自动重新加载一次，防止操作手清空ui或者忘记登录之类的
    if (!first_game_start_flag_ && referee->game_status_.game_progress >= 2) {
      first_game_start_flag_ = true;
      reload();
    }
    for (int i = 0; i < ui_func_num; i++) {
      ui_func[i].decodingFunction();
    }
#else
  for (auto& i : ui_func) {
    i.decodingFunction();
  }
#endif

    graph_n_ = 0;
    graphManagerHandle();
    if (++seq_ >= 100)
      seq_ = 1;
    if (graph_n_ > 0)
      drawGraphs(graph_n_);

#ifndef UI_TEST_ON_COMPUTER
  }
#endif
}

void UI::clearGraphBuffer() {
  memset(buf + 13, 0, 120 - 13);
  graph_n_ = 0;
}

void UI::addGraph(interaction_figure_t graph) {
  *(interaction_figure_t*)(buf + 13 + 15 * graph_n_++) = graph;
}

void UI::drawStr(interaction_figure_t g_data, char content[30]) {
  //    printf(content);
  setHeader(51, 0x0110);
  *(interaction_figure_t*)(buf + 13) = g_data;
  memcpy(buf + 28, content, 30);
  CRC16_Append(buf, 60);
#ifndef UI_TEST_ON_COMPUTER
  HAL_UART_Transmit_IT(huart, buf, 60);
#endif
  graph_n_ = 0;
}
//// UI version 3.5
