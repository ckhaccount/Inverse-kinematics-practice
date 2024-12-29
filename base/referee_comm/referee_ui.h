//
// Created by yixin on 2023/2/5.
//

#ifndef REFEREE_UI_H
#define REFEREE_UI_H

#include "referee_comm.h"
#include "referee_protocol.h"

#ifndef __packed
#define __packed __attribute__((__packed__))
#endif

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

enum GraphOperateType {
  NO_OPERATE = 0,
  ADD = 1,
  MODIFY = 2,
  DELETE = 3,
};

enum GraphType { LINE, RECT, CIRCLE, ELLIPSE, ARC, FLOAT, INT, STRING };

enum Color {
  TEAM_COLOR,
  YELLOW,
  GREEN,
  ORANGE,
  PURPLE,
  PINK,
  CYAN,
  BLACK,
  WHITE,
};

// 包头
typedef struct {
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
} __attribute__((__packed__)) frame_head_t;

// 数据段头
typedef struct {
  uint16_t data_cmi_id;
  uint16_t sender_ID;
  uint16_t receiver_ID;
} __attribute__((__packed__)) data_head_t;

//// ----------------UI------------------
//// UI version 3.5
struct UIPoint {
  int16_t x;
  int16_t y;
  UIPoint operator+(const UIPoint& point) const {
    return UIPoint{(int16_t)(x + point.x), (int16_t)(y + point.y)};
  }
};

struct UIVec {
  int16_t x;
  int16_t y;
  UIVec operator+(const UIVec& vec) const {
    return UIVec{(int16_t)(x + vec.x), (int16_t)(y + vec.y)};
  }
};

struct UIPolarVec {
  int16_t r;
  float theta;
};

struct UIVecWithStartPoint {
  UIPoint start_point;
  UIPolarVec vec;
};

UIPoint polar2Point(UIPolarVec vec);

struct DrawProperty {
  int priority = 0;
  uint16_t count_since_last_update =
      0;  // 一次 count 的实际时间为 refresh_delay_ ms
  uint8_t min_request_refresh_count = 100;  // 默认100次handle一定要刷新一次
  ////TODO 刷新
  bool static_content =
      false;  // 如果为真，则不会主动刷新，除非操作手按指定按键触发刷新ui
  bool drawn = false;
  bool content_changed = true;
};

/**
 * @brief 这个类的对象包含了优先级，逻辑，相关函数变量，以及图形数据本身。
 *        此类对象一旦被创建且作为参数传入 manager
 *类，就会被认为是一个画图请求。删除此图形也属于画图请求，但是不需要创建对象，而通过
 *        名字索引直接在 manager 类中调用删除函数
 **/
class Graph {
 public:
  interaction_figure_t getGraphData() const { return graph_data_; }

  Graph& setGraphData(const interaction_figure_t& graphData) {
    graph_data_ = graphData;
    return *this;
  }

  Graph* setPos(
      u16 x, u16 y,
      bool if_start_point = true) {  // if start point默认修改start
                                     // point, 设置为false修改end point
    if (if_start_point) {
      graph_data_.start_x = x;
      graph_data_.start_y = y;
    } else {
      // graph_data_.end_x = x;
      // graph_data_.end_y = y;
      graph_data_.details_d = x;
      graph_data_.details_e = y;
    }
    return this;
  }
  Graph* setPos(UIPoint point, bool if_start_point = true) {
    return setPos(point.x, point.y, if_start_point);
  }
  Graph* setPos(UIVec vec, bool if_start_point = true) {
    return setPos(vec.x, vec.y, if_start_point);
  }
  Graph* setCompletePos(UIVecWithStartPoint vec) {
    setPos(vec.start_point.x, vec.start_point.y, true);
    return setPos(polar2Point(vec.vec) + vec.start_point, false);
  }
  Graph* setColor(Color color) {
    graph_data_.color = color;
    return this;
  }
  Graph* setRelativePos(const char target[3], u16 relative_x, u16 relative_y,
                        bool if_start_point = true);
  Graph* setRelativePos(const char target[3], uint16_t (*func1)(uint16_t),
                        uint16_t (*func2)(uint16_t),
                        bool if_start_point = true);
  Graph* setRelativePos(const char target[3], UIPoint point,
                        bool if_start_point = true) {
    return setRelativePos(target, point.x, point.y, if_start_point);
  };
  Graph* setRelativePos(const char target[3], UIVec vec,
                        bool if_start_point = true) {
    return setRelativePos(target, vec.x, vec.y, if_start_point);
  };
  Graph* setFollowColor(const char target[3]);
  Graph* setLine(int16_t x, int16_t y) {
    return setPos(this->graph_data_.start_x + x, this->graph_data_.start_y + y,
                  false);
  };  // 根据直线的第一个点坐标确定另一个点坐标
  Graph* setLine(UIPoint vec) {
    return setPos(vec + UIPoint{(int16_t)this->graph_data_.start_x,
                                (int16_t)this->graph_data_.start_y},
                  false);
  };
  Graph* setLine(UIPolarVec vec) {
    return setPos(
        polar2Point(vec) + UIPoint{(int16_t)this->graph_data_.start_x,
                                   (int16_t)this->graph_data_.start_y},
        false);
  };

  Graph(const Graph&) = default;
  Graph() = default;
  virtual ~Graph() = default;

  void calcThisPriority() {
    if (priority_calc_func_ != nullptr)
      this_priority_ = priority_calc_func_(draw_property_);
  }
  int getThisPriority() const { return this_priority_; }
  Graph* setPriority(int priority) {
    Graph::draw_property_.priority = priority;
    return this;
  }
  void setOperateType(GraphOperateType ot) {
    // 设置操作类型（ADD, DELETE, MODIFY）
    graph_data_.operate_type = ot;
    // 如果画过了而且操作还是ADD，那么content_changed是false
    if (draw_property_.drawn && ot == ADD)
      draw_property_.content_changed = false;
    // 如果画过了而且操作是MODIFY，那么content_changed是true
    if (draw_property_.drawn && ot == MODIFY)
      draw_property_.content_changed = true;
  }

  DrawProperty getDrawProperty() const { return draw_property_; }
  unsigned char* getGraphicName() { return graph_data_.figure_name; }
  virtual char* getCharContent() { return nullptr; };

  void addCountSinceLastUpdate() { draw_property_.count_since_last_update++; }
  void setDrawn() {
    draw_property_.count_since_last_update = 0;
    draw_property_.drawn = true;
    if (graph_data_.operate_type == ADD)
      draw_property_.content_changed = false;
    if (graph_data_.operate_type == MODIFY) {
      // 如果是修改，然后这次画上去了，就把operate
      // type重新设置回add，content_changed设置为false
      graph_data_.operate_type = ADD;
      draw_property_.content_changed = false;
    }
  }
  Graph* setPriorityCalcFunc(int (*priorityCalcFunc)(const DrawProperty&)) {
    priority_calc_func_ = priorityCalcFunc;
    return this;
  }
  Graph* setStaticContent() {
    draw_property_.static_content = true;
    return this;
  }
  Graph* setMinRefreshCnt(int cnt) {
    draw_property_.min_request_refresh_count = cnt;
    return this;
  }

 protected:
  // interaction_figure_t只包含能够定义图形本身的部分数据，比如类型，大小，位置，颜色，但是不包含优先级，上次更新时间
  interaction_figure_t graph_data_{};
  DrawProperty draw_property_{};
  int this_priority_ =
      0;  // 设置的 priority 不是最终的 priority ，实际比较需要此动态值
  int (*priority_calc_func_)(const DrawProperty&) = nullptr;
};

class Circle : public Graph {
 public:
  Circle(const char figure_name[3], u8 layer, Color color, u8 width,
         u16 center_x, u16 center_y, u16 radius);
};

class Rect : public Graph {
 public:
  Rect(const char figure_name[3], u8 layer, Color color, u8 width, u16 x1,
       u16 y1, u16 x2, u16 y2);
};

class Line : public Graph {
 public:
  Line(const char figure_name[3], u8 layer, Color color, u8 width, u16 x1,
       u16 y1, u16 x2, u16 y2);
};

class Ellipse : public Graph {
 public:
  Ellipse(const char figure_name[3], u8 layer, Color color, u8 width,
          u16 center_x, u16 center_y, u16 ax, u16 ay);
};

class Arc : public Graph {
 public:
  Arc(const char figure_name[3], u8 layer, Color color, u8 width, u16 center_x,
      u16 center_y, u16 ax, u16 ay, u16 angle_st, u16 angle_ed);
};

class IntNum : public Graph {
 public:
  IntNum(const char figure_name[3], u8 layer, Color color, u8 width, u8 size,
         u16 x, u16 y, int32_t num);
};

class FloatNum : public Graph {
 public:
  FloatNum(const char figure_name[3], u8 layer, Color color, u8 width, u8 size,
           u8 effective_digit, u16 x, u16 y, float num);
};

class Str : public Graph {
 public:
  Str(const char figure_name[3], u8 layer, Color color, u8 size, u8 len,
      u8 width, u16 x, u16 y, const char* __restrict format, ...);

  ~Str() override { delete[] content_; }

  char* getCharContent() override { return content_; }

  char* content_ = nullptr;  // 只有字符串需要用到
};

int relativeStaticPriRule(const DrawProperty& state);

int defaultPriRule(const DrawProperty& state);

struct GraphListNode {
  Graph* graph;
  struct GraphListNode* next;
  struct GraphListNode* prev;
};

typedef struct {
  void (*decodingFunction)();
} UIFunc_t;

extern UIFunc_t ui_func[];

//// UI version 3.5
class UI {
 public:
#ifdef UI_TEST_ON_COMPUTER
  static UI ui;
#else
  explicit UI(UART_HandleTypeDef* huart_, RefereeComm* referee_,
              UIFunc_t* ui_func_, uint16_t ui_func_num_)
      : huart(huart_),
        referee(referee_),
        ui_func(ui_func_),
        ui_func_num(ui_func_num_) {}

  UART_HandleTypeDef* huart;
  RefereeComm* referee;
  UIFunc_t* ui_func;
  uint16_t ui_func_num;
#endif
  ~UI() {
    GraphListNode* node = head_;
    while (node != nullptr) {
      GraphListNode* tmp = node;
      node = node->next;
      delete tmp->graph;
      delete tmp;
    }
  }
  void handle();
  void init();

  // 在客户端上删除所有图形并且重置graph manager
  void reload();

  void deleteGraph(
      uint8_t all,
      uint8_t layer);  // all: 0:delete one layer_; 1:delete all layers
 private:
  // ui 画图用到的变量，用户不需要去管
  char char_buf[30]{};
  uint8_t buf[120]{};
  frame_head_t frame_head{};
  data_head_t data_head{};

  uint8_t layer_ = 0;
  uint8_t graph_n_ = 0;
  uint8_t seq_ = 1;  // 1~100

  uint32_t last_tick_ = 0;
  uint8_t refresh_delay_ = 130;

  bool first_game_start_flag_ = false;

  void drawGraphs(uint8_t graph_n);
  void drawStr(interaction_figure_t g_data, char content[30]);
  void addGraph(interaction_figure_t graph);
  void clearGraphBuffer();
  void setHeader(uint16_t data_len, uint16_t cmd_id);

  // graph manager 相关
 public:
  UI& add(Graph* graph);
  UI& del(const char figure_name[3]);
  void delAll();
  static interaction_figure_t getGraphData(const char name[3]);

 private:
  void graphManagerHandle();

  GraphListNode* head_ = nullptr;
  uint16_t graph_num_ = 0;
  uint8_t draw_nums = 7;  // 这次循环画的图形数量

  GraphListNode* fineNode(const char name[3]) {
    GraphListNode* node = head_;
    while (node != nullptr) {
      auto* tmp = node->graph->getGraphicName();
      if (tmp[0] == name[0] && tmp[1] == name[1] && tmp[2] == name[2])
        return node;
      node = node->next;
    }
    return nullptr;
  }
  GraphListNode* getCircularNextNode(GraphListNode* node) {
    if (node->next != nullptr)
      return node->next;
    return head_;
  }
  void addNode(Graph* graph) {
    auto* node = new GraphListNode;
    node->graph = graph;
    node->next = head_;
    node->prev = nullptr;
    if (head_ != nullptr)
      head_->prev = node;
    head_ = node;
    graph_num_++;
  }
  void delNode(GraphListNode* node) {
    if (node->prev != nullptr)
      node->prev->next = node->next;
    if (node->next != nullptr)
      node->next->prev = node->prev;
    if (node == head_)
      head_ = node->next;
    delete node;
    graph_num_--;
  }
};

#endif  // UI_RECONSTRUCT_UI_H
