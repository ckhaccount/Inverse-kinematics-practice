/**
 ******************************************************************************
 * @file    client_ui.cpp/h
 * @brief   RM client ui design. RM比赛客户端ui设计
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/client_ui.h"

void draw_example();
void test_draw();
void DrawAimLine();

extern UI ui;

UIFunc_t ui_func[]{
    DrawAimLine,
    draw_example,
};

void draw_example() {
  static int cnt =0;
  if (cnt++ < 10)
    return;

  ui.add(
      (new Circle("c01", 1, TEAM_COLOR, 2, 1000, cnt++, 20))
          ->setPriority(1)
          ->setPriorityCalcFunc(defaultPriRule));
  ui
      .add((new Circle("c02", 1, BLACK, 2, 1100, 900, 20))
               ->setPriority(10)
               ->setPriorityCalcFunc(defaultPriRule))
      .add((new Circle("c03", 1, PURPLE, 2, 1200, 900, 20))
               ->setPriority(20)
               ->setRelativePos(
                   "c02", [](u16 rev_x) { return (uint16_t)(600 * 2 - rev_x); },
                   [](u16 rev_y) { return (uint16_t)(1080 - rev_y); })
               ->setFollowColor("c01")
               ->setPriorityCalcFunc(defaultPriRule))
      .add((new Circle("c04", 1, GREEN, 2, 1300, 900, 20))
               ->setPriority(8)
               ->setPriorityCalcFunc(defaultPriRule)
               ->setMinRefreshCnt(100))
      .add((new Line("l00", 1, BLACK, 2, 0, 0, 0, 0))
               ->setPriority(8)
               //                         ->setPriorityCalcFunc(DefaultPriRule)
               ->setCompletePos({{5, 10}, {10, 3.14159f / 4}})
               ->setPos(UIPoint{123, 456})
               ->setPos(UIPoint{123, 456}, false)
               ->setLine(10, 10));
  if (cnt > 5 && cnt < 100) {
    ui.del("c01");
  }
  for (int i = 0; i < 50; ++i) {
    char name[3];
    name[0] = 'c';
    name[1] = 'i';
    name[2] = '!' + i;
    ui.add(
        (new Circle(name, i / 10, TEAM_COLOR, 2, 900 + i * 10,
                    800 - i * 2 + cnt, 10))
            ->setPriority(8)
            ->setPriorityCalcFunc(defaultPriRule));
  }
}

void test_draw() {
  ui
      //            .add(Rect("r01", 2, BLACK, 2, 1500, 900, 1700, 800)
      //                         .setPriority(0)
      //                         .setPriorityCalcFunc(DefaultPriRule)
      //            )
      //            .add(IntNum("i10", 3, TEAM_COLOR, 2, 10, 600, 900,
      //            UI::ui.my_cnt++)
      //                         .setPriority(20)
      //                         .setPriorityCalcFunc(DefaultPriRule)
      //            )
      .add((new Str("s00", 2, PURPLE, 20, 4, 4, 900, 500, "Cap:"))
               ->setPriority(1)
               ->setPriorityCalcFunc(relativeStaticPriRule)
               ->setStaticContent());
}

void DrawAimLine() {
  // la0  line aim 0 以此类推
  ui
      .add((new Line("la0", 1, PURPLE, 3, 960, 240, 960, 405))
               ->setPriority(1)
               ->setPriorityCalcFunc(relativeStaticPriRule)
               ->setStaticContent())
      .add((new Line("la4", 1, ORANGE, 1, 960, 405 + 70, 960, 405 + 70))
               ->setLine(25, 0)
               ->setPriority(1)
               ->setPriorityCalcFunc(relativeStaticPriRule)
               ->setStaticContent())
      .add((new Line("la8", 1, ORANGE, 1, 960, 370, 1060, 370))
               ->setLine(50, 0)
               ->setPriority(1)
               ->setPriorityCalcFunc(relativeStaticPriRule)
               ->setStaticContent())
      .add((new Line("la6", 2, ORANGE, 1, 960, 405, 0, 0))
               ->setLine(50, 0)
               ->setPriority(1)
               ->setPriorityCalcFunc(relativeStaticPriRule)
               ->setStaticContent())
      .add((new Line("lax", 2, ORANGE, 1, 960, 405, 0, 0))
               ->setRelativePos("la8", 0, -50)
               ->setLine(50, 0)
               ->setPriority(1)
               ->setPriorityCalcFunc(relativeStaticPriRule)
               ->setStaticContent())
      .add((new IntNum("ia6", 1, PURPLE, 2, 10, 1070, 430, 6))
               ->setRelativePos("la6", 60, 0)
               ->setPriority(1)
               ->setPriorityCalcFunc(relativeStaticPriRule)
               ->setStaticContent())
      .add((new IntNum("ia8", 1, PURPLE, 2, 10, 1070, 392, 8))
               ->setRelativePos("la8", 60, 0)
               ->setPriority(1)
               ->setPriorityCalcFunc(relativeStaticPriRule)
               ->setStaticContent())
      .add((new IntNum("iax", 1, PURPLE, 2, 10, 1070, 370, 10))
               ->setRelativePos("lax", 60, 0)
               ->setPriority(1)
               ->setPriorityCalcFunc(relativeStaticPriRule)
               ->setStaticContent())
      .add((new Line("laA", 2, ORANGE, 1, 960, 405, 0, 0))
               ->setRelativePos("lax", 0, -20)
               ->setLine(25, 0)
               ->setPriority(1)
               ->setPriorityCalcFunc(relativeStaticPriRule)
               ->setStaticContent())
      .add((new Line("laB", 2, ORANGE, 1, 960, 405, 0, 0))
               ->setRelativePos("laA", 0, -20)
               ->setLine(25, 0)
               ->setPriority(1)
               ->setPriorityCalcFunc(relativeStaticPriRule)
               ->setStaticContent())
      .add((new Line("laC", 2, ORANGE, 1, 960, 405, 0, 0))
               ->setRelativePos("laB", 0, -20)
               ->setLine(25, 0)
               ->setPriority(1)
               ->setPriorityCalcFunc(relativeStaticPriRule)
               ->setStaticContent())
      .add((new Line("laD", 2, ORANGE, 1, 960, 405, 0, 0))
               ->setRelativePos("laC", 0, -20)
               ->setLine(25, 0)
               ->setPriority(1)
               ->setPriorityCalcFunc(relativeStaticPriRule)
               ->setStaticContent());
}
