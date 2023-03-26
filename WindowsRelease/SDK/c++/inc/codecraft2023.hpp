#ifndef SOLUTION_HPP
#define SOLUTION_HPP
#include <bits/stdc++.h>
#include "define.hpp"
#include "io.hpp"
#include "tool.hpp"
#include "math_tool.hpp"
#include "mission.hpp"
#include "robot.hpp"
#include "workbench.hpp"
#include "mcmf.hpp"
#include "util.hpp"
using namespace std;

extern int K;                               // 工作台数
extern int N;                               // 机器人数
extern int frameID;                         // 当前帧
extern int curMoney;                        // 当前金钱
extern char plat[MAP_SIZE][MAP_SIZE];       // 输入地图
extern int totalSellNum[WORKBENCH_SIZE];    // 物品的出售次数
extern const double PI;                     // 圆周率

extern map<int, vector<int>> type2BuyIndex;                     // 根据产品类型寻找收购方下标
extern pair<pair<int,int>,int> profitAndTime[WORKBENCH_SIZE];   // 记录收购价、购入价以及生产用时

extern double para1;
extern double para2;
extern double para4;

void collitionAvoidance();
void ori_collitionAvoidance();
void ori_solution();
#endif
