#include <bits/stdc++.h>
#define MAP_SIZE 100
#define WORKBENCH_SIZE 50
#define ROBOT_SIZE 50
#define vec coordinate
using namespace std;

struct coordinate {
    double x, y;
    void set(double xx, double yy) {x = xx; y = yy;}
};

struct workbench {
    int type;
    coordinate location;
    int rtime;   // 剩余生产时间 Remaining production time
    int rstatus; // 原材料格状态 Raw—material status
    int pstatus; // 产品格状态   Product status
};

struct command {
    double forward;
    double rotate;
    bool buy = false;
    bool sell = false;
    bool destroy = false;
};

struct robot {
    int wb_id;    // 所处工作台ID
    int pd_id;    // 携带产品类型
    double tvc;   // 时间价值系数 Time value coefficient
    double cvc;   // 碰撞价值系数 Collision value coefficient
    double asp;   // Angular speed
    vec lsp;      // Linear speed
    double toward; // 朝向，弧度制
    coordinate location;
    command cmd;  // 当前帧要发布的控制指令
};

int K;                         // 工作台数
robot rt[ROBOT_SIZE];          // 机器人
workbench wb[WORKBENCH_SIZE];  // 工作台
char plat[MAP_SIZE][MAP_SIZE]; // 输入地图