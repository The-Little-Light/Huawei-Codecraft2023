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

    // 判断原材料proType型号格是否为空，空返回false
    bool checkHaveProType(int proType) {
        int s = 1;
        while(proType--) s <<= 1;
        return s & rstatus;
    }
    void setProType(int proType) {
        int s = 1;
        while(proType--) s <<= 1;
        rstatus = s | rstatus;
    }
};

struct command { // 汇总当前帧机器人的控制指令
    double forward;
    double rotate;
    bool buy = false;
    bool sell = false;
    bool destroy = false;
};

struct task { // 机器人的当前目标工作
    coordinate destCo;  // 目的坐标
    int destId;         // 目标工作台下标
    bool buy = false;
    bool sell = false;
    task(coordinate c, int d) {
        destCo = c;
        destId = d;
    }
};


/* 
定义任务 m(A, x, B) 表示把物品x从A工作台购入并出售给B工作台
任务价值函数 v(m) 表示完成任务m可以获得的潜在收益
    1：v 跟x在AB的差价正相关
    2：跟B的剩余原材料格、AB路径负相关
*/
struct misson {
    int startIndex; // 起点工作台下标
    int endIndex;   // 终点工作台下标
    int proType;    // 产品型号
    double v = 0;   // 价值函数

    double dis(int s, int e) {
        coordinate& c1 = wb[s].location;
        coordinate& c2 = wb[e].location;
        double ans = (c1.x - c2.x)*(c1.x - c2.x) + (c1.y - c2.y)*(c1.y - c2.y);
        return sqrt(ans);
    }
    misson(int s, int e, int p) {
        startIndex = s;
        endIndex = e;
        proType = p;
        v = profitAndTime[p].first * para1 + para2 / dis(s,e);
    }
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
    void setSpeed(coordinate dest); // 负责从当前位置移动到目的地的线速度和角速度指令

    queue<task> taskQueue; // 任务队列
    misson curMisson;
    void checkDest();
    void checkTask();
};

int K;                         // 工作台数
robot rt[ROBOT_SIZE];          // 机器人
workbench wb[WORKBENCH_SIZE];  // 工作台
char plat[MAP_SIZE][MAP_SIZE]; // 输入地图

map<int, vector<int>> type2BuyIndex; // 根据产品类型寻找收购方下标

pair<int,int> profitAndTime[8];

void solution();