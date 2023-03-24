#ifndef SOLUTION_HPP
#define SOLUTION_HPP
#include <bits/stdc++.h>
#include "util.hpp"
#define MAP_SIZE 100
#define WORKBENCH_SIZE 50
#define ROBOT_SIZE 4
#define DEBUG
// #define ESTIMATE
//TODO polish
#define poolSize  WORKBENCH_SIZE * 6    //产品池大小
#define maxNode  WORKBENCH_SIZE * 4 + ROBOT_SIZE * 2 + poolSize * 2 
#define inf  990000
#define INF  1e18
#define vec coordinate
using namespace std;

struct coordinate {
    double x, y;
    coordinate() {};
    coordinate(double xx, double yy) {x = xx; y = yy;}
    void set(double xx, double yy) {x = xx; y = yy;}
};

struct workbench {
    int type;       // 工作台类型 
    coordinate location;
    int rtime;      // 剩余生产时间 Remaining production time
    int rstatus;    // 原材料格状态 Raw—material status
    int pstatus;    // 产品格状态   Product status
    bool reachable; // 可达标志，记录是否有机器人选取该工作台作为购买目标

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
    // 计算当前已拥有原材料数量
    int rawMaterNum() {
        int r = rstatus, cnt = 0;
        while (r) {
            if (r & 1) cnt++;
            r >>= 1;
        }
        return cnt;
    }
};

struct command { // 汇总当前帧机器人的控制指令
    double forward = 0;
    double rotate = 0;
    bool buy = false;
    bool sell = false;
    bool destroy = false;
    inline void clean() {
        buy = sell = destroy = false;
    }
};

struct task { // 机器人的当前目标工作
    coordinate destCo;  // 目的坐标
    int destId;         // 目标工作台下标
    bool buy = false;
    bool sell = false;
    task(coordinate c, int d, bool b, bool s) {
        destCo = c;
        destId = d;
        buy = b;
        sell = s;
    }
    task():destId(-1),destCo(0,0){}
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
    double estFrame = 0; // 估计任务消耗帧数
    
    misson(int s, int e, int p) {
        startIndex = s;
        endIndex = e;
        proType = p;
    }
    misson(){};
    void countValue(coordinate& rtCo, int proType, vec& lsp);
};

struct robot {
    int rtIdx;
    int wb_id;    // 所处工作台ID
    int pd_id;    // 携带产品类型
    double tvc;   // 时间价值系数 Time value coefficient
    double cvc;   // 碰撞价值系数 Collision value coefficient
    double pcvc;  // 前一帧的碰撞价值系数 Collision value coefficient
    double asp;   // Angular speed
    vec lsp;      // Linear speed
    double toward; // 朝向，弧度制
    coordinate location;
    command cmd;  // 当前帧要发布的控制指令
    int wallCollisionNum;  // 跟墙壁碰撞次数
    int roboCollisionNum;  // 跟其他机器人碰撞次数
    int item;          // 当前帧产品类型
    int buyNum[8];     // 物品的购买次数
    int sellNum[8];    // 物品的出售次数
    void setSpeed(coordinate dest); // 负责从当前位置移动到目的地的线速度和角速度指令

    queue<task> taskQueue; // 任务队列
    misson curMission;

    // 碰撞避免持续时间
    int holdTime = 0;
    bool leftOrRight = 0; // left:0, right:1
    vec avoidance;
    // 势能场临时中间点碰撞避免所需变量
    coordinate temDest;
    bool haveTemDest = 0;

    // 当前任务
    task curTask;
    //携带的产品来自的节点
    int nodeId = -1;

    void checkDest();
    void checkTask();
    void checkSpeed();
    void findMission(vector<misson>&, coordinate&, vec&);
    void setTemporaryDest(coordinate&); // 设置临时目的地

    // 用于统计
    void collisionCount();
    void buysellCount();
};

struct mcmf {
    
    const double eps = 1e-6;
    double para1 = -45000; // 时间相关系数
    double para2 = 5;    // 价值相关系数


    int S, T;
    int robotId[ROBOT_SIZE];
    int workbenchProductId[WORKBENCH_SIZE][3]; // 工作台原料格id
    int ProductId2Workbench[maxNode];          // 原料格id转工作台 //TODO:check whether should use map

    int workbenchId[WORKBENCH_SIZE];           // 工作台产品格id
    int cnt;                                   // 节点总数
    int curSize;                               // 空闲产品节点数量
    int pool[poolSize];                        // 空闲节点池

    set<int> produce2sell[10];                 // TODO :whether should move to global

    struct edge {                              // 边
        int rev,to,cap;
        double cost;
        edge(){};
        edge(int to,double cost,int cap,int rev):to(to),cost(cost),cap(cap),rev(rev){}

    };
    vector<edge> G[maxNode];            // 邻接表
    void addEdge(int from,int to,double cost,int cap){                // 加边
        G[from].push_back(edge(to,cost,cap,G[to].size()));
        G[to].push_back(edge(from,-cost,0,G[from].size()-1));
    }
    void setEdgeCap(int from,int index,int cap){
        edge &tmp = G[from][index];
        tmp.cap = cap;
        G[tmp.to][tmp.rev].cap = 0;
    }
    void setEdgeCost(int from,int index,double cost){
        edge &tmp = G[from][index];
        tmp.cost = cost;
        G[tmp.to][tmp.rev].cost = -cost;
    }
    

    double shortDis[maxNode];                // 最短路
    int cut[maxNode];                        // 顶点访问次数
    int vis[maxNode];                        // 顶点是否在队列中
    int pre[maxNode];                        // 最短路上的前驱节点
    int pe[maxNode];                         // 最短路上的前驱边
    int leftTime[maxNode];                   // 剩于生产时间
    int stateBuf[ROBOT_SIZE][ROBOT_SIZE * 15][2];// 用于权值回退
    int bufCur = 0;                          // 已使用的Buf数
    int flow = 0;

    
    int getNode(){return cnt++;}
    void init();                        // 在init()后运行

    mcmf():cnt(0),curSize(0){}
    int spfa();                         
    int solve();                        // 基于spfa计算最小费用流

    void releaseNode(int id);           // 将某个节点移入空闲节点池
    void lockNode(int rtIdx,int wbIdx); // 工作台产品被机器人获取
    void allocateNode(int wbIdx);       // 为工作台产品分配节点

    void adjustEdge(int rtIdx);         // 每帧开始时为机器人调整边权
    void adjustTask(int rtIdx);         // 每帧开始时根据费用流为机器人调整任务
    void resetCap();                    // 每帧结束时回退费用流
    void showNodeEdge(int id,int condition = 1); //打印输出某一个节点的所有边
    void showFlow(int condition = 1,int detailed = 0);   // 可视化当前网络流                 
    void solution();
    void checkDest(int rtIdx);
    int checkVaild(double a);           // 检测浮点数类型

    // 以下计算的为代价,越小优先级越大
    // 计算价值函数,计算机器人购买产品后，将其运往出售的开销
    // 参数依次为携带产品类型、起点、终点
    double countValue(int proType,int startIndex,int endIndex);

    // 计算价值函数,计算机器人购买产品后，将其销毁并前往其它工作台的开销
    // 参数依次为携带产品类型(0表示没有)、机器人下标、终点
    double countBuyValue(int proType,int rtIdx,int endIndex);

    // 计算价值函数,计算机器人从当前位置前往对应工作台出售的开销
    // 参数依次为携带产品类型、机器人下标、终点
    double countSellValue(int proType,int rtIdx,int endIndex);
};

extern int frameID;                   // 当前帧
extern int K;                         // 工作台数
extern int N;                         // 机器人数
extern int curMoney;                  // 当前金钱
extern robot rt[ROBOT_SIZE];          // 机器人
extern workbench wb[WORKBENCH_SIZE];  // 工作台
extern char plat[MAP_SIZE][MAP_SIZE]; // 输入地图
extern int totalSellNum[8];    // 物品的出售次数
extern const double PI;               // 圆周率
extern mcmf curFlow;                  // 网络流实例
extern ofstream fout;                 // 与日志文件关联的输出流

    extern double para1;
    extern double para2;
    extern double para4;

extern map<int, vector<int>> type2BuyIndex; // 根据产品类型寻找收购方下标

extern pair<pair<int,int>,int> profitAndTime[8];

extern double dis(coordinate& c1, coordinate& c2);
extern double crossProduct(vec& a, vec& b);
extern double dotProduct(vec& a, vec& b);
extern double modulusOfVector(vec& a);
extern double cntAngle(vec& a, vec& b);

void collitionAvoidance();
void ori_collitionAvoidance();
void ori_solution();
void solution();
#endif
