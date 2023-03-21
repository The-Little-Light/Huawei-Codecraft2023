/*** 
 * @Author: Xzx
 * @Date: 2023-03-15 00:10:42
 * @LastEditTime: 2023-03-21 21:16:11
 * @LastEditors: Xzh
 * @Description: 
 */
#include "solution.hpp"
using namespace std;

int frameID;                   // 当前帧
int K;                         // 工作台数
int N;                         // 机器人数
int curMoney;                  // 当前金钱
robot rt[ROBOT_SIZE];          // 机器人
workbench wb[WORKBENCH_SIZE];  // 工作台
char plat[MAP_SIZE][MAP_SIZE]; // 输入地图
int collisionNum[ROBOT_SIZE];   // 碰撞次数
int buyNum[8][ROBOT_SIZE];      // 物品的购买次数
int sellNum[8][ROBOT_SIZE];     // 物品的出售次数
ofstream fout;                 // 与日志文件关联的输出流
mcmf curFlow;                  // 网络流实例

map<int, vector<int>> type2BuyIndex; // 根据产品类型寻找收购方下标

pair<int,int> profitAndTime[8];


void init() {
    // 初始化工作台信息
    K = 0;
    for(int i = 0; i < MAP_SIZE; ++i){
        for(int j = 0; j < MAP_SIZE; ++j){
            if(isdigit(plat[i][j])) wb[K++].type = plat[i][j] - '0';
            if(plat[i][j] == 'A') N++;
        }
    }
    profitAndTime[0] = make_pair(0, 10000);
    // 检测平台log
    fout.open("log.txt", ios_base::app);
    for (int i = 0; i < ROBOT_SIZE; ++i) rt[i].rtIdx = i+1;
    profitAndTime[1] = make_pair(6000-3000, 50);
    profitAndTime[2] = make_pair(7600-4400, 50);
    profitAndTime[3] = make_pair(9200-5800, 50);
    profitAndTime[4] = make_pair(22500-15400, 500);
    profitAndTime[5] = make_pair(25000-17200, 500);
    profitAndTime[6] = make_pair(27500-19200, 500);
    profitAndTime[7] = make_pair(105000-76000, 1000);
    // 初始化 type2BuyIndex，为收购方建立索引
    for (int wbIdx = 0; wbIdx < K; ++wbIdx) {
        switch (wb[wbIdx].type) {
        case 4:
            type2BuyIndex[1].push_back(wbIdx);
            type2BuyIndex[2].push_back(wbIdx);
            break;
        case 5:
            type2BuyIndex[1].push_back(wbIdx);
            type2BuyIndex[3].push_back(wbIdx);
            break;
        case 6:
            type2BuyIndex[2].push_back(wbIdx);
            type2BuyIndex[3].push_back(wbIdx);
            break;
        case 7:
            type2BuyIndex[4].push_back(wbIdx);
            type2BuyIndex[5].push_back(wbIdx);
            type2BuyIndex[6].push_back(wbIdx);
            break;
        case 8:
            type2BuyIndex[7].push_back(wbIdx);
            break;
        case 9:
            for (int i = 1; i <= 7; ++i) {
                type2BuyIndex[i].push_back(wbIdx);
            }
            break;
        default:
            break;
        }
    }
    // 预初始化网络流
    curFlow.init();
}


void readPlat() {
    char line[1024];
    for (int i = 0; i < 100; ++i) {
        fgets(plat[i], sizeof(line), stdin);
    }
    fgets(line, sizeof line, stdin); // receive OK
}

void readInfo() {
    char line[3];
    double x, y;
    scanf("%d %d",&curMoney, &K);
    for (int i = 0; i < K; ++i) {
        scanf("%d %lf %lf %d %d %d", 
            &wb[i].type,
            &x, &y, // location
            &wb[i].rtime,
            &wb[i].rstatus,
            &wb[i].pstatus
        ); wb[i].location.set(x, y);
    }
    for (int i = 0; i < ROBOT_SIZE; ++i) {
        rt[i].pcvc = rt[i].cvc;
        scanf("%d %d %lf %lf %lf %lf %lf",
            &rt[i].wb_id,
            &rt[i].pd_id,
            &rt[i].tvc,
            &rt[i].cvc,
            &rt[i].asp,
            &x, &y // lsp
        ); rt[i].lsp.set(x, y);
        scanf("%lf %lf %lf",
            &rt[i].toward,
            &x, &y // location
        ); rt[i].location.set(x, y);
        if ((rt[i].pcvc - rt[i].cvc >= 0.001) && (rt[i].cvc > 0.79)) {
            // fout << frameID << "(" << "robot" << i+1 << ")" << ": " << rt[i].pcvc << " -> " << rt[i].cvc << endl;
            ++collisionNum[i];
        }
    }
    getchar();
    fgets(line, sizeof line, stdin); // receive OK

}

void debug(){
    for(int robotId = 0; robotId < 4; robotId++){
        if (rt[robotId].cmd.sell)  fprintf(stderr,"sell %d\n", robotId);
        if (rt[robotId].cmd.buy)  fprintf(stderr,"buy %d\n", robotId);
        if (rt[robotId].cmd.destroy)  fprintf(stderr,"destroy %d\n", robotId);
        fprintf(stderr,"forward %d %f\n", robotId, rt[robotId].cmd.forward);
        fprintf(stderr,"rotate %d %f\n", robotId, rt[robotId].cmd.rotate);
        fprintf(stderr,"curtask.buy %d\n", (int) rt[robotId].curTask.buy);
        fprintf(stderr,"curtask.sell %d\n",  (int)rt[robotId].curTask.sell);
        fprintf(stderr,"destId %d\n", rt[robotId].curTask.destId);
        fprintf(stderr,"nodeId %d\n\n\n", rt[robotId].nodeId);

    }
    cerr<<"cnt : "<<curFlow.cnt<<endl;
    cerr<<"curpool : "<<curFlow.curSize<<endl;
    cerr<<"curframeID : "<<frameID<<endl;
    cerr<<"flow : "<<curFlow.flow<<endl;
    cerr<<"--------------------------\n";

}

void printRobotCommand(int robotId) {
    if (rt[robotId].cmd.sell)  printf("sell %d\n", robotId);
    if (rt[robotId].cmd.buy)  printf("buy %d\n", robotId);
    if (rt[robotId].cmd.destroy)  printf("destroy %d\n", robotId);
    printf("forward %d %f\n", robotId, rt[robotId].cmd.forward);
    printf("rotate %d %f\n", robotId, rt[robotId].cmd.rotate);
}

int main() {
    // bool initMark = true;
    readPlat();
    init();
    puts("OK");
    fflush(stdout);
    while (scanf("%d", &frameID) != EOF) {
        readInfo();
        
        // if (initMark) {
        //     init();
        //     initMark = false;
        // }
        printf("%d\n", frameID);
        /**** CORE ****/
        // ori_solution();
        curFlow.solution();
        /**************/
        for(int robotId = 0; robotId < ROBOT_SIZE; robotId++){
            printRobotCommand(robotId);
        }
        // if(frameID>48) debug();
        printf("OK\n");
        fflush(stdout);
    }
    
    fout << "******************************LOG INFORMATION START******************************";
    fout << endl << setw(15) << "ROBOT:";
    for (int i = 1; i <= ROBOT_SIZE; ++i)    fout << setw(8) << i;
    fout << endl << setw(15) << "COLLOSION:";
    for (int i = 0; i < ROBOT_SIZE; ++i)    fout << setw(8) << collisionNum[i];
    for (int i = 1; i <= 7; ++i) {
        fout << endl << setw(10) << i << "_BUY:";
        for (int j = 0; j < ROBOT_SIZE; ++j)    fout << setw(8) << buyNum[i][j];
        fout << endl << setw(9) << i << "_SELL:";
        for (int j = 0; j < ROBOT_SIZE; ++j)    fout << setw(8) << sellNum[i][j];
    }
    fout << endl << "******************************LOG INFORMATION END*******************************" << endl << endl << endl;
    fout.close();

    return 0;
}