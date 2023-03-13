#include "solution.hpp"
using namespace std;

int K;                         // 工作台数
robot rt[ROBOT_SIZE];          // 机器人
workbench wb[WORKBENCH_SIZE];  // 工作台
char plat[MAP_SIZE][MAP_SIZE]; // 输入地图

map<int, vector<int>> type2BuyIndex; // 根据产品类型寻找收购方下标

pair<int,int> profitAndTime[8];

void init() {
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
    int curMoney;
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
    for (int i = 0; i < 4; ++i) {
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
    }
    getchar();
    fgets(line, sizeof line, stdin); // receive OK
}

void printRobotCommand(int robotId) {
    printf("forward %d %f\n", robotId, rt[robotId].cmd.forward);
    printf("rotate %d %f\n", robotId, rt[robotId].cmd.rotate);
    cerr << "rotate " << robotId << " " << rt[robotId].cmd.rotate << endl;
    cerr << "forward " << robotId << " " << rt[robotId].cmd.forward << endl;
    if (rt[robotId].cmd.sell)  printf("sell %d\n", robotId);
    if (rt[robotId].cmd.buy)  printf("buy %d\n", robotId);
    if (rt[robotId].cmd.destroy)  printf("destroy %d\n", robotId);
}

int main() {
    bool initMark = true;
    readPlat();
    puts("OK");
    fflush(stdout);
    int frameID;
    while (scanf("%d", &frameID) != EOF) {
        readInfo();
        if (initMark) {
            init();
            initMark = false;
        }
        printf("%d\n", frameID);
        /**** CORE ****/
        solution();
        /**************/
        for(int robotId = 0; robotId < 4; robotId++){
            printRobotCommand(robotId);
        }
        printf("OK\n");
        fflush(stdout);
    }
    return 0;
}