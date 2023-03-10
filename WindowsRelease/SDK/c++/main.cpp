#include "para.cpp"
using namespace std;

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

int main() {
    readPlat();
    puts("OK");
    fflush(stdout);
    int frameID;
    while (scanf("%d", &frameID) != EOF) {
        readInfo();
        printf("%d\n", frameID);
        int lineSpeed = 3;
        double angleSpeed = 1.5;
        for(int robotId = 0; robotId < 4; robotId++){
            printf("forward %d %d\n", robotId, lineSpeed);
            printf("rotate %d %f\n", robotId, angleSpeed);
        }
        printf("OK\n");
        fflush(stdout);
    }
    return 0;
}