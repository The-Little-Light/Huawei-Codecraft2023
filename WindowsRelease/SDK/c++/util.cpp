#include "util.hpp"

// 输出地图的日志信息
void printLog() {
    fout << "******************************LOG INFORMATION START******************************";
    fout << endl << setw(15) << "ROBOT:";
    for (int i = 1; i <= ROBOT_SIZE; ++i)    fout << setw(8) << i;
    fout << setw(8) << "TOTAL";

    int sum = 0;
    fout << endl << setw(15) << "COLLOSION:";
    for (int i = 0; i < ROBOT_SIZE; ++i) {
        fout << setw(8) << rt[i].collisionNum;
        sum += rt[i].collisionNum;
    }   
    fout << setw(8) << sum;
    
    for (int i = 1; i <= 7; ++i) {
        fout << endl << setw(10) << i << "_BUY:";
        sum = 0;
        for (int j = 0; j < ROBOT_SIZE; ++j) {
            fout << setw(8) << rt[j].buyNum[i];
            sum += rt[j].buyNum[i];
        }
        fout << setw(8) << sum;

        fout << endl << setw(9) << i << "_SELL:";
        sum = 0;
        for (int j = 0; j < ROBOT_SIZE; ++j) {
            fout << setw(8) << rt[j].sellNum[i];
            sum += rt[j].sellNum[i];
        }
        fout << setw(8) << sum;
    }
    fout << endl << "******************************LOG INFORMATION END*******************************" << endl << endl << endl;
    fout.close();
}