/*** 
 * @Author: Xzx
 * @Date: 2023-03-14
 * @LastEditTime: 2023-03-16 15:30:31
 * @LastEditors: Xzh
 * @Description: 
 *      碰撞避免，分类讨论
 ***/
#include "solution.hpp"
// 向量模
double modulusOfVector(vec& a) {
    return sqrt(a.x*a.x + a.y*a.y);
}

// 向量叉积
double crossProduct(vec& a, vec& b) {
    return a.x * b.y - a.y * b.x;
}

// 向量点乘
double dotProduct(vec& a, vec& b) {
    return a.x * b.x + a.y * b.y;
}

// 向量夹角
double angle(vec& a, vec& b) {
    return acos(dotProduct(a, b) / (dis(a, b) * dis(a, b)));
}

// void collitionAvoidance() {
//     // 检测机器人之间的运动向量，估计碰撞可能
//     double colDiss = 3;
//     for (int rt1 = 0; rt1 < 4; ++rt1) {
        
//         // if (rt[rt1].holdTime == 0)
        
//         for (int rt2 = 0; rt2 < 4; ++rt2) {
//             if (rt1 == rt2) continue;
//             coordinate& a = rt[rt1].location;
//             coordinate& b = rt[rt2].location;
//             if (dis(a, b) < colDiss) {
//                 vec& lsp = rt[rt1].lsp;
//                 vec disVec; 
//                 disVec.set(b.x - a.x, b.y - a.y);
//                 if (dotProduct(lsp, disVec) > 0) { // 两者夹角小于90度
//                     double ratio  = 10.0 / (1 + 9 * dis(a, b));
//                     double cp = crossProduct(lsp, disVec);
//                     double ag = dotProduct(rt[rt1].lsp, rt[rt2].lsp);
//                     double para1 = min(PI, ratio * 2.5 * 1.8);
//                     double para2 = min(6.0, 6.0 / ratio);
//                     if (ag < 0){
//                         if (cp < 0) {
//                             // 对方在右手边，逆时针旋转
//                             rt[rt1].avoidance.set(-5,para1);
//                         } else {
//                             // 左手边，顺时针旋转
//                             rt[rt1].avoidance.set(-5,-para1);
//                         }
//                     }else{
//                         para2 = min(rt[rt1].cmd.forward, para2);
//                         rt[rt1].avoidance.set(5, para2);

//                     }
                    
//                     rt[rt1].holdTime = 5; // 持续时间帧数
//                     // cerr << "collision avoidance \n";
//                 }
//             }
//         }
//         if (rt[rt1].holdTime > 0) {
//             if (rt[rt1].holdTime != 5) {
//                 rt[rt1].holdTime = rt[rt1].holdTime / 2;
//             }
//             if(rt[rt1].avoidance.x < 0){
//                 rt[rt1].cmd.rotate = rt[rt1].avoidance.y;
//             }else{
//                 rt[rt1].cmd.forward = rt[rt1].avoidance.y;
//             }
//         }
//     }
// }

// 估计碰撞时间
int forecastCollision(int rt1, int rt2) {
    double colRrad = 0.54;              // 碰撞半径
    int predictNumber = 5;              // 预测帧数
    int sampleNumber = 40;              // 采样点数

    double legthTime = 1 / 50.0;        // 帧的长度
    double step = legthTime *  predictNumber / sampleNumber; // 采样步长


    coordinate& a = rt[rt1].location;
    coordinate& b = rt[rt2].location;
    vec& lsp1 = rt[rt1].lsp;
    vec& lsp2 = rt[rt2].lsp;
    coordinate a1, b1;

    a1.set(a.x, a.y), b1.set(b.x, b.y);

    for (int i = 0; i < sampleNumber; ++i) {
        a1.x += lsp1.x * step;
        a1.y += lsp1.y * step;
        b1.x += lsp2.x * step;
        b1.y += lsp2.y * step;
        if (dis(a1, b1) < colRrad * 2) {
            return i;
        }
    }
    
    return -1;
}

void collitionAvoidance() {
    double inf = 1e9;
    double holdTime = 5;  // 碰撞避免措辞持续时间帧数
    int forecastMatrix[4][4]; // 预测矩阵
    for (int i = 0; i < 4; ++i) {
        forecastMatrix[i][i] = -1;
        for (int j = i + 1; j < 4; ++j) {
            forecastMatrix[j][i] = forecastMatrix[i][j] = forecastCollision(i, j);
        }
    }

    for (int rt1 = 0; rt1 < 4; ++rt1) {
        
        vector<pair<int,int> > colRt;
        for (int rt2 = 0; rt2 < 4; ++rt2) {
            if (forecastMatrix[rt1][rt2] != -1) {
                colRt.push_back(make_pair(forecastMatrix[rt1][rt2], rt2));
            }
        }
        sort(colRt.begin(), colRt.end());
        rt[rt1].holdTime = 0;
        rt[rt1].avoidance.set(inf, rt[rt1].cmd.forward);

        for(int i = colRt.size() - 1; i >= 0; i--){
            int rt2 = colRt[i].second;
            coordinate& a = rt[rt1].location;
            coordinate& b = rt[rt2].location;

            double ratio  = 100.0 / (1 + 9 * colRt[i].first);
            // double cp = crossProduct(lsp, disVec);
            double ag = dotProduct(rt[rt1].lsp, rt[rt2].lsp);
            double para1 = min(PI, ratio * 2.5 * 1.8);
            double para2 = min(6.0, 6.0 / ratio);
            if (ag < 0){
                if (rt[rt1].avoidance.x > 0) {
                    rt[rt1].avoidance.x = para1;
                } else {
                    rt[rt1].avoidance.x = -para1;
                }
                // rt[rt1].avoidance.y = min(rt[rt1].avoidance.y, para2);
                rt[rt1].holdTime = holdTime;
            } else {
                vec& lsp = rt[rt1].lsp;
                vec disVec; 
                disVec.set(b.x - a.x, b.y - a.y);
                if (dotProduct(lsp, disVec) > 0){
                    rt[rt1].holdTime = holdTime;
                    vec lsp2 = rt[rt2].lsp;
                    double speed = dotProduct(lsp, lsp2) / modulusOfVector(lsp);
                    rt[rt1].avoidance.y = min(rt[rt1].avoidance.y, speed * 0.98);

                }
            }

        }
        if (rt[rt1].holdTime > 0) {
            if(rt[rt1].avoidance.x < inf){
                rt[rt1].cmd.rotate = rt[rt1].avoidance.x;
            }
            if(rt[rt1].avoidance.y < inf){
                rt[rt1].cmd.forward = rt[rt1].avoidance.y;
            }
        }
    }
}
