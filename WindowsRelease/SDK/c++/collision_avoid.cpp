/*** 
 * @Author: Xzx
 * @Date: 2023-03-14
 * @LastEditTime: 2023-03-14 17:36
 * @LastEditors: Xzx
 * @Description: 
 *      碰撞避免，分类讨论
 ***/
#include "solution.hpp"

// 向量叉积
double crossProduct(coordinate& a, coordinate& b) {
    return a.x * b.y - a.y * b.x;
}

// 向量点乘
double dotProduct(coordinate& a, coordinate& b) {
    return a.x * b.x + a.y * b.y;
}

void collitionAvoidance() {
    // 检测机器人之间的运动向量，估计碰撞可能
    double colDiss = 2;
    for (int rt1 = 0; rt1 < 4; ++rt1) {
        if (rt[rt1].holdTime > 0) {
            // if (rt[rt1].leftOrRight) {
            //     rt[rt1].cmd.rotate = 2.5;
            // }
            // else {
            //     rt[rt1].cmd.rotate = -2.5;
            // }
            rt[rt1].cmd.forward = 0.5;
            continue;
        }
        for (int rt2 = rt1 + 1; rt2 < 4; ++rt2) {
            coordinate& a = rt[rt1].location;
            coordinate& b = rt[rt2].location;
            if (dis(a, b) < colDiss) {
                vec& lsp = rt[rt1].lsp;
                vec disVec; 
                disVec.set(b.x - a.x, b.y - a.y);
                if (dotProduct(lsp, disVec) > 0) { // 两者夹角小于90度
                    double cp = crossProduct(lsp, disVec);
                    if (cp < 0) {
                        // 对方在右手边，逆时针旋转
                        // rt[rt1].cmd.rotate = 2.5;
                        rt[rt1].cmd.forward = 0.5;
                        rt[rt1].leftOrRight = 1;
                    }
                    else {
                        // 左手边，顺时针旋转
                        // rt[rt1].cmd.rotate = -2.5;
                        rt[rt1].cmd.forward = 0.5;
                        rt[rt1].leftOrRight = 0;
                    }
                    rt[rt1].holdTime = 10; // 持续时间帧数
                    break;
                    // cerr << "collision avoidance \n";
                }
            }
        }
    }
}
