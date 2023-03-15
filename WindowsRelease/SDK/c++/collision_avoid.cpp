/*** 
 * @Author: Xzx
 * @Date: 2023-03-14
 * @LastEditTime: 2023-03-15 15:43:35
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

void collitionAvoidance() {
    // 检测机器人之间的运动向量，估计碰撞可能
    double colDiss = 2;
    for (int rt1 = 0; rt1 < 4; ++rt1) {
        if (rt[rt1].holdTime > 0) {
            if(rt[rt1].avoidance.x == 0){
                rt[rt1].cmd.rotate = rt[rt1].avoidance.y;
            }else{
                rt[rt1].cmd.forward = rt[rt1].avoidance.y;
            }
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
                    double ag = dotProduct(rt[rt1].lsp, rt[rt2].lsp);
                    if (ag < 0){
                        if (cp < 0) {
                            // 对方在右手边，逆时针旋转
                            rt[rt1].cmd.rotate = 2.5;
                            rt[rt1].leftOrRight = 1;
                            rt[rt1].avoidance.set(0,2.5);
                        }
                        else {
                            // 左手边，顺时针旋转
                            rt[rt1].cmd.rotate = -2.5;
                            rt[rt1].leftOrRight = 0;
                            rt[rt1].avoidance.set(0,-2.5);
                        }
                    }else{
                        rt[rt1].cmd.forward = min(rt[rt1].cmd.forward, 0.5);
                        rt[rt1].avoidance.set(1,rt[rt1].cmd.forward);

                    }
                    
                    rt[rt1].holdTime = 5; // 持续时间帧数
                    break;
                    // cerr << "collision avoidance \n";
                }
            }
        }
    }
}
