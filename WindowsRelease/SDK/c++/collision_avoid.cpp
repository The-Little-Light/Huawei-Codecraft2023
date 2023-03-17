/*** 
 * @Author: Xzx
 * @Date: 2023-03-14
 * @LastEditTime: 2023-03-17 01:17:15
 * @LastEditors: Xzx
 * @Description: 
 *      引入势能场的概念，把碰撞处理从紧急避让变成引入势能较低点作为临时目的地
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

// 计算势能分布 a 为角度，lsp 为线速度向量
double cntR(double a, vec& lsp) {
    double lm = modulusOfVector(lsp);
    double e_up = -1 * a * a * lm * lm / 36;
    return exp(e_up) * lm / 6;
}

// 计算机器人 rtIdx 对点 d 的势能
double cntPontEnergy(int rtIdx, coordinate& d) {
    robot& rbt = rt[rtIdx];
    vec r2d(d.x - rbt.location.x, d.y - rbt.location.y); 
    double angle = cntAngle(rbt.lsp, r2d);
    return 1.2 * cntR(angle, rbt.lsp) / dis(d, rbt.location);
}

// 设置临时目的地
void robot::setTemporaryDest(coordinate& td) {
    temDest = td;
    haveTemDest = true;
    // 立即前往临时目的地
    setSpeed(temDest);
}

void collitionAvoidance() {
    double u = 0.5; // 拥塞阈值
    for (int curRt = 0; curRt < 4; ++curRt) {
        // if (rt[curRt].haveTemDest) continue;
        // 枚举每个机器人，计算其碰撞势能检测点受到的势能
        double pe = 0.0;                        // potentail energy
        pair<double, int> maxPeComponent(0,0);  // 维护势能分量最大的机器人
        coordinate detectPoint;                 // 探测点
        coordinate& rLoca = rt[curRt].location;
        vec& lsp = rt[curRt].lsp;
        detectPoint.set(rLoca.x + 0.4 * lsp.x, rLoca.y + 0.4 * lsp.y);
        for (int otherRt = 0; otherRt < 4; ++otherRt) {
            if (curRt == otherRt) continue;
            double peComponent = cntPontEnergy(otherRt, detectPoint);
            pe += peComponent;
            if (maxPeComponent.first < peComponent) {
                // 记录最大势能分量贡献者
                maxPeComponent.first = peComponent;
                maxPeComponent.second = otherRt;
            }
        }
        if (pe >= u) {
            // 需要进行碰撞避免，进行让路者选举
            double lm1 = modulusOfVector(lsp);
            double lm2 = modulusOfVector(rt[maxPeComponent.second].lsp);
            if (lm1 <= lm2) {
                // fprintf(stderr, "cur speed:%.2f\n",lm1);
                // 两个避让候选点根据势能选择低势能者为临时目的地
                double rot = PI/6;
                vec l_lsp(lsp.x * cos(rot) + lsp.y * sin(rot), -lsp.x * sin(rot) + lsp.y * cos(rot));     // 逆时针旋转60°
                vec r_lsp(lsp.x * cos(-rot) + lsp.y * sin(-rot), -lsp.x * sin(-rot) + lsp.y * cos(-rot)); // 顺时针旋转60°
                coordinate aLeft(rLoca.x + 0.2 * l_lsp.x, rLoca.y + 0.2 * l_lsp.y);
                coordinate aRight(rLoca.x + 0.2 * r_lsp.x, rLoca.y + 0.2 * r_lsp.y);
                double aLeftPe = 0, aRightPe = 0;
                for (int otherRt = 0; otherRt < 4; ++otherRt) {
                    if (curRt == otherRt) continue;
                    aLeftPe += cntPontEnergy(otherRt, aLeft);
                    aRightPe += cntPontEnergy(otherRt, aRight);
                }
                if (aLeftPe <= aRightPe) {
                    rt[curRt].setTemporaryDest(aLeft);
                }
                else {
                    rt[curRt].setTemporaryDest(aRight);
                }
                // fprintf(stderr,"rt[%d] local:%.2f %.2f  tmp:%.2f %.2f\n",rt[curRt].rtIdx,rLoca.x, rLoca.y, rt[curRt].temDest.x, rt[curRt].temDest.y);
            }
        }
    }
    return;
}

void ori_collitionAvoidance() {
    // 检测机器人之间的运动向量，估计碰撞可能
    double colDiss = 3;
    for (int rt1 = 0; rt1 < 4; ++rt1) {
        
        // if (rt[rt1].holdTime == 0)
        
        for (int rt2 = 0; rt2 < 4; ++rt2) {
            if (rt1 == rt2) continue;
            coordinate& a = rt[rt1].location;
            coordinate& b = rt[rt2].location;
            if (dis(a, b) < colDiss) {
                vec& lsp = rt[rt1].lsp;
                vec disVec; 
                disVec.set(b.x - a.x, b.y - a.y);
                if (dotProduct(lsp, disVec) > 0) { // 两者夹角小于90度
                    double ratio  = 10.0 / (1 + 9 * dis(a, b));
                    double cp = crossProduct(lsp, disVec);
                    double ag = dotProduct(rt[rt1].lsp, rt[rt2].lsp);
                    double para1 = min(PI, ratio * 2.5 * 1.8);
                    double para2 = min(6.0, 6.0 / ratio);
                    if (ag < 0){
                        if (cp < 0) {
                            // 对方在右手边，逆时针旋转
                            rt[rt1].avoidance.set(-5,para1);
                        } else {
                            // 左手边，顺时针旋转
                            rt[rt1].avoidance.set(-5,-para1);
                        }
                    }else{
                        para2 = min(rt[rt1].cmd.forward, para2);
                        rt[rt1].avoidance.set(5, para2);

                    }
                    
                    rt[rt1].holdTime = 5; // 持续时间帧数
                    // cerr << "collision avoidance \n";
                }
            }
        }
        if (rt[rt1].holdTime > 0) {
            if (rt[rt1].holdTime != 5) {
                rt[rt1].holdTime = rt[rt1].holdTime / 2;
            }
            if(rt[rt1].avoidance.x < 0){
                rt[rt1].cmd.rotate = rt[rt1].avoidance.y;
            }else{
                rt[rt1].cmd.forward = rt[rt1].avoidance.y;
            }
        }
    }
}
