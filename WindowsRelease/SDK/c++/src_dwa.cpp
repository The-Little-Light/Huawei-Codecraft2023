#include "inc_codecraft2023.hpp"
/*** 
 * @Description: 
 *      带势能场的 DWA 动态避障算法
 ***/

double dwa_para1 = 0.5;    // 势能分量系数
double dwa_para2 = 0.25;    // 目标角度系数
double dwa_para3 = 0.25;    // 有效速度系数


// G(v, w) = dwa_para1*Pe(postion) + dwa_para2*H(loca, dest, speed) + dwa_para3*V(speed)
double motionEvaluate(coordinate postion,int rtIdx,vec speed) {
    robot& rbt = rt[rtIdx];
    coordinate& dest = rbt.haveTemDest ? rbt.temDest : rbt.curTask.destCo;
    // caculate Pe(postion)
    double pe = 0;
    for (int otherRtIdx = 0; otherRtIdx < ROBOT_SIZE; ++otherRtIdx) {
        if (otherRtIdx != rtIdx) {
            pe += cntPontEnergy(otherRtIdx, postion);
        }
    }
    // caculate H(postion, dest, speed)
    vec p2d(dest.x - postion.x, dest.y - postion.y);
    double heading = cntAngle(p2d, speed);
    // caculate V(speed), velocity to destination
    double v = dotProduct(p2d, speed) / modulusOfvector(p2d);
    return dwa_para1*pe + dwa_para2*heading + dwa_para3*v;
}

vec motionPredict(int rtIdx) {
    robot &bot = rt[rtIdx];
    int type = bot.pd_id > 0;
    double dvMax = type ? dv1Max : dv0Max;
    double daMax = type ? da1Max : da0Max;

    double score = -INF;
    vec best = vec(0,0);

    dvMax = dvMax * dt;
    daMax = daMax * dt;

    auto pathAssess = [&](vec speed)->double {

    };
    


}