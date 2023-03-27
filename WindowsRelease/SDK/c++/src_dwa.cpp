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
    const double eps = 1e-6;

    robot &bot = rt[rtIdx];
    int type = bot.pd_id > 0;
    double dvMax = type ? dv1Max : dv0Max;
    double daMax = type ? da1Max : da0Max;

    double score = -0.5;
    vec best = vec(0,0);

    dvMax = dvMax * dt;
    vec &curSpeed = bot.lsp;
    double curLineSpeed = modulusOfvector(curSpeed);
    double dv = dvMax / dwaM, da = daMax / dwaM;

    daMax = daMax * dt;
    double &curAsp = bot.asp;

    vec &curLocation = bot.location;
    double ly = curLocation.y, lx = curLocation.x, tmpy, tmpx;
    double curToward = bot.toward;

    double tmpasp, tmpLineSpeed = curLineSpeed;


    auto pathEvaluate = [&]() -> double {
        double w1 = (tmpasp + curAsp) / 2.0,v1 = (tmpasp + curLineSpeed) / 2.0;
        tmpy = ly, tmpx = lx;
        double tmpToward = curToward + dt * w1;
        if (fabs(w1) <= eps) {
            tmpx += v1 * dt * cos(curToward);
            tmpy += v1 * dt * sin(curToward);
        } else {
            tmpx += v1/w1 * (sin(curToward) - sin(tmpToward));
            tmpy -= v1/w1 * (cos(curToward) - cos(tmpToward));
        }

        w1 = tmpasp, v1 = tmpasp;
        if (fabs(w1) <= eps) {
            double tmp = v1 * dt * (N - 1);
            tmpx += tmp * cos(tmpToward);
            tmpy += tmp * sin(tmpToward);
        } else {
            double tmp = w1 * dt * (N - 1);
            tmpx += v1/w1 * (sin(tmpToward) - sin(tmpToward + tmp));
            tmpy -= v1/w1 * (cos(tmpToward) - cos(tmpToward + tmp));
            tmpToward += tmp;
        }

        return motionEvaluate(vec(tmpx,tmpy),rtIdx,vec(v1*cos(tmpToward),v1*sin(tmpToward)));
    };


    for (int i = 0; i < dwaM; i++) {
        tmpLineSpeed += dv;
        if (tmpLineSpeed > 6 + eps) break;
        tmpasp = curAsp;

        for (int j = 0; j < dwaM; j++) {
            tmpasp += da;
            if (tmpasp > PI + eps) break;
            if (pathEvaluate() > score) best.set(tmpLineSpeed,tmpasp);
        }

        tmpasp = curAsp;
        for (int j = 0; j < dwaM; j++) {
            tmpasp -= da;
            if (tmpasp < -PI - eps) break;
            if (pathEvaluate() > score) best.set(tmpLineSpeed,tmpasp);
        }
    }

    tmpLineSpeed = curLineSpeed;
    for (int i = 0; i < dwaM; i++) {
        tmpLineSpeed -= dv;
        if (tmpLineSpeed <= -2 - eps) break;
        tmpasp = curAsp;

        for (int j = 0; j < dwaM; j++) {
            tmpasp += da;
            if (tmpasp > PI + eps) break;
            if (pathEvaluate() > score) best.set(tmpLineSpeed,tmpasp);
        }

        tmpasp = curAsp;
        for (int j = 0; j < dwaM; j++) {
            tmpasp -= da;
            if (tmpasp < -PI - eps) break;
            if (pathEvaluate() > score) best.set(tmpLineSpeed,tmpasp);
        }
    }

    return best;
}