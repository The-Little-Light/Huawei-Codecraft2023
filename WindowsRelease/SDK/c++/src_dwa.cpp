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

    // 获取机器人最大加速度
    robot &bot = rt[rtIdx];
    int type = bot.pd_id > 0;
    double dvMax = type ? dv1Max : dv0Max;
    double daMax = type ? da1Max : da0Max;

    // 维护最佳路径
    double score = -0.5;
    vec best = vec(0,0);

    // 设置速度单位采样变化量
    dvMax = dvMax * dt;
    vec &curSpeed = bot.lsp;
    double curLineSpeed = modulusOfvector(curSpeed);
    double dv = dvMax / dwaM;

    // 设置角速度单位采样变化量
    daMax = daMax * dt;
    double &curAsp = bot.asp, da = daMax / dwaM;

    // 维护当前位置和朝向
    vec &curLocation = bot.location;
    double ly = curLocation.y, lx = curLocation.x, tmpy, tmpx;
    double curToward = bot.toward;

    double tmpasp = curAsp, tmpLineSpeed = curLineSpeed;

    // 根据设置的1帧后的速度和角速度，预测N帧后的位置和朝向，并维护最佳路径评估得分
    auto pathEvaluate = [&]() {
        double w1 = (tmpasp + curAsp) / 2.0,v1 = (tmpasp + curLineSpeed) / 2.0;
        tmpy = ly, tmpx = lx;

        // 假设速度大小、角速度变化均匀，故第一帧按平均速度、角速度计算
        double tmpToward = curToward + dt * w1;
        if (fabs(w1) <= eps) {
            tmpx += v1 * dt * cos(curToward);
            tmpy += v1 * dt * sin(curToward);
        } else {
            tmpx += v1/w1 * (sin(curToward) - sin(tmpToward));
            tmpy -= v1/w1 * (cos(curToward) - cos(tmpToward));
        }

        // 假设后N-1帧速度大小、角速度不改变，计算位置和朝向
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

        double tmpScore = motionEvaluate(vec(tmpx,tmpy),rtIdx,vec(v1*cos(tmpToward),v1*sin(tmpToward)));
        if (tmpScore > score) score = tmpScore, best.set(tmpLineSpeed,tmpasp);
    };


    // 总采样点数 : 4M^2 + 1

    // 对[v,a]进行评估
    pathEvaluate();


    // 对[v, v + dvMax]进行采样
    for (int i = 0; i < dwaM; i++) {
        tmpLineSpeed += dv;
        if (tmpLineSpeed > 6 + eps) break;
        tmpasp = curAsp;

        // 对[a, a + daMax]进行采样
        for (int j = 0; j < dwaM; j++) {
            tmpasp += da;
            if (tmpasp > PI + eps) break;
            pathEvaluate();
        }

        // 对[a - daMax, a]进行采样
        tmpasp = curAsp;
        for (int j = 0; j < dwaM; j++) {
            tmpasp -= da;
            if (tmpasp < -PI - eps) break;
            pathEvaluate();
        }
    }

    // 对[v - dvMax, v]进行采样
    tmpLineSpeed = curLineSpeed;
    for (int i = 0; i < dwaM; i++) {
        tmpLineSpeed -= dv;
        if (tmpLineSpeed <= -2 - eps) break;
        tmpasp = curAsp;

        // 对[a, a + daMax]进行采样
        for (int j = 0; j < dwaM; j++) {
            tmpasp += da;
            if (tmpasp > PI + eps) break;
            pathEvaluate();
        }

        // 对[a - daMax, a]进行采样
        tmpasp = curAsp;
        for (int j = 0; j < dwaM; j++) {
            tmpasp -= da;
            if (tmpasp < -PI - eps) break;
            pathEvaluate();
        }
    }

    return best;
}