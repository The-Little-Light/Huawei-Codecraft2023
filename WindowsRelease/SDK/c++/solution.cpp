#include "solution.hpp"

/******************************
author:     xiezx
date:       2023-3-15
describe:   
    1、价值函数加入旋转时间考量
    2、降低直接把产品送到8、9号工作台的权重，优先合成
    3、估算任务需要帧长度
******************************/

bool cmp (misson& a, misson& b) {
    return a.v > b.v;
}

double dis(coordinate& c1, coordinate& c2) {
    double ans = (c1.x - c2.x)*(c1.x - c2.x) + (c1.y - c2.y)*(c1.y - c2.y);
    return sqrt(ans);
}

void findMission(vector<misson>& msNode, coordinate& rtCo, vec& lsp) {
    for (int wbIdx = 0; wbIdx < K; ++wbIdx) {
        // 寻找有现成产品的工作台
        if (wb[wbIdx].pstatus) {
            int proType = wb[wbIdx].type;
            // 遍历收购方
            for (auto buyWbIdx: type2BuyIndex[proType]) {
                // 收购方是8或9号工作台，或者，对应原材料格为空
                if (wb[buyWbIdx].type > 7 || !wb[buyWbIdx].checkHaveProType(proType)) {
                    // 此时从 wbIdx 到 buyWbIdx 是一个潜在任务
                    misson pot = misson(wbIdx, buyWbIdx, proType);
                    pot.countValue(rtCo, proType, lsp);
                    msNode.push_back(pot);
                } 
            }
        }
    }
    sort(msNode.begin(), msNode.end(), cmp);
}

// 计算两个向量的夹角
double cntAbsAngle(vec& a, vec& b){
    double angleDiff = acos(dotProduct(a, b) / (modulusOfVector(a)*modulusOfVector(b)));
    return (angleDiff);
}

void misson::countValue(coordinate& rtCo, int proType, vec& lsp) {
    // 计算价值函数 参数依次为机器人坐标、预计携带产品类型、机器人速度向量
    coordinate s = wb[startIndex].location;
    coordinate e = wb[endIndex].location;
    double dd = dis(rtCo, s) + dis(s, e);   // 机器人到起点再到终点的距离
    vec r2s(s.x - rtCo.x, s.y - rtCo.y);    // 机器人到起点向量
    vec s2e(e.x - s.x, e.y - s.y);          // 起点到终点的向量
    double rr = cntAbsAngle(lsp, r2s) + cntAbsAngle(r2s, s2e); // 任务所需转动角度和
    double tt = dd/6 + rr/PI;
    estFrame = tt * 50 + 10;
    double vv = profitAndTime[proType].first;
    if (wb[endIndex].type > 7) {
        vv *= 0.8;
    }
    v = para1 / dd + para2 * vv;
} 

void robot::checkDest() {
    if (!taskQueue.empty()) {
        task& curTask = taskQueue.front();
        if (wb_id == curTask.destId) {
            // 到达当前工作目的地，交付工作
            cmd.buy = curTask.buy;
            cmd.sell = curTask.sell;
            taskQueue.pop();
        }
    }
}

void robot::checkTask() {
    if (taskQueue.empty()) {
        // 分配新任务
        vector<misson> msNode; // 任务节点
        vec lspToward(lsp.x, lsp.y);
        if (abs(lspToward.x) < 0.00001 && abs(lspToward.y) < 0.00001) {
            lspToward.x = 5*cos(toward);
            lspToward.y = 5*sin(toward);
        }
        findMission(msNode, location, lspToward);
        bool success = false;
        for (int i = 0; i < msNode.size(); ++i) {
            misson selected = msNode[i];
            // 预计任务能在第9000帧之前完成才接单
            // cerr << "robot" << rtIdx << ": " << frameID << "   " << selected.estFrame + frameID << endl;
            if (selected.estFrame + frameID < 9000) {
                curMisson = selected;
                taskQueue.push(task(wb[selected.startIndex].location ,selected.startIndex, 1, 0));
                taskQueue.push(task(wb[selected.endIndex].location ,selected.endIndex, 0, 1));
                wb[curMisson.startIndex].pstatus = 0;
                wb[curMisson.endIndex].setProType(curMisson.proType);
                success = true;
                break;
            }
        }
        if (!success) return;
    }
    // 执行当前任务，前往目的地
    task& curTask = taskQueue.front();
    setSpeed(curTask.destCo);
}

// void motion_test(){
//     robot& tmp = rt[0];
//     if(tmp.wb_id = tmp.taskQueue.front().destId) {
//         while(1){
//             int next = rand()%K;
//             if(next != tmp.wb_id) {
//                 tmp.taskQueue.front().destId = next;
//                 break;
//             }
//         }
//     }
//     int next = tmp.taskQueue.front().destId;
//     tmp.setSpeed(wb[next].location);
//     cerr<<next<<" "<<tmp.cmd.forward<<" "<<tmp.cmd.rotate<<endl;
// }

void solution() {
    // 根据已分配任务把工作台信息进行同步
    for (int rtIdx = 0; rtIdx < 4; ++rtIdx) {
        if (rt[rtIdx].taskQueue.size() == 2) {
            misson& tmp = rt[rtIdx].curMisson;
            wb[tmp.startIndex].pstatus = 0;
            wb[tmp.endIndex].setProType(tmp.proType);
        }
        else if (rt[rtIdx].taskQueue.size() == 1) {
            misson& tmp = rt[rtIdx].curMisson;
            wb[tmp.endIndex].setProType(tmp.proType);
        }
    }
    // 指令规划
    for (int rtIdx = 0; rtIdx < 4; ++rtIdx) {
        if (rt[rtIdx].holdTime) --rt[rtIdx].holdTime;
        rt[rtIdx].cmd.clean(); // 清除之前指令设置
        rt[rtIdx].checkDest(); // 检查是否到达目的地
        rt[rtIdx].checkTask(); // 任务执行->运动指令
    }
    // 碰撞避免
    collitionAvoidance();
    return;
}