#include "solution.hpp"

/******************************
author:     xiezx
date:       2023-3-13
describe:   运动路径优化+碰撞避免 
******************************/

bool cmp(misson& a, misson& b) {
    return a.v > b.v;
}

double dis(coordinate& c1, coordinate& c2) {
    double ans = (c1.x - c2.x)*(c1.x - c2.x) + (c1.y - c2.y)*(c1.y - c2.y);
    return sqrt(ans);
}

void findMission(vector<misson>& msNode, coordinate& rtCo) {
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
                    pot.countValue(rtCo, proType);
                    msNode.push_back(pot);
                } 
            }
        }
    }
    sort(msNode.begin(), msNode.end(), cmp);
}

void misson::countValue(coordinate& rtCo, int proType) {
    // 计算价值函数
    double dd = dis(rtCo, wb[startIndex].location) 
        + dis(wb[startIndex].location, wb[endIndex].location);
    double vv = profitAndTime[proType].first;
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
        // 无空闲任务时如何处理？
        vector<misson> msNode; // 任务节点
        findMission(msNode, location);
        if (msNode.size() == 0) return;
        misson selected = msNode[0];
        curMisson = selected;
        taskQueue.push(task(wb[selected.startIndex].location ,selected.startIndex, 1, 0));
        taskQueue.push(task(wb[selected.endIndex].location ,selected.endIndex, 0, 1));
        wb[curMisson.startIndex].pstatus = 0;
        wb[curMisson.endIndex].setProType(curMisson.proType);
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