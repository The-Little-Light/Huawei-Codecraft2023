#include "solution.hpp"

/******************************
author:     xiezx
date:       2023-3-11
describe:   第一版bassline
******************************/

double para1 = 1;
double para2 = 100;
double para3 = 1;

bool cmp(misson& a, misson& b) {
    return a.v > b.v;
}

void findMission(vector<misson>& msNode) {
    for (int wbIdx = 0; wbIdx < K; ++wbIdx) {
        // 寻找有现成产品的工作台
        if (wb[wbIdx].pstatus) {
            int proType = wb[wbIdx].type;
            // 遍历收购方
            for (auto buyWbIdx: type2BuyIndex[proType]) {
                // 对应原材料格为空
                if (!wb[buyWbIdx].checkHaveProType(proType)) {
                    // 此时从 wbIdx 到 buyWbIdx 是一个潜在任务
                    msNode.push_back(misson(wbIdx, buyWbIdx, proType));
                }
            }
        }
    }
    sort(msNode.begin(), msNode.end(), cmp);
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
        findMission(msNode);
        if (msNode.size() == 0) return;
        misson selected = msNode[0];
        curMisson = selected;
        taskQueue.push(task(wb[selected.startIndex].location ,selected.startIndex));
        taskQueue.push(task(wb[selected.endIndex].location ,selected.endIndex));
        wb[curMisson.startIndex].pstatus = 0;
        wb[curMisson.startIndex].setProType(curMisson.proType);
    }
    // 执行当前任务，前往目的地
    task& curTask = taskQueue.front();
    setSpeed(curTask.destCo);
}
void motion_test(){
    robot& tmp = rt[0];
    if(tmp.wb_id = tmp.taskQueue.front().destId) {
        while(1){
            int next = rand()%K;
            if(next != tmp.wb_id) {
                tmp.taskQueue.front().destId = next;
                break;
            }
        }
    }
    int next = tmp.taskQueue.front().destId;
    tmp.setSpeed(wb[next].location);
    cerr<<next<<" "<<tmp.cmd.forward<<" "<<tmp.cmd.rotate<<endl;
}
void solution() {
    // // 根据已分配任务把工作台信息进行同步
    // for (int rtIdx = 0; rtIdx < 4; ++rtIdx) {
    //     if (rt[rtIdx].taskQueue.size() == 2) {
    //         misson& tmp = rt[rtIdx].curMisson;
    //         wb[tmp.startIndex].pstatus = 0;
    //         wb[tmp.startIndex].setProType(tmp.proType);
    //     }
    //     else if (rt[rtIdx].taskQueue.size() == 1) {
    //         misson& tmp = rt[rtIdx].curMisson;
    //         wb[tmp.startIndex].setProType(tmp.proType);
    //     }
    // }
    // // 指令规划
    // for (int rtIdx = 0; rtIdx < 4; ++rtIdx) {
    //     rt[rtIdx].checkDest();
    //     rt[rtIdx].checkTask();
    // }
    motion_test();
    return;
}