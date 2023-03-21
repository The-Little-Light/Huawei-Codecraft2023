#include "solution.hpp"

// 设置临时目的地
void robot::setTemporaryDest(coordinate& td) {
    temDest = td;
    haveTemDest = true;
    // 立即前往临时目的地
    setSpeed(temDest);
}

// 检查是否到达目的地
void robot::checkDest() {
    if (haveTemDest) { 
        // 检查是否到达临时目的地附近
        if (dis(temDest, location) < 0.5) {
            // 视为到达
            haveTemDest = false;
        }
    }
    else {
        if (!taskQueue.empty()) {
            task& curTask = taskQueue.front();
            if (wb_id == curTask.destId) {
                // 到达当前工作目的地，交付工作
                cmd.buy = curTask.buy;
                if (cmd.buy) ++buyNum[curMission.proType][rtIdx-1];
                cmd.sell = curTask.sell;
                if (cmd.sell) ++sellNum[curMission.proType][rtIdx-1];
                taskQueue.pop();
            }
        }
    }
}

// 检查任务队列情况
void robot::checkTask() {
    if (taskQueue.empty()) {
        // 分配新任务
        vector<misson> msNode; // 任务节点
        findMission(msNode, location, lsp);
        bool success = false;
        for (int i = 0; i < msNode.size(); ++i) {
            misson selected = msNode[i];
            // 预计任务能在第9000帧之前完成才接单
            // cerr << "robot" << rtIdx << ": " << frameID << "   " << selected.estFrame + frameID << endl;
            if (selected.estFrame + frameID < 9000) {
                curMission = selected;
                taskQueue.push(task(wb[selected.startIndex].location ,selected.startIndex, 1, 0));
                taskQueue.push(task(wb[selected.endIndex].location ,selected.endIndex, 0, 1));
                wb[curMission.startIndex].pstatus = 0;
                wb[curMission.endIndex].setProType(curMission.proType);
                success = true;
                break;
            }
        }
        if (!success) return;
    }
    if (haveTemDest) {
        // 前往临时目的地
        setSpeed(temDest);
    }
    else {
        // 执行当前任务，前往目的地
        task& curTask = taskQueue.front();
        setSpeed(curTask.destCo);
    }
}

// 速度过低时用朝向来为其赋一个明确速度
void robot::checkSpeed() {
    if (fabs(lsp.x) < 0.01 && fabs(lsp.y) < 0.01) {
        lsp.x += 1*cos(toward);
        lsp.y += 1*sin(toward);
    }
}

bool cmp (misson& a, misson& b) {
    return a.v > b.v;
}

void robot::findMission(vector<misson>& msNode, coordinate& rtCo, vec& lsp) {
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

// MCMF中检查目的地
void robot::MCMFcheckDest() {
    if (haveTemDest) { 
        // 检查是否到达临时目的地附近
        if (dis(temDest, location) < 0.5) {
            // 视为到达
            haveTemDest = false;
        }
    } else {
        if (curTask.checkVaild()) {
            if (wb_id == curTask.destId) {
                // 到达当前工作目的地，交付工作
                cmd.buy = curTask.buy;
                cmd.sell = curTask.sell;
                
                if (cmd.sell) {
                    if (wb[wb_id].type < 8) {
                        int index = 0;
                        switch (wb[wb_id].type) {
                        case 4:
                            index = pd_id == 2;
                        case 5:
                        case 6:
                            index = pd_id == 3;
                            break;
                        case 7:
                            index = pd_id - 4;
                            break;
                        default:
                            break;
                        }
                        int id = curFlow.workbenchProductId[wb_id][index];
                        curFlow.setEdgeCap(id,0,0);
                    }
                    curFlow.releaseNode(nodeId); 
                    nodeId = -1,pd_id = 0;
                } else {
                    // TODO: assume robot must buy material here,is it not realistic
                    curFlow.lockNode(rtIdx,wb_id);
                    pd_id = wb[wb_id].type;
                }
                curTask.setVaild(0);
            }
        }
    }
}