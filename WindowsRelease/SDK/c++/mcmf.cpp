/*** 
 * @Author: Xzh
 * @Date: 2023-03-20 22:55:25
 * @LastEditTime: 2023-03-22 22:21:33
 * @LastEditors: Xzh
 * @Description: 
 *      引入最小费用最大流进行全局任务规划，优化任务分配
 */

#include "solution.hpp"


void mcmf::init(){
    S = getNode(), T = getNode();
    for (int i = 0; i < N; i++) robotId[i] = getNode(),addEdge(S,robotId[i],0,1);
    for (int i = 0; i < K; i++) {
        workbenchId[i] = -1;
        for(int j = 0; j < 3; j++) workbenchProductId[i][j] = -1;
    }

    vector<int> index;      //与T相连的工作台

    // 为原料格加点加边+维护index数组
    auto setID = [&](int &pst,int wbIdx,int cap = inf){
        pst = getNode();
        addEdge(pst,T,0,cap);
        index.push_back(pst);
        ProductId2Workbench[pst] = wbIdx;
    };
    // 建立原料格对汇点的边
    for (int wbIdx = 0; wbIdx < K; ++wbIdx) {
        switch (wb[wbIdx].type) {
        case 7:
            setID(workbenchProductId[wbIdx][2],wbIdx,1);
        case 4:
        case 5:
        case 6:
            setID(workbenchProductId[wbIdx][1],wbIdx,1);
            setID(workbenchProductId[wbIdx][0],wbIdx,1);
            break;
        case 8:
        case 9:
            setID(workbenchProductId[wbIdx][0],wbIdx);
            break;
        default:
            break;
        }
    }
    // 建立产品型号到收购方原材料格id的映射
    for (int wbIdx = 0; wbIdx < K; ++wbIdx) {
        switch (wb[wbIdx].type) {
        case 4:
            produce2sell[1].insert(workbenchProductId[wbIdx][0]);
            produce2sell[2].insert(workbenchProductId[wbIdx][1]);
            break;
        case 5:
            produce2sell[1].insert(workbenchProductId[wbIdx][0]);
            produce2sell[3].insert(workbenchProductId[wbIdx][1]);
            break;
        case 6:
            produce2sell[2].insert(workbenchProductId[wbIdx][0]);
            produce2sell[3].insert(workbenchProductId[wbIdx][1]);
            break;
        case 7:
            produce2sell[4].insert(workbenchProductId[wbIdx][0]);
            produce2sell[5].insert(workbenchProductId[wbIdx][1]);
            produce2sell[6].insert(workbenchProductId[wbIdx][2]);
            break;
        case 8:
            produce2sell[7].insert(workbenchProductId[wbIdx][0]);
            break;
        case 9:
            for (int i = 1; i <= 7; ++i) {
                produce2sell[i].insert(workbenchProductId[wbIdx][0]);
            }
            break;
        default:
            break;
        }
    }

    // 初始化空闲产品节点池
    if (cnt&1) getNode();
    for (; curSize < poolSize; curSize++) {
        int indexSize = index.size();
        pool[curSize] = getNode();
        int codNode = getNode();

        for (int i = 0; i < N; i++) addEdge(robotId[i],pool[curSize],0,0);
        addEdge(pool[curSize],codNode,0,1);
        for (int i = 0; i < indexSize; i++) addEdge(codNode,index[i],0,0);
    }
}

//静态价值参数调整
double mcmf::countValue(int proType,int startIndex,int endIndex) {
    coordinate s = wb[startIndex].location;
    coordinate e = wb[endIndex].location;
    double dd = dis(s, e);                  // 机器人从起点到终点的距离
    double tt = dd/6 + 1;
    double vv = profitAndTime[proType].first.first; // 出售收入
    double nextVv = profitAndTime[wb[endIndex].type].first.first - profitAndTime[wb[endIndex].type].first.second;
    // 考虑剩余原材料格对价值的影响，目标工作台的剩余材料格越少越重视
    if (wb[endIndex].type > 7) {
        vv *= 0.8;
    }
    else if (wb[endIndex].type == 7) {
        vv += 0.35*nextVv/(4-wb[endIndex].rawMaterNum());
    }
    else if (wb[endIndex].type > 3) {
        vv += 0.35*nextVv/(3-wb[endIndex].rawMaterNum());
    }
    return  - ( para2 * vv + para1 / tt) + inf;
}

//TODO is it can merge with above function
double mcmf::countSellValue(int proType,int rtIdx,int endIndex){
    coordinate s = rt[rtIdx].location;
    coordinate e = wb[endIndex].location;
    
    double dd = dis(s, e);                      // 机器人到终点的距离
    vec s2e(e.x - s.x, e.y - s.y);              // 起点到终点的向量
    double rr = cntAngle(rt[rtIdx].lsp, s2e);   // 任务所需转动角度和
    double tt = dd/6 + rr/PI + 1;
    double vv = profitAndTime[proType].first.first; // 出售收入
    double nextVv = profitAndTime[wb[endIndex].type].first.first - profitAndTime[wb[endIndex].type].first.second;
    // 考虑剩余原材料格对价值的影响，目标工作台的剩余材料格越少越重视
    if (wb[endIndex].type > 7) {
        vv *= 0.8;
    }
    else if (wb[endIndex].type == 7) {
        vv += 0.35*nextVv/(4-wb[endIndex].rawMaterNum());
    }
    else if (wb[endIndex].type > 3) {
        vv += 0.35*nextVv/(3-wb[endIndex].rawMaterNum());
    }
    return  - ( para2 * vv + para1 / tt) + inf;
}

double mcmf::countBuyValue(int proType,int rtIdx,int endIndex) {
    coordinate s = rt[rtIdx].location;
    coordinate e = wb[endIndex].location;
    double dd = dis(s, e);                    // 机器人到终点的距离
    vec s2e(e.x - s.x, e.y - s.y);            // 机器人到终点的向量
    double rr = cntAngle(rt[rtIdx].lsp, s2e); // 任务所需转动角度和
    double tt = dd/6 + rr/PI + 1;
    // double vv = -profitAndTime[proType].first.second; // 已购入支出
    int nextPro = wb[endIndex].type;
    // vv -= -profitAndTime[nextPro].first.second; // 先预计购入支出
    if (proType != 0) return INF*2;
    return  -(para1 / tt) + inf;
}


void mcmf::allocateNode(int wbIdx){
    if (workbenchId[wbIdx] == -1) {
        workbenchId[wbIdx] = pool[--curSize];
        int id = workbenchId[wbIdx],type = wb[wbIdx].type;
        ProductId2Workbench[id] = wbIdx;
        ProductId2Workbench[id ^ 1] = wbIdx;
        
        for (int i = 0; i < N; i++) {
            edge &tmp = G[id][i];
            tmp.cap = 0;
            G[tmp.to][tmp.rev].cap = 1;
        }
        G[id][N].cap = 1;
        id ^= 1;
        G[id][0].cap = 0;

        for (int index = 1,size = G[id].size(); index < size; index++) {
            edge &tmp = G[id][index];
            int towbIdx = ProductId2Workbench[tmp.to];
            if (produce2sell[type].count(tmp.to)) {
                tmp.cap = 1, G[tmp.to][tmp.rev].cap = 0;
                tmp.cost = countValue(type,wbIdx,towbIdx);
                G[tmp.to][tmp.rev].cost = -tmp.cost;
            }

        }
    }
}

int mcmf::spfa(){
    fill(shortDis, shortDis + cnt, INF);
    memset(vis,0,sizeof(int)*(cnt));
    memset(cut,0,sizeof(int)*(cnt));
    deque<int> q;
    int k = cnt;

    vis[S] = 1,shortDis[S] = 0;
    q.push_back(S);

    while(q.size()){
        int n = q.front();q.pop_front();
        double d = shortDis[n];
        vis[n] = 0;

        for (int i = 0, size = G[n].size(); i < size; i++) {
            const edge& e = G[n][i];
            int to = e.to;

            if (e.cap > 0 && shortDis[to] > d + e.cost + eps) {
                shortDis[to] = d + e.cost + eps,cut[to] = cut[n] + 1,pre[to] = n,pe[to] = i;
                if(cut[to] > k-1) return -1;
                if(!vis[to])  vis[to] = 1,(q.size()&&shortDis[to] < shortDis[q.front()] - eps)?q.push_front(to):q.push_back(to);//slf优化
            }
        }
    }
    return cut[T];
}

int mcmf::solve() {
    if(spfa() < 0) {
        cerr << "fatal mcmf error!!!!" << endl;
        return -1;
    }
    bufCur = 0;
    flow = 0;
    while(spfa() > 0){
        int newflow = inf,index = 0;
        for(int x = T; x != S; x = pre[x])  newflow = min(newflow,G[pre[x]][pe[x]].cap);
        for(int x = T; x != S; x = pre[x])  {
            stateBuf[bufCur][index][0] = pre[x],stateBuf[bufCur][index++][1] = pe[x];
            edge&ed = G[pre[x]][pe[x]];
            ed.cap -= newflow;
            G[ed.to][ed.rev].cap += newflow;
        }
        flow += newflow;
        ++bufCur;
    }
    return 1;
}

void mcmf::showNodeEdge(int id, int condition) {
    if (condition) {
        fprintf(stderr,"\n");
        for (int i = 0,size = G[id].size(); i < size; i++) {
            edge&ed = G[id][i];
            if (G[ed.to][ed.rev].cap || 1) {
                fprintf(stderr," %d %d %d -------> %d %d\n",id,i,ed.cap,ed.to,G[ed.to][ed.rev].cap);
            }
        }
        fprintf(stderr,"\n");
    }
}

void mcmf::showFlow(int condition,int detailed) {
    if (condition) {
        fprintf(stderr,"frameID : %d \n",frameID);
        for (int i = 0; i < bufCur; i++){
            int index = 0,pe,pv;
            fprintf(stderr,"%d ",T);
            do {
                pv = stateBuf[i][index][0],pe = stateBuf[i][index][1];
                ++index;
                fprintf(stderr,"<------ %d ",pv);
            }while(pv != S);
            fprintf(stderr,"\n");
        }
        fprintf(stderr,"\n");
        if (detailed) {
            fprintf(stderr,"\n");
            for (int i = 0; i < bufCur; i++){
                int index = 0,pe,pv;
                do {
                    pv = stateBuf[i][index][0],pe = stateBuf[i][index][1];
                    edge &ed = G[pv][pe];
                    fprintf(stderr,"%d %d -----> %d %d\n",pv,ed.cap,ed.to,G[ed.to][ed.rev].cap);
                    ++index;
                }while(pv != S);
                fprintf(stderr,"\n");
            }
            fprintf(stderr,"\n");
        }
    }

}

void mcmf::resetCap(){
    for (int i = 0; i < bufCur; i++){
        int index = 0,pe,pv;
        do {
            pv = stateBuf[i][index][0],pe = stateBuf[i][index][1];
            ++index;
            edge&ed = G[pv][pe];
            ed.cap += 1;
            G[ed.to][ed.rev].cap -= 1;
        }while(pv != S);
    }
}

void mcmf::releaseNode(int id){
    for(edge&ed:G[id]){
        ed.cap = G[ed.to][ed.rev].cap = 0;
    }
    pool[curSize++] = id;
    id ^= 1;
    for(edge&ed:G[id]){
        ed.cap = G[ed.to][ed.rev].cap = 0;
    }
}

void mcmf::lockNode(int rtIdx,int wbIdx){
    rt[rtIdx].nodeId = workbenchId[wbIdx];
    workbenchId[wbIdx] = -1;
}

void mcmf::adjustEdge(int rtIdx){
    int id = robotId[rtIdx];
    edge&tmp = G[id][0]; // robot to S
    tmp.cap = 0,G[S][tmp.rev].cap = 1;

    int nodeId = rt[rtIdx].nodeId,type = rt[rtIdx].pd_id;
    for (int i = 1,size = G[id].size(); i < size; i++) {
        edge&ed = G[id][i];
        if (ed.to != nodeId) {
            if (!ed.cap) continue;
            ed.cost = countBuyValue(type,rtIdx,ProductId2Workbench[ed.to]);
            G[ed.to][ed.rev].cost = -ed.cost;
            // if (nodeId < 0) ed.cap = 1,G[tmp.to][tmp.rev].cap = 0;
        } else ed.cost = inf,G[ed.to][ed.rev].cost = -ed.cost;
    }

    if (~nodeId) {
        for (int i = 0; i < N; i++) {
            edge &tmp = G[nodeId][i];
            tmp.cap = 0;
            G[tmp.to][tmp.rev].cap = id == tmp.to;
        }

        G[nodeId][N].cap = 1;
        nodeId ^= 1;
        G[nodeId][0].cap = 0;
        
        for (int index = 1,size = G[nodeId].size(); index < size; index++) {
            edge &tmp = G[nodeId][index];
            int towbIdx = ProductId2Workbench[tmp.to];
            if (produce2sell[type].count(tmp.to)) {
                tmp.cap = 1, G[tmp.to][tmp.rev].cap = 0;
                tmp.cost = countSellValue(type,rtIdx,towbIdx);
                G[tmp.to][tmp.rev].cost = -tmp.cost;
            }
        }
    } 
}

/**
 *return value : 
 *   0 when a is normal value;
 *   1 when a is NaN;
 *   0 when a is inf;
 */
int mcmf::checkVaild(double a) {
    if (isnan(a)) {
        return 1;
    } else if (isinf(a)) {
        return 2;
    } else {
        return 0;
    }
}

void mcmf::adjustTask(int rtIdx){
    int id = robotId[rtIdx];

    int nextId = -1,nodeId = rt[rtIdx].nodeId,type = rt[rtIdx].pd_id;

    if (~nodeId) {
        if (G[nodeId][rtIdx].cap) {
            nodeId ^= 1;
            for (int index = 1,size = G[nodeId].size(); index < size; index++) {
                edge &ed = G[nodeId][index];
                // TODO can first check original dest
                if (G[ed.to][ed.rev].cap) {
                    int wbIdx = ProductId2Workbench[ed.to];
                    rt[rtIdx].curTask = task(wb[wbIdx].location, wbIdx,0,1);
                    break;
                }
            }
        } else {
            cerr << "destroyed " <<rtIdx << "  curframeID " <<frameID<< endl;
            int debugLevel = 1;    // 调试级别 0不输出报错信息,1表示简略的报错信息，2是详细的报错信息。
            int errorType = 0;

            if (debugLevel) {
                auto check = [&](int id) -> int {
                    int flag = 0;
                    for (int i = 0,size = G[id].size(); i < size; i++) {
                        edge&ed = G[id][i];
                        if (G[ed.to][ed.rev].cap || ed.cap) {
                            if (debugLevel > 1)
                            fprintf(stderr,"%d %d %d -------> %d %d %f %f\n",id,i,ed.cap,ed.to,G[ed.to][ed.rev].cap,ed.cost,G[ed.to][ed.rev].cost);
                            flag |= checkVaild(ed.cost);
                        }
                    }
                    if (debugLevel > 1) cerr << endl;
                    errorType |= flag;
                    return flag;
                };

                auto printError = [&](const char *pre,int flag) {
                    if (flag & 1) fprintf(stderr,"%s   NaN checked\n",pre);
                    if (flag & 2) fprintf(stderr,"%s   inf checked\n",pre);       
                    if (flag & 4) fprintf(stderr,"%s   Unknown Error\n",pre);       
                };

                printError("countBuyValue",check(id));

                cerr << "nodeId "<<nodeId << endl;
                check(nodeId);

                int tmpId = nodeId ^ 1;
                printError("countSellValue",check(tmpId));


                vector<int> de;
                for (int i = 0,size = G[tmpId].size(); i < size; i++) {
                    edge&ed = G[tmpId][i];
                    if (ed.cap) de.push_back(ed.to);
                }

                for (auto id : de) {
                    printError("countSellValue | countValue",check(id));
                }

                if (!errorType) printError("",4);
                // showNodeEdge(T);
                
                // for (int wbIdx = 0; wbIdx < K; ++wbIdx) {
                //     int rstatus = wb[wbIdx].rstatus;
                //     fprintf(stderr," wbIdx : %d  rstatus : %d\n", wbIdx, rstatus);
                // }

                showFlow();
                cerr << endl;
            }
            
            // TODO if this situation happen,this solution
        }
    } else {
        for (int i = 1,size = G[id].size(); i < size; i++) {
            edge&ed = G[id][i];
            if (G[ed.to][ed.rev].cap) {
                int wbIdx = ProductId2Workbench[ed.to];
                rt[rtIdx].curTask = task(wb[wbIdx].location, wbIdx,1,0);
                break;
            }
        }
    }
}

void mcmf::checkDest(int rtIdx) {
    robot &bot = rt[rtIdx];
    if (bot.haveTemDest) { 
        // 检查是否到达临时目的地附近
        if (dis(bot.temDest, bot.location) < 0.5) {
            // 视为到达
            bot.haveTemDest = false;
        }
    } else {
        if (bot.curTask.destId != -1) {
            if (bot.wb_id == bot.curTask.destId) {
                // 到达当前工作目的地，交付工作
                bot.cmd.sell = bot.curTask.sell;
                bot.cmd.buy = bot.curTask.buy;
                
                if (bot.cmd.sell) {
                    if (wb[bot.wb_id].type < 8) {
                        int index = 0;
                        switch (wb[bot.wb_id].type) {
                        case 4:
                            index = bot.pd_id == 2;
                        case 5:
                        case 6:
                            index = bot.pd_id == 3;
                            break;
                        case 7:
                            index = bot.pd_id - 4;
                            break;
                        default:
                            break;
                        }
                        int id = workbenchProductId[bot.wb_id][index];
                        setEdgeCap(id,0,0);
                    }
                    releaseNode(bot.nodeId); 
                    bot.nodeId = -1,bot.pd_id = 0;
                } else {
                    // TODO: assume robot must buy material here,is it not realistic
                    lockNode(rtIdx,bot.wb_id);
                    bot.pd_id = wb[bot.wb_id].type;
                }
                bot.curTask.setVaild(0);
            }
        }
    }
}

void mcmf::solution() {
    // 检查工作台状态
    //TODO parallel
    for (int wbIdx = 0; wbIdx < K; ++wbIdx) {
        if (wb[wbIdx].pstatus) {
            allocateNode(wbIdx);
        }
        int rstatus = wb[wbIdx].rstatus;
        auto testAndSetEdgeCap = [&](int bit,int index){
            setEdgeCap(workbenchProductId[wbIdx][index],0, !((rstatus >> bit) & 1));
        };
        switch (wb[wbIdx].type) {
        case 4:
            testAndSetEdgeCap(1,0);
            testAndSetEdgeCap(2,1);
            break;
        case 5:
            testAndSetEdgeCap(1,0);
            testAndSetEdgeCap(3,1);
            break;
        case 6:
            testAndSetEdgeCap(2,0);
            testAndSetEdgeCap(3,1);
            break;
        case 7:
            testAndSetEdgeCap(4,0);
            testAndSetEdgeCap(5,1);
            testAndSetEdgeCap(6,2);
            break;
        default:
            break;
        }
    }

    //TODO parallel
    // 检查机器人运动状态
    for (int rtIdx = 0; rtIdx < N; ++rtIdx) {
        if (rt[rtIdx].holdTime) --rt[rtIdx].holdTime;
        rt[rtIdx].cmd.clean(); // 清除之前指令设置
        rt[rtIdx].checkSpeed();// 保证速度非0
        checkDest(rtIdx);// 检查是否到达目的地
        adjustEdge(rtIdx);// 调整网络
    }
    // 指令规划
    solve(); // 费用流运行
    
    auto setDest = [&](int rtIdx){
        if (rt[rtIdx].haveTemDest) {
            rt[rtIdx].setSpeed(rt[rtIdx].temDest);
        } else if(rt[rtIdx].curTask.checkVaild()) {
            rt[rtIdx].setSpeed(rt[rtIdx].curTask.destCo);
        }
    };
    //TODO parallel
    // 检查调整机器人任务执行
    for (int rtIdx = 0; rtIdx < N; ++rtIdx) {
        adjustTask(rtIdx);// 调整任务
        setDest(rtIdx);
    }

    // 碰撞避免
    collitionAvoidance();

    auto printError = [&]() {
        if (flow != N && frameID >= 50) {
            fprintf(stderr, "flow Error curframeID : %d\n\n",frameID);
        }
    };

    // showFlow(frameID >= 1000 && frameID <= 1500);
    // showNodeEdge(T);
    // printError();

    resetCap();
    return;
}