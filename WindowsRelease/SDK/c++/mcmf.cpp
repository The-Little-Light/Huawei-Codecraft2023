/*** 
 * @Author: Xzh
 * @Date: 2023-03-20 22:55:25
 * @LastEditTime: 2023-03-21 10:09:20
 * @LastEditors: Xzh
 * @Description: 
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

    // 为原料格加点加点+维护index数组
    auto setID = [&](int &pst,int wbIdx,int value = inf){
        pst = getNode();
        addEdge(pst,T,0,value);
        index.push_back(pst);
        ProductId2Workbench[pst] = wbIdx;
    };

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

    produce2sell[1] = set<int> {4,5,9};
    produce2sell[2] = set<int> {4,6,9};
    produce2sell[3] = set<int> {5,6,9};
    produce2sell[4] = set<int> {7,9};
    produce2sell[5] = set<int> {7,9};
    produce2sell[6] = set<int> {7,9};
    produce2sell[7] = set<int> {8,9};

    // 初始化空闲产品节点池
    for (; curSize < poolSize; curSize++) {
        if (cnt&1) getNode();
        int indexSize = index.size();
        pool[curSize] = getNode();

        int codNode = getNode();
        for (int i = 0; i < N; i++) addEdge(robotId[i],pool[curSize],0,0);
        addEdge(pool[curSize],codNode,0,1);
        for (int i = 0; i < indexSize; i++) addEdge(codNode,index[i],0,0);
    }
}

//TODO 静态价值参数调整
double mcmf::countValue(int proType,int startIndex,int endIndex) {
    coordinate s = wb[startIndex].location;
    coordinate e = wb[endIndex].location;
    double dd = dis(s, e);                  // 机器人从起点到终点的距离
    vec s2e(e.x - s.x, e.y - s.y);          // 起点到终点的向量
    double tt = dd/6;
    // estFrame = tt * 50 + 50;

    double vv = profitAndTime[proType].first;
    // 考虑剩余原材料格对价值的影响，目标工作台的剩余材料格越少越重视
    if (wb[endIndex].type > 7) {
        vv *= 0.8;
    }
    else if (wb[endIndex].type == 7) {
        vv += 0.35*profitAndTime[wb[endIndex].type].first/(3-wb[endIndex].rawMaterNum());
    }
    else if (wb[endIndex].type > 3) {
        vv += 0.35*profitAndTime[wb[endIndex].type].first/(2-wb[endIndex].rawMaterNum());
    }
    return  - ( para2 * vv) + inf;
}

//TODO is it can merge with above function
double mcmf::countSellValue(int proType,int rtIdx,int endIndex){
    coordinate s = rt[rtIdx].location;
    coordinate e = wb[endIndex].location;
    
    double dd = dis(s, e);                     // 机器人到终点的距离
    vec s2e(e.x - s.x, e.y - s.y);          // 起点到终点的向量
    double rr = cntAngle(rt[rtIdx].lsp, s2e); // 任务所需转动角度和
    double tt = dd/6 + rr/PI;
    double vv = profitAndTime[proType].first;

    // 考虑剩余原材料格对价值的影响，目标工作台的剩余材料格越少越重视
    if (wb[endIndex].type > 7) {
        vv *= 0.8;
    }
    else if (wb[endIndex].type == 7) {
        vv += 0.35*profitAndTime[wb[endIndex].type].first/(3-wb[endIndex].rawMaterNum());
    }
    else if (wb[endIndex].type > 3) {
        vv += 0.35*profitAndTime[wb[endIndex].type].first/(2-wb[endIndex].rawMaterNum());
    }
    return - (para2 * vv) + inf;
}

double mcmf::countBuyValue(int proType,int rtIdx,int endIndex) {
    coordinate s = rt[rtIdx].location;
    coordinate e = wb[endIndex].location;
    double dd = dis(s, e);                    // 机器人到终点的距离
    vec s2e(e.x - s.x, e.y - s.y);            // 机器人到终点的向量
    double rr = cntAngle(rt[rtIdx].lsp, s2e); // 任务所需转动角度和
    double tt = dd/6 + rr/PI;
    double vv = profitAndTime[proType].first + inf * 2;

    return  para2 * vv + inf;
}


void mcmf::allocateNode(int wbIdx){
    if (!(~workbenchId[wbIdx])) {
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
            int towbIdx = ProductId2Workbench[tmp.to],toType = wb[towbIdx].type;
            if (produce2sell[type].count(toType)) {

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

    int h = 0;
    while(q.size()){
        h = max<int>(q.size(),h);
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
    while(spfa() > 0){
        int newflow = inf,index = 0;
        for(int x = T; x != S; x = pre[x])  newflow = min(newflow,G[pre[x]][pe[x]].cap);
        for(int x = T; x != S; x = pre[x])  {
            stateBuf[bufCur][index][0] = pre[x],stateBuf[bufCur][index++][1] = pe[x];
            edge&ed = G[pre[x]][pe[x]];
            ed.cap -= newflow;
            G[ed.to][ed.rev].cap += newflow;
        }
        ++bufCur;
    }
    return 1;
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
    edge&tmp = G[id][0];
    tmp.cap = 0,G[S][tmp.rev].cap = 1;

    int nodeId = rt[rtIdx].nodeId,type = rt[rtIdx].pd_id;
    for (int i = 1,size = G[id].size(); i < size; i++) {
        edge&ed = G[id][i];
        if (ed.to != nodeId) {
            ed.cost = countBuyValue(type,rtIdx,ProductId2Workbench[ed.to]);
            // if (nodeId < 0) ed.cap = 1,G[tmp.to][tmp.rev].cap = 0;
        }
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
            int towbIdx = ProductId2Workbench[tmp.to],toType = wb[towbIdx].type;
            if (produce2sell[type].count(toType)) {
                tmp.cap = 1, G[tmp.to][tmp.rev].cap = 0;
                tmp.cost = countSellValue(type,rtIdx,towbIdx);
                G[tmp.to][tmp.rev].cost = -tmp.cost;
            }
        }
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
            cerr << "destroyed " << endl;
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

void robot::checkDest() {
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

void solution() {
    // 检查工作台状态

    //TODO parallel
    for (int i = 0; i < K; ++i) {
        if (wb[i].pstatus) {
            // cerr<<i<<endl;
            curFlow.allocateNode(i);
        }
        for (int wbIdx = 0; wbIdx < K; ++wbIdx) {
            int rstatus = wb[wbIdx].rstatus;
            auto testAndSetEdgeCap = [&](int bit,int index){
                if (!((rstatus >> bit) & 1)) {
                    curFlow.setEdgeCap(curFlow.workbenchProductId[wbIdx][index],0,1);
                }
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
    }


    //TODO parallel
    // 检查机器人运动状态
    for (int rtIdx = 0; rtIdx < N; ++rtIdx) {
        if (rt[rtIdx].holdTime) --rt[rtIdx].holdTime;
        rt[rtIdx].cmd.clean(); // 清除之前指令设置
        rt[rtIdx].checkSpeed();// 保证速度非0
        rt[rtIdx].checkDest(); // 检查是否到达目的地
        curFlow.adjustEdge(rtIdx);// 调整网络
    }
    if(frameID /10 == 5)
    // for(int i = 0;i < curFlow.cnt;i++) for(auto e:curFlow.G[i]) if(e.cap>0)fprintf(stderr,"%d ---> %d\n",i,e.to);
    // 指令规划
    curFlow.solve(); // 费用流运行
    // if(frameID>48)  cerr<<"检查费用流运行"<<' '<< "final" << endl;
    auto setDest = [&](int rtIdx){
        //TODO merge this function into robot
        if (rt[rtIdx].haveTemDest) {
            rt[rtIdx].setSpeed(rt[rtIdx].temDest);
        } else if(rt[rtIdx].curTask.checkVaild()) {
            rt[rtIdx].setSpeed(rt[rtIdx].curTask.destCo);
        }
    };
    //TODO parallel
    // 检查调整机器人任务执行
    for (int rtIdx = 0; rtIdx < N; ++rtIdx) {
        curFlow.adjustTask(rtIdx);// 调整任务
        setDest(rtIdx);
    }

    // 碰撞避免
    collitionAvoidance();
    // ori_collitionAvoidance(); 

    curFlow.resetCap();
    return;
}