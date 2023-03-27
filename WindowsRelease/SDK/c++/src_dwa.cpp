#include "inc_codecraft2023.hpp"



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