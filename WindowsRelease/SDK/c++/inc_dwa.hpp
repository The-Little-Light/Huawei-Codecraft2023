#ifndef DWA_HPP
#define DWA_HPP
#include "inc_codecraft2023.hpp"

extern double dv0Max;       // 携带物品时线最大加速度
extern double dv1Max;       // 不携带物品时线最大加速度
extern double da1Max;       // 携带物品时角最大加速度
extern double da0Max;       // 不携带物品时角最大加速度

extern int dwaN;            // 预测N帧
extern int dwaM;            // 速度空间采样点数
extern const double dt;     // 帧长度


double motionEvaluate(vec postion,int rtIdx,vec speed);


/**
 * @descriptin: 输入机器人编号，返回最佳预测路径对应下的朝向和速度。
*/
vec motionPredict(int rtIdx);


#endif