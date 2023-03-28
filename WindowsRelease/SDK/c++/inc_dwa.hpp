#ifndef DWA_HPP
#define DWA_HPP
#include "inc_codecraft2023.hpp"

extern double dv0Max;       // 不携带物品时线最大加速度
extern double dv1Max;       // 携带物品时线最大加速度
extern double da0Max;       // 不携带物品时角最大加速度
extern double da1Max;       // 携带物品时角最大加速度

extern int dwaN;            // 预测N帧
extern int dwaM;            // 速度空间采样点数
extern const double dt;     // 帧长度

extern double dwa_para1;    // 势能分量系数
extern double dwa_para2;    // 目标角度系数
extern double dwa_para3;    // 有效速度系数


double motionEvaluate(coordinate postion,int rtIdx,vec speed);


/**
 * @descriptin: 输入机器人编号，返回最佳预测路径对应下的(线速度,角速度)。
 * constrait:速度非零
*/
vec motionPredict(int rtIdx);


#endif