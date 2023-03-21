/*** 
 * @Author: Xzx
 * @Date: 2023-03-21
 * @Description: 
 *      自用数学库的实现
 ***/
#include "solution.hpp"

const double PI = acos(-1);

// 向量模
double modulusOfVector(vec& a) {
    return sqrt(a.x*a.x + a.y*a.y);
}

// 向量叉积
double crossProduct(vec& a, vec& b) {
    return a.x * b.y - a.y * b.x;
}

// 向量点乘
double dotProduct(vec& a, vec& b) {
    return a.x * b.x + a.y * b.y;
}

// 两点距离
double dis(coordinate& c1, coordinate& c2) {
    double ans = (c1.x - c2.x)*(c1.x - c2.x) + (c1.y - c2.y)*(c1.y - c2.y);
    return sqrt(ans);
}

// 计算两个向量的夹角
double cntAngle(vec& a, vec& b){
    double angleDiff = acos(dotProduct(a, b) / (modulusOfVector(a)*modulusOfVector(b)));
    return (angleDiff);
}
