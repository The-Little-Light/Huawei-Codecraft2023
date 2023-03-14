/*** 
 * @Author: Xzh
 * @Date: 2023-03-12 22:50:56
 * @LastEditTime: 2023-03-13 09:59:07
 * @LastEditors: Xzh
 * @Description: 
 ***/
#include "solution.hpp"

double PI = acos(-1);

void robot::setSpeed(coordinate dest){
    double dx = dest.x - location.x;
    double dy = dest.y - location.y;
    double dist = sqrt(dx * dx + dy * dy);
    double angle = atan2(dy, dx);
    double angleDiff = angle - toward;
    double sign = 0;
    double absAngleDiff = 0;
    double minLSpeed = 6.0;
    // angleDiff in [-PI, PI]
    if (angleDiff > PI) angleDiff -= 2 * PI;
    if (angleDiff < -PI) angleDiff += 2 * PI;
    if (angleDiff > 0) sign = 1.0;
    else if (angleDiff < 0) sign = -1.0;
    absAngleDiff = sign*angleDiff;
    // Limit the linear speed
    if (absAngleDiff * 2 > PI) {
        minLSpeed = 0;
        cmd.rotate = sign * min(absAngleDiff*4, PI);   // 调参处
    }
    else if (absAngleDiff * 4 > PI) {
        minLSpeed = 2;
        cmd.rotate = sign * min(absAngleDiff*3, PI);   // 调参处
    }
    else {
        cmd.rotate = sign * min(absAngleDiff*2, PI);   // 调参处
    }
    cmd.forward = min(minLSpeed, 0.1*dist*dist + 4);// 调参处
}