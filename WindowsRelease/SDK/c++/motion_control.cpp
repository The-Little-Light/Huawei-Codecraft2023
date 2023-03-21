/*** 
 * @Author: Xzh
 * @Date: 2023-03-12 22:50:56
 * @LastEditTime: 2023-03-17 01:10
 * @LastEditors: Zym
 * @Description: 
 *      优化路径规划，并基本保证行星运动避免
 *      添加对墙壁的碰撞避免
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
    // Limit the angular velocity
    if (absAngleDiff * 15 > PI) {
        cmd.rotate = sign * PI;
    }
    else {
        cmd.rotate = sign * 15 * absAngleDiff;
    }
    // Limit the velocity according to the angle and distance
    if (absAngleDiff * 2 > PI) {
        minLSpeed = 2 * cos(absAngleDiff);
    }
    else {
        minLSpeed = 6 * cos(absAngleDiff); 
    }
    // 对撞墙进行特判
    if (taskQueue.size() == 1) {
        coordinate detectPoint;                 // 探测点
        detectPoint.set(location.x + 0.02 * lsp.x, location.y + 0.02 * lsp.y);
        if ((detectPoint.x <= 0.77 && lsp.x < 0) || (detectPoint.x >= 50 - 0.77 && lsp.x > 0)) {
            if (minLSpeed > 0) minLSpeed = 0.3 * minLSpeed;
        }
        else if ((detectPoint.y <= 0.77 && lsp.y < 0) || (detectPoint.y >= 50 - 0.77 && lsp.y > 0)) {
            if (minLSpeed > 0) minLSpeed = 0.3 * minLSpeed;
        }
    }
    cmd.forward = minLSpeed;
    // if (absAngleDiff * 7 > PI && dist < 1.2) {
    //     cmd.forward = min(minLSpeed, 0.1*dist*dist + 1);
    // }
    // else cmd.forward = min(minLSpeed, 0.1*dist*dist + 5);
}