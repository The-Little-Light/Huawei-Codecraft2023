/*** 
 * @Author: Xzh
 * @Date: 2023-03-12 22:50:56
 * @LastEditTime: 2023-03-12 22:51:44
 * @LastEditors: Xzh
 * @Description: 
 */

#include "solution.hpp"


double PI = acos(-1);

void robot::setSpeed(coordinate dest){
    double dx = dest.x - location.x;
    double dy = dest.y - location.y;
    double dist = sqrt(dx * dx + dy * dy);
    double angle = atan2(dy, dx);
    double angleDiff = angle - toward;
    // angleDiff in [-PI, PI]
    if (angleDiff > PI) angleDiff -= 2 * PI;
    if (angleDiff < -PI) angleDiff += 2 * PI;

    cmd.rotate = angleDiff;
    cmd.forward = dist * cos(angleDiff);
}