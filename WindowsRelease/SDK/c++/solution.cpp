#include "solution.hpp"
using namespace std;

void solution() {
    robot& tmp = rt[0];
    if(tmp.wb_id = tmp.taskQueue.front().destId) {
        while(1){
            int next = rand()%K;
            if(next != tmp.wb_id) {
                tmp.taskQueue.front().destId = next;
                break;
            }
        }
    }
    int next = tmp.taskQueue.front().destId;
    tmp.setSpeed(wb[next].location);
    return;
}