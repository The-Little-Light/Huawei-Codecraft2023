/*
地图数据（可参考 maps/*.txt）是一个 100 行 100 列的字符矩阵。 地图大小为 50 米*50
米。 从此得知， 每个字符对应地图的 0.5 米*0.5 米区域。地图数据中每个字符含义如
下：
‘.’ ： 空地。
‘A’ ： 机器人起始位置，总共 4 个。
数字 1-9：工作台 1-9，总个数<=50，注意同一类工作台可能多次出现。
其余字符均为不合法字符，不会出现在地图数据中。
当地图上标记某个位置为机器人或者工作台时，则他们的坐标是该区域的中心坐标。
地图第一行对应地图的最上方，最后一行对应地图的最下方,因此第一行第一列的中心
坐标为： (0.25,49.75)。
判题器对地图数据按输入顺序逐行扫描，对遇到的机器人、工作台进行依次编号，选
手可以在初始化期间即获取所有机器人、工作台的编号与位置信息。

约束：
 由于一个格子大小容纳不下机器人，为避免区域重叠，保证机器人初始位置不会
靠墙、相互不会相邻。
 工作台个数<=50 个。
*/

#include <bits/stdc++.h>
using namespace std;
#define fo(x,y,z) for(int x = y; x < z; x++)
int mp[120][120];
int main(int argc, char *argv[]){
    // int seed = time(NULL);
    // stringstream ss;
    // if(argc > 1){
    //     ss.clear();
    //     ss<<argv[1];
    //     ss>>seed;
    // }
    random_device seed;
    mt19937 RA(seed());
    int n = (RA()%50) + 1;
    while(n){
        int i = RA()%100,j = RA()%100;
        if(!mp[i][j]) mp[i][j] = (RA()%9) + 1,n--;
    }
    n = 4;
    while(n){
        int i = (RA()%98)+1,j = (RA()%98)+1,flag = 0;
        fo(z,-1,2) fo(x,-1,2) flag|=(mp[i+z][j+x]==10);
        if(!flag) mp[i][j] = 10,n--;
    }
    fo(i,0,100){
        fo(j,0,100){
            if(!mp[i][j]) putchar('.');
            else if(mp[i][j] == 10) putchar('A');
            else putchar(mp[i][j] + '0');
        }
        puts("");
    }
    return 0;
}