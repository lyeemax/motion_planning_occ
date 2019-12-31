//
// Created by unicorn on 2019/12/27.
//
#include "map/OccMap.h"
#include "rrt/MainRosRRT.h"
int main(int argc,char **argv){
    Vector2f start={18.6,37.09};
    MainRosRRT RRTNode(argc,argv,HYBIRD_RRT_STAR_M);
    RRTNode.setStartPoint(start);
    RRTNode.run();
}
