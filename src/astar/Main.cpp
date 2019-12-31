//
// Created by unicorn on 2019/12/28.
//

#include "map/OccMap.h"
#include "astar/MainRosAstar.h"
int main(int argc,char **argv){
    Vector2f start={18.6,37.09};
    MainRosAstar AstarNode(argc,argv);
    AstarNode.setStartPoint(start);
    AstarNode.run();
}
