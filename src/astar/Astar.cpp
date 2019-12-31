//
// Created by unicorn on 2019/12/28.
//

#include "astar/Astar.h"
#include "tools/Viewer.h"
Astar::Astar(int r,GraphSerachMethod meth){
    rules={0,0,1,-1,1,-1,0,0};
    rulesDig={1,1,-1,-1,1,-1,1,-1};
    radius=r;
    method=meth;
}
void Astar::InitMap(OccMap *m,ros::Publisher testmapPub){
    map=m;
    mx=floor(map->getWidth()/radius);
    my=floor(map->getHeight()/radius);
    vector<Vector2f> testmap;
    step_length=radius*map->getRes();
    costmap.resize(my);
    for (int y = 0; y < my; ++y) {
        costmap[y].resize(mx);
        for (int x = 0; x <mx ; ++x) {
            Vector2f p=map->inx2coor(Vector2i(x*radius,y*radius));
            costmap[y][x]=make_shared<Node>(p,!map->isValidPoint(p,radius),x,y);
        }
    }
//    int count=0;
//    for (int j = 0; j <1000000 ; ++j) {
//        std::uniform_real_distribution<double > distx{map->minx_,map->maxx_};
//        std::uniform_real_distribution<double > disty{map->miny_,map->maxy_};
//        std::random_device rd;
//        std::default_random_engine rng {rd()};
//        double x=distx (rng);
//        double y=disty(rng);
//        Vector2i Pixel=map->coor2XYPixel(Vector2f(x,y));
//        auto cmIdx=Pixel/radius;
//        auto Node=costmap[cmIdx[1]][cmIdx[0]];
//        cout<<" test "<<(Vector2f(x,y)-Node->pw).transpose()<<endl;
//        if((!map->isVaild(x,y))==Node->isObstacle) count++;
//    }
//    cout<<float(count)/1000000.0<<" passed"<<endl;

}
void Astar::InitStart(){
    Vector2i startPixel=map->coor2XYPixel(start);
    auto cmIdx=startPixel/radius;
    startNode=costmap[cmIdx[1]][cmIdx[0]];
    assert(!startNode->isObstacle);
    startNode->gn=0;
    startNode->Parent= nullptr;
    //for now, we have no information for goal,so we init the state in setGoalNode
}
void Astar::resetMap() {
    for (int j = 0; j <costmap.size() ; ++j) {
        for (int k = 0; k <costmap[j].size() ; ++k) {
            costmap[j][k]->Parent= nullptr;
            costmap[j][k]->Visted= false;
            costmap[j][k]->gn=INT_MAX;
        }
    }
    goalNode= nullptr;
    startNode->gn=0;
}
void Astar::setGoal(Vector2f goal){
    Vector2i goalPixel=map->coor2XYPixel(goal);
    auto cmIdx=goalPixel/radius;
    goalNode=costmap[cmIdx[1]][cmIdx[0]];

}
float Astar::getHeuristic(Node::Ptr p1,Node::Ptr p2){
    if(method==ASTAR_M){
        return map->distance(p1,p2);
    }
    if(method==DIJISTRA_M){
        return 0;
    }
}
//first is non-Dig neighbor second is Dig
pair<vector<Node::Ptr>,vector<Node::Ptr>> Astar::getNeighbor(Node::Ptr p){
    vector<Node::Ptr> neighbors;
    for (int j = 0; j <rules.size()/2 ; ++j) {
        Vector2i neighborPixelIdx=Vector2i(p->x,p->y)+Vector2i(rules[j],(rules[j+4]));
        if(neighborPixelIdx[0]>=mx ||neighborPixelIdx[0]<0 ||neighborPixelIdx[1]>=my || neighborPixelIdx[1]<0) continue;
        Node::Ptr neighbor=costmap[neighborPixelIdx.y()][neighborPixelIdx.x()];
        if(!neighbor->isObstacle){
            neighbors.push_back(neighbor);
            vistedArea.push_back(neighbor->pw);
        }
    }

    vector<Node::Ptr> Digneighbors;
    for (int j = 0; j <rulesDig.size()/2 ; ++j) {
        Vector2i neighborPixelIdx=Vector2i(p->x,p->y)+Vector2i(rulesDig[j],(rulesDig[j+4]));
        if(neighborPixelIdx[0]>=mx ||neighborPixelIdx[0]<0 ||neighborPixelIdx[1]>=my || neighborPixelIdx[1]<0) continue;
        Node::Ptr neighbor=costmap[neighborPixelIdx.y()][neighborPixelIdx.x()];
        if(!neighbor->isObstacle){
            neighbors.push_back(neighbor);
            vistedArea.push_back(neighbor->pw);
        }
    }
    return make_pair(neighbors,Digneighbors);
}
bool Astar::solve(ros::Publisher coverageAreaPub){
    openSet.clear();
    openSet.insert(make_pair(map->distance(startNode,goalNode),startNode));
    while(!openSet.empty()){
        auto node=openSet.begin()->second;
        node->Visted=true;
        openSet.erase(openSet.begin());
        if(node==goalNode){
            cout<<"path find!"<<endl;
            return true;
        }
        auto neighbors=getNeighbor(node);
        vis_coverage(vistedArea,map,coverageAreaPub);
        for (int j = 0; j <neighbors.first.size() ; ++j) {
            auto mNode=neighbors.first[j];
            if(mNode->gn==INT_MAX && !mNode->Visted){
                mNode->gn=node->gn+step_length;
                mNode->Parent=node;
                float h=mNode->gn+getHeuristic(mNode,goalNode);
                openSet.insert(make_pair(h,mNode));
                continue;
            } else if (node->gn+step_length<mNode->gn &&!mNode->Visted&& mNode->gn<INT_MAX){
                mNode->gn=node->gn+step_length;
                mNode->Parent=node;
                continue;
            }
        }
        for (int j = 0; j <neighbors.second.size() ; ++j) {
            auto mNode=neighbors.second[j];
            if(mNode->gn==INT_MAX&&!mNode->Visted){
                mNode->gn=node->gn+step_length*1.414;
                mNode->Parent=node;
                float h=mNode->gn+getHeuristic(mNode,goalNode);
                openSet.insert(make_pair(h,mNode));
                continue;
            } else if (node->gn+1.414*step_length<mNode->gn &&!mNode->Visted&& mNode->gn<INT_MAX){
                mNode->gn=node->gn+step_length*1.141;
                mNode->Parent=node;
                continue;
            }
        }

    }
    cout<<"cannot find a path"<<endl;
    return false;
}
