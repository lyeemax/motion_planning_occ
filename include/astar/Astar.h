//
// Created by unicorn on 2019/12/28.
//

#ifndef MOTION_PLAN_OCC_ASTAR_H
#define MOTION_PLAN_OCC_ASTAR_H

#include "map/OccMap.h"
class Astar{
private:
    float step_length;
    array<int,8> rules;
    array<int,8> rulesDig;
    multimap<double,Node::Ptr> openSet;
    OccMap *map;
    Vector2f start;  //map is set after set start Node,so start Node must create in map callback
    int mx;
    int my;
    GraphSerachMethod method;
public:
    Node::Ptr startNode;
    Node::Ptr goalNode;
    vector<vector<Node::Ptr>> costmap;
    list<Vector2f> vistedArea;
    int radius;//pixel radius,regard radius(number) as one node for fast search
public:
    Astar(int r,GraphSerachMethod meth);
    void InitMap(OccMap *m,ros::Publisher testmapPub);
    void setStart(Vector2f s){start=s;};
    void InitStart();
    void setGoal(Vector2f goal);
    float getHeuristic(Node::Ptr p1,Node::Ptr p2);
    //first is non-Dig neighbor second is Dig
    pair<vector<Node::Ptr>,vector<Node::Ptr>> getNeighbor(Node::Ptr p);
    bool solve(ros::Publisher coverageAreaPub);
    void resetMap();
};
#endif //MOTION_PLAN_OCC_ASTAR_H
