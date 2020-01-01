//
// Created by unicorn on 2019/12/30.
//

#ifndef MOTION_PLAN_OCC_HYBIRDRRTSTAR_H
#define MOTION_PLAN_OCC_HYBIRDRRTSTAR_H
#include "map/OccMap.h"
#include "tools/Viewer.h"
class HybirdRRTstar{
public:
    shared_ptr<Node> startNode;
    shared_ptr<Node> goalNode;
    OccMap *map;
    multimap<float,Node::Ptr> sampledPointsWithFn;
    vector<Node::Ptr> pathNode;
    vector<pair<Node::Ptr,Node::Ptr>> sampleTree;

private:
    float stepsize_; //rrt step for new point generation
    float minDis;//min distance between start and goal;

public:
    void setStart(Vector2f start);
    void setStepSize(float stepsize);
    void setGoal(Vector2f goal);
    void setMap(OccMap *occMap);
    bool isInitilized();
    Node::Ptr Near(Node::Ptr n);
    float getHeuristic(Node::Ptr);
    Node::Ptr sampleWithLowDensity(Node::Ptr curr,Node::Ptr goal,float length);
    //return the nearest k neighbor and their distance
    multimap<float,Node::Ptr> getNearestR(Node::Ptr cur,float radius);
    bool Steer(Node::Ptr n1,Node::Ptr n2,Node::Ptr &xnew);
    bool solve(int iter_max,ros::Publisher sampletreePub,ros::Publisher sample_point_pub,ros::Publisher path_pub,ros::Publisher ellipse_pub);
    void Rewire(Node::Ptr cur,multimap<float,Node::Ptr> nearkmap);
};

#endif //MOTION_PLAN_OCC_HYBIRDRRTSTAR_H
