//
// Created by unicorn on 2019/12/27.
//

#ifndef MOTION_PLAN_OCC_RRT_H
#define MOTION_PLAN_OCC_RRT_H
#include "map/OccMap.h"
#include "tools/Viewer.h"

class RRT{
public:
    shared_ptr<Node> startNode;
    shared_ptr<Node> goalNode;
    OccMap *map;
    vector<Node::Ptr> sampledPoints;
    vector<pair<Node::Ptr,Node::Ptr>> sampleTree;
private:
    float stepsize_; //rrt step for new point generation
public:
    void setStart(Vector2f start);
    void setStepSize(float stepsize);
    void setGoal(Vector2f goal);
    void setMap(OccMap *occMap);
    bool isInitilized();
    Node::Ptr Near(Node::Ptr n);
    bool Steer(Node::Ptr n1,Node::Ptr n2,Node::Ptr &xnew);
    bool solve(int iter_max,ros::Publisher sampletreePub,ros::Publisher sample_point_pub);
};

#endif //MOTION_PLAN_OCC_RRT_H
