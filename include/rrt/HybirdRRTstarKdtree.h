//
// Created by unicorn on 2020/1/2.
//

#ifndef MOTION_PLAN_OCC_HYBIRDRRTSTARKDTREE_H
#define MOTION_PLAN_OCC_HYBIRDRRTSTARKDTREE_H

#include "pcl/kdtree/kdtree_flann.h"
#include <pcl/point_cloud.h>
#include "map/OccMap.h"
#include "tools/Viewer.h"
#include <map>
class HybirdRRTstarKdtree{
public:
    shared_ptr<Node> startNode;
    shared_ptr<Node> goalNode;
    OccMap *map;
    //sampledPointsWithFn storage sample point Node with ordered cost
    multimap<float,Node::Ptr> sampledPointsWithFn;
    //sampleTreeCloud use to get K nearest points
    pcl::PointCloud<pcl::PointXY>::Ptr sampleTreeCloud;
    //searchChart used to find a give K nearest points index to get its Node pointer in sampledPointsWithFn
    std::map<int,multimap<float,Node::Ptr>::iterator> searchChart;
    pcl::KdTreeFLANN<pcl::PointXY> kdtree;

    vector<Node::Ptr> pathNode;
    vector<pair<Node::Ptr,Node::Ptr>> sampleTree;

private:
    float stepsize_; //rrt step for new point generation
    float minDis;//min distance between start and goal;

public:
    HybirdRRTstarKdtree();
    void setStart(Vector2f start);
    void setStepSize(float stepsize);
    void setGoal(Vector2f goal);
    void setMap(OccMap *occMap);
    bool isInitilized();
    Node::Ptr Near(Node::Ptr n);
    float getHeuristic(Node::Ptr);
    Node::Ptr sampleWithLowDensity(Node::Ptr curr,Node::Ptr goal,float length);
    multimap<float,Node::Ptr> ellipse_sample_star(Node::Ptr curr,Node::Ptr goal,float min_step);
    //return the nearest k neighbor and their distance
    multimap<float,Node::Ptr> getNearestR(Node::Ptr cur,float radius);
    bool Steer(Node::Ptr n1,Node::Ptr n2,Node::Ptr &xnew);
    bool solve(int iter_max,ros::Publisher sampletreePub,ros::Publisher sample_point_pub,ros::Publisher path_pub,ros::Publisher ellipse_pub);
    void Rewire(Node::Ptr cur,multimap<float,Node::Ptr> nearkmap);
};
#endif //MOTION_PLAN_OCC_HYBIRDRRTSTARKDTREE_H
