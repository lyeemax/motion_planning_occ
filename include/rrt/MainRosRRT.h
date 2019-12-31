//
// Created by unicorn on 2019/12/27.
//

#ifndef MOTION_PLAN_OCC_MAINROSRRT_H
#define MOTION_PLAN_OCC_MAINROSRRT_H
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <array>
#include <random>
#include <math.h>
#include "map/OccMap.h"
#include "rrt/RRT.h"
#include "rrt/InformedRRTstar.h"
#include "rrt/HybirdRRTstar.h"
#include "rrt/RRTstar.h"
#include "tools/Viewer.h"
using  namespace std;
using  namespace Eigen;


class MainRosRRT{
public:
    ros::Publisher sampler_pointsPub,pathPub,costmapPub,sampletreePub,ellipse_pub;
    ros::Subscriber mapSub,move_base_goalSub;
    ros::NodeHandle *nh;
    OccMap * occmap;
    RRT *rrt;
    RRTstar *rrtstar;
    InformedRRTstar *informedrrt;
    HybirdRRTstar *hybirdrrt;

    vector<Vector2f> testGoals;

    RRTMethod method;

    MainRosRRT(int argc,char ** argv,RRTMethod meth);

    void setStartPoint(Vector2f s);

    void mapCallBack(nav_msgs::OccupancyGridConstPtr map);

    void goalCallBack(geometry_msgs::PoseStampedConstPtr ps);

    void run();

};

#endif //MOTION_PLAN_OCC_MAINROSRRT_H
