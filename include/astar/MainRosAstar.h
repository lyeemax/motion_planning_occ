//
// Created by unicorn on 2019/12/28.
//

#ifndef MOTION_PLAN_OCC_MAINROSASTAR_H
#define MOTION_PLAN_OCC_MAINROSASTAR_H
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
#include "astar/Astar.h"
#include "tools/Viewer.h"
using  namespace std;
using  namespace Eigen;

class MainRosAstar{
public:
    ros::Publisher coverageAreaPub,pathPub,costmapPub,testmapPub;
    ros::Subscriber mapSub,move_base_goalSub;
    ros::NodeHandle *nh;
    OccMap * occmap;
    Astar *astar;

    MainRosAstar(int argc,char ** argv);

    void setStartPoint(Vector2f s);

    void mapCallBack(nav_msgs::OccupancyGridConstPtr map);

    void goalCallBack(geometry_msgs::PoseStampedConstPtr ps);

    void run();

};
#endif //MOTION_PLAN_OCC_MAINROSASTAR_H

