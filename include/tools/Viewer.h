//
// Created by unicorn on 2019/12/27.
//
#ifndef MOTION_PLAN_OCC_VIEWER_H
#define MOTION_PLAN_OCC_VIEWER_H
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include "Eigen/Geometry"
#include <array>
#include <random>
#include <math.h>
#include "map/OccMap.h"
extern void Vis_sample(vector<Node::Ptr> points , ros::Publisher sampler_pub);

extern void Vis_sample_tree(vector<pair<Node::Ptr,Node::Ptr>> trees , ros::Publisher sampler_tree_pub);

extern void vis_costmap(OccMap *occmap , ros::Publisher costmap_pub);

extern void vis_path(vector<Vector2f> path , ros::Publisher path_pub);

extern void vis_coverage(list<Vector2f> path,OccMap *occmap,ros::Publisher cover_pub);
extern void vis_testmap(vector<Vector2f> testmap , ros::Publisher test_pub);
extern void vis_Xrand(multimap<float,Node::Ptr> coverlist,OccMap *occmap,ros::Publisher cover_pub);

extern void vis_ellipse(Node::Ptr n1,Node::Ptr n2,float focal, float pathlength,ros::Publisher ellipse_pub);

#endif //MOTION_PLAN_OCC_VIEWER_H
