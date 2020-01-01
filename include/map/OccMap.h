//
// Created by unicorn on 2019/12/27.
//

#ifndef MOTION_PLAN_OCC_OCCMAP_H
#define MOTION_PLAN_OCC_OCCMAP_H
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Core>
#include <array>
#include <random>
#include <math.h>
#include "Eigen/Geometry"
#include <chrono>
using namespace std;
using namespace Eigen;

enum RRTMethod{
    RRT_M,
    RRTSTAR_M,
    INFORMED_RRT_STAR_M,
    HYBIRD_RRT_STAR_M
};

enum GraphSerachMethod{
    ASTAR_M,
    DIJISTRA_M,
    DYNAMIC_ASTAR_M
};

struct Node{
    typedef shared_ptr<Node> Ptr;
    Vector2f pw; //for sample based algorithm
    float gn; //cost function
    Node::Ptr Parent;
    int x;//index of map used in graph search based algorithm
    int y;//index of map used in graph search based algorithm
    bool isObstacle;
    bool Visted;
    float gc;//growth cost
    bool ToBeGoal;
    Node(Vector2f p,bool isObs= true,int ix=-1,int iy=-1,float g=0,bool tobegoal= false){
        pw=p;
        Parent= nullptr;
        gn=INT_MAX;
        isObstacle=isObs;
        x=ix;
        y=iy;
        Visted= false;
        gc=g;
        ToBeGoal=tobegoal;
    }
};

class OccMap{
private:
    uint32_t height_;
    uint32_t width_;
    double res_;
    double orix_;
    double oriy_;
    double step_;//resolution of line,use for evaluating connection
public:
    vector<long int> costmap;
    nav_msgs::OccupancyGrid gridmap;
    vector<Vector2i> rules; //use for smooth
    double minx_;
    double maxx_;
    double miny_;
    double inflation_radius_;
    double maxy_;
public:
    OccMap(double step,double r):step_(step),inflation_radius_(r){
        Vector2i a,b,c,d;
        a={0,1};b={0,-1};c={1,0};d={-1,0};
        rules.push_back(a);rules.push_back(b);rules.push_back(c);rules.push_back(d);
    };
    float getRes(){return res_;};
    void setMap(nav_msgs::OccupancyGridConstPtr map);
    pair<double,double> inx2coor(int index);
    Vector2f inx2coor(Vector2i index);
    long int coor2inx(double cx, double cy);
    Vector2i coor2XYPixel(Vector2f Pw);
    long int XYPixel2index(Vector2i p);
    bool isValid(double cx,double cy);
    bool isValid(int index);
    bool isValidPoint(Vector2f p,int r); //p is world coordinate ,r is pixel size
    bool isValid(Vector2i pixel);
    bool isValid(double Acx,double Acy,double Bcx,double Bcy);
    bool ValidConnect(Node::Ptr p1,Node::Ptr p2);
    bool ValidConnect(Node::Ptr p1,Node::Ptr p2,Node::Ptr &pnew);
    bool uniform_sample(Node::Ptr & result);
    Node::Ptr uniform_sample_area(double originx, double originy,double r,int times);
    Node::Ptr ellipse_sample(Node::Ptr n1,Node::Ptr n2,float pathlength);
    multimap<float,Node::Ptr> ellipse_sample_star(Node::Ptr curr,Node::Ptr goal,float min_step); //return fn with nodes
    float distance(Node::Ptr n1,Node::Ptr n2);
    double getMaxx(){ return maxx_;}
    double getMaxy(){return maxy_;}
    double getMinx(){return minx_;}
    double getMiny(){return miny_;}
    int getWidth(){return width_;}
    int getHeight(){return height_;}

};
#endif //MOTION_PLAN_OCC_OCCMAP_H
