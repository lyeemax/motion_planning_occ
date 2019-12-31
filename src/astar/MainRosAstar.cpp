//
// Created by unicorn on 2019/12/28.
//

#include "astar/MainRosAstar.h"

MainRosAstar::MainRosAstar(int argc,char ** argv){
    astar=new Astar(20,ASTAR_M);
    occmap=new OccMap(0.05,0.2);
    ros::init(argc,argv,"astar");
    nh=new ros::NodeHandle("~");
    coverageAreaPub= nh->advertise<visualization_msgs::Marker>("/coverageArea",1);
    pathPub= nh->advertise<visualization_msgs::Marker>("/path",1);
    costmapPub=nh->advertise<visualization_msgs::Marker>("/costmap",1);
    testmapPub=nh->advertise<visualization_msgs::Marker>("/testmap",1);
    mapSub=nh->subscribe("/map",10,&MainRosAstar::mapCallBack,this);
    move_base_goalSub=nh->subscribe("/move_base_simple/goal",10,&MainRosAstar::goalCallBack,this);

}

void MainRosAstar::setStartPoint(Vector2f s){
    astar->setStart(s);
}

void MainRosAstar::mapCallBack(nav_msgs::OccupancyGridConstPtr map){
    occmap->setMap(map);
    astar->InitMap(occmap,testmapPub);
    astar->InitStart();
    vis_costmap(occmap,costmapPub);
}

void MainRosAstar::goalCallBack(geometry_msgs::PoseStampedConstPtr ps){
    cout<<"goal is"<<Vector2f(ps->pose.position.x,ps->pose.position.y).transpose()<<endl;
    astar->vistedArea.clear();
    astar->setGoal(Vector2f(ps->pose.position.x,ps->pose.position.y));
    if(astar->goalNode->isObstacle){
        cout<<"target is in Obstacle!"<<endl;
        return ;
    }
    bool succ=astar->solve(coverageAreaPub);
    if (succ) {
        vector<Vector2f> path;
        path.push_back(Vector2f(astar->goalNode->pw[0] , astar->goalNode->pw[1]));
        auto temp=astar->goalNode;
        while (temp->Parent != nullptr) {
            temp= temp->Parent;
            path.push_back(Vector2f(temp->pw[0] , temp->pw[1]));
        }
        vis_path(path , pathPub);
    }
    astar->resetMap();

}

void MainRosAstar::run(){

    ros::Rate rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}
