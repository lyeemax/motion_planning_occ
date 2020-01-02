//
// Created by unicorn on 2019/12/28.
//

#include "rrt/MainRosRRT.h"

MainRosRRT::MainRosRRT(int argc,char ** argv,RRTMethod meth){
    if(meth==RRT_M){
        rrt=new RRT;
        rrt->setStepSize(0.1);
        method=RRT_M;
    } else if(meth==RRTSTAR_M){
        rrtstar=new RRTstar;
        rrtstar->setStepSize(1.0);
        method=RRTSTAR_M;
    } else if(meth==INFORMED_RRT_STAR_M){
        informedrrt=new InformedRRTstar;
        informedrrt->setStepSize(1.0);
        method=INFORMED_RRT_STAR_M;
    } else if(meth==HYBIRD_RRT_STAR_M){
        hybirdrrt=new HybirdRRTstarKdtree;
        hybirdrrt->setStepSize(0.5);
        method=HYBIRD_RRT_STAR_M;
    }
    occmap=new OccMap(0.05,0.2);
    ros::init(argc,argv,"rrt");
    nh=new ros::NodeHandle("~");
    sampler_pointsPub= nh->advertise<visualization_msgs::Marker>("/sampler_points",1);
    pathPub= nh->advertise<visualization_msgs::Marker>("/path",1);
    costmapPub=nh->advertise<visualization_msgs::Marker>("/costmap",1);
    sampletreePub=nh->advertise<visualization_msgs::MarkerArray>("/sample_tree",1);
    ellipse_pub=nh->advertise<visualization_msgs::Marker>("/ellipse",1);
    mapSub=nh->subscribe("/map",10,&MainRosRRT::mapCallBack,this);
    move_base_goalSub=nh->subscribe("/move_base_simple/goal",10,&MainRosRRT::goalCallBack,this);
    testGoals.push_back(Vector2f(27.0,18.98));
    testGoals.push_back(Vector2f(34.6,32.38));
    testGoals.push_back(Vector2f(29.58,5.09));
    testGoals.push_back(Vector2f(6.32,23.06));
    testGoals.push_back(Vector2f(27.56,52.36));
    testGoals.push_back(Vector2f(35.72,55.86));

}

void MainRosRRT::setStartPoint(Vector2f s){
    if(method==RRT_M)
        rrt->setStart(s);
    else if(method==RRTSTAR_M)
        rrtstar->setStart(s);
    else if(method==INFORMED_RRT_STAR_M)
        informedrrt->setStart(s);
    else if(method==HYBIRD_RRT_STAR_M)
        hybirdrrt->setStart(s);
}

void MainRosRRT::mapCallBack(nav_msgs::OccupancyGridConstPtr map){
    occmap->setMap(map);
    if(method==RRT_M)
        rrt->setMap(occmap);
    else if(method==RRTSTAR_M)
        rrtstar->setMap(occmap);
    else if(method==INFORMED_RRT_STAR_M)
        informedrrt->setMap(occmap);
    else if(method==HYBIRD_RRT_STAR_M)
        hybirdrrt->setMap(occmap);
    vis_costmap(occmap,costmapPub);
}

void MainRosRRT::goalCallBack(geometry_msgs::PoseStampedConstPtr ps){
    static int goalIndex=0;
//    if(goalIndex>5) goalIndex=0;
//    auto goal=testGoals[goalIndex];
    auto goal=Vector2f(ps->pose.position.x,ps->pose.position.y);
    cout<<"goal is"<<goal.transpose()<<endl;
    if(method==RRT_M){
        rrt->sampledPoints.clear();
        rrt->sampleTree.clear();
        rrt->setGoal(goal);
        if(rrt->isInitilized()){
            bool succ=rrt->solve(999999,sampletreePub,sampler_pointsPub);
            if(succ){
                vector<Vector2f> path;
                path.push_back(Vector2f(rrt->goalNode->pw[0],rrt->goalNode->pw[1]));
                while(rrt->goalNode->Parent!= nullptr){
                    rrt->goalNode=rrt->goalNode->Parent;
                    path.push_back(Vector2f(rrt->goalNode->pw[0],rrt->goalNode->pw[1]));
                }
                vis_path(path,pathPub);
            }
        }
    } else if (method==RRTSTAR_M){
        rrtstar->sampledPoints.clear();
        rrtstar->sampleTree.clear();
        rrtstar->setGoal(goal);
        if(rrtstar->isInitilized())
            rrtstar->solve(999999,sampletreePub,sampler_pointsPub,pathPub);
    }else if (method==INFORMED_RRT_STAR_M){
        informedrrt->sampledPoints.clear();
        informedrrt->sampleTree.clear();
        informedrrt->pathNode.clear();
        informedrrt->setGoal(goal);
        if(informedrrt->isInitilized())
            informedrrt->solve(999999,sampletreePub,sampler_pointsPub,pathPub,ellipse_pub);
    }else if (method==HYBIRD_RRT_STAR_M){
        hybirdrrt->sampledPointsWithFn.clear();
        hybirdrrt->sampleTree.clear();
        hybirdrrt->pathNode.clear();
        hybirdrrt->setGoal(goal);
        if(hybirdrrt->isInitilized())
            hybirdrrt->solve(999999,sampletreePub,sampler_pointsPub,pathPub,ellipse_pub);
    }
    goalIndex++;

}

void MainRosRRT::run(){

    ros::Rate rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}