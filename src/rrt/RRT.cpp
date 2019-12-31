//
// Created by unicorn on 2019/12/28.
//
#include "rrt/RRT.h"

void RRT::setStart(Vector2f start){
    startNode=make_shared<Node>(start);
    startNode->Parent= nullptr;
}
void RRT::setStepSize(float stepsize){
    stepsize_=stepsize;
}

void RRT::setGoal(Vector2f goal){
    goalNode=make_shared<Node>(goal);
}
void RRT::setMap(OccMap *occMap){
    map=occMap;
}
bool RRT::isInitilized(){
    return map!= nullptr&& goalNode!= nullptr&& startNode!= nullptr;
}

Node::Ptr RRT::Near(Node::Ptr n){
    float mindis=INT_MAX; Node::Ptr minNode= nullptr;
    for(auto iter=sampledPoints.begin();iter!=sampledPoints.end();iter++){
        if(*iter==n) continue;
        float dis=map->distance(n,*iter);
        if(dis<mindis && dis>0.05) {
            mindis=dis;
            minNode=*iter;
        }

    }
    return minNode;
}

bool RRT::Steer(Node::Ptr n1,Node::Ptr n2,Node::Ptr &xnew){
    double costh=abs((n1->pw[0]-n2->pw[0]))/sqrt(pow(n1->pw[1]-n2->pw[1],2)+pow(n1->pw[0]-n2->pw[0],2));
    double sinth=sqrt(1-pow(costh,2));
    double xsign=1.0,ysign=1.0;
    if(n1->pw[0]>n2->pw[0]){
        xsign=-1.0;
    }
    if(n1->pw[1]>n2->pw[1]){
        ysign=-1.0;
    }
    float cx=n1->pw[0]+costh*xsign*stepsize_,cy=n1->pw[1]+sinth*ysign*stepsize_;

    if(map->isValid(cx,cy)){
        xnew=make_shared<Node>(Vector2f(cx,cy));
        return true;
    }
    else
        return false;

}
bool RRT::solve(int iter_max,ros::Publisher sampletreePub,ros::Publisher sample_point_pub){
    sampledPoints.push_back(startNode);
    Node::Ptr xrand;
    //init the tree,put start node and a vaild child in the tree
    while(1){
        xrand=map->uniform_sample_area(startNode->pw[0],startNode->pw[1],2*stepsize_,1000);
        if(map->ValidConnect(xrand,startNode)){
            xrand->Parent=startNode;
            sampledPoints.push_back(xrand);
            sampleTree.push_back(make_pair(xrand,startNode));
            break;
        }
    }

    int j=0;
    while(j<iter_max){
        //for the first time ,xrand exist
        if(j!=0){
           if(!map->uniform_sample(xrand)) continue;
        }
        shared_ptr<Node> xnear=Near(xrand);
        if(xnear== nullptr) continue;
        shared_ptr<Node> xnew;
        if(!Steer(xrand,xnear,xnew)) continue;
        if(map->ValidConnect(xnear,xnew)){
            xnew->Parent=xnear;
            sampleTree.push_back(make_pair(xnew,xnear));
            sampledPoints.push_back(xnew);
            j++;

        } else continue;
        //cout<<"traget distance : "<<map->distance(xnew,goalNode)<<endl;
        if(map->distance(xnew,goalNode)<4*stepsize_){
            int iterMax=80;
            while(iterMax){
                auto nearGoal=map->uniform_sample_area(xnew->pw[0],xnew->pw[1],4*stepsize_,80);
                if(nearGoal!= nullptr && map->ValidConnect(nearGoal,goalNode)){
                    if(map->distance(nearGoal,goalNode)<2*stepsize_){
                        nearGoal->Parent=xnew;
                        goalNode->Parent=nearGoal;
                        sampleTree.push_back(make_pair(nearGoal,goalNode));
                        cout<<"RRT sovled "<<j<<" times to find a path"<<endl;
                        return true;
                    }
                    j++;

                }
                iterMax--;
            }

        }
        Vis_sample(sampledPoints,sample_point_pub);
        Vis_sample_tree(sampleTree,sampletreePub);
    }
    cout<<"RRT couldnt find a path "<<endl;
    return false;
}

