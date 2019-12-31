//
// Created by unicorn on 2019/12/29.
//
#include "rrt/RRTstar.h"


void RRTstar::setStart(Vector2f start){
    startNode=make_shared<Node>(start);
    startNode->gn=0;
    startNode->Parent= nullptr;
}
void RRTstar::setStepSize(float stepsize){
    stepsize_=stepsize;
}

void RRTstar::setGoal(Vector2f goal){
    goalNode=make_shared<Node>(goal);
}
void RRTstar::setMap(OccMap *occMap){
    map=occMap;
}
bool RRTstar::isInitilized(){
    return map!= nullptr&& goalNode!= nullptr&& startNode!= nullptr;
}

Node::Ptr RRTstar::Near(Node::Ptr n){
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

bool RRTstar::Steer(Node::Ptr n1,Node::Ptr n2,Node::Ptr &xnew){
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
multimap<float,Node::Ptr> RRTstar::getNearestR(Node::Ptr cur,float radius){
    multimap<float,Node::Ptr> nearKMap;
    for (int j = 0; j <sampledPoints.size() ; ++j) {
        float dis=map->distance(sampledPoints[j],cur);
        if(dis<radius && map->ValidConnect(cur,sampledPoints[j])){
            nearKMap.insert(make_pair(dis,sampledPoints[j]));
        }
    }
    return nearKMap;

}
void RRTstar::Rewire(Node::Ptr cur,multimap<float,Node::Ptr> nearkmap){
    for(auto iter=nearkmap.begin();iter!=nearkmap.end();iter++){
        if(iter->second->gn > cur->gn+iter->first && map->ValidConnect(iter->second,cur)){
            iter->second->Parent=cur;
            iter->second->gn=cur->gn+iter->first;
            //TODO:remove old connections in sample tree
            sampleTree.push_back(make_pair(iter->second,cur));
            continue;
        } else
            continue;
    }

}
bool RRTstar::solve(int iter_max,ros::Publisher sampletreePub,ros::Publisher sample_point_pub,ros::Publisher path_pub){
    bool foundSolution=false;
    float lastAverage=0;
    int times=0;
    vector<float> length;//record path length
    sampledPoints.push_back(startNode);
    Node::Ptr xrand;
    //init the tree,put start node and a vaild child in the tree
    while(1){
        xrand=map->uniform_sample_area(startNode->pw[0],startNode->pw[1],2*stepsize_,1000);
        if(map->ValidConnect(xrand,startNode)){
            xrand->Parent=startNode;
            xrand->gn=map->distance(xrand,startNode);
            sampledPoints.push_back(xrand);
            sampleTree.push_back(make_pair(xrand,startNode));
            break;
        }
    }

    int j=0;
    while(j<iter_max){
        //cout<<"sampled "<<j<<" times sampled points is "<<sampledPoints.size()<<endl;
        //for the first time ,xrand exist
        if(j!=0){
            if(!map->uniform_sample(xrand)) continue;
        }
        shared_ptr<Node> xnear=Near(xrand);
        if(xnear== nullptr) continue;
        shared_ptr<Node> xnew;
        if(!Steer(xrand,xnear,xnew)) continue;

        auto nearestK=getNearestR(xnew,2*stepsize_);
        if(nearestK.empty())continue;
        Node::Ptr minNode;
        float minGn=INT_MAX;
        for (auto iter=nearestK.begin();iter!=nearestK.end();iter++) {
            float dis=iter->second->gn+map->distance(iter->second,xnew);
            if(dis<minGn){
                minGn=dis;
                minNode=iter->second;
            }
        }
        if(map->ValidConnect(minNode,xnew)){
            xnew->Parent=minNode;
            xnew->gn=minNode->gn+map->distance(minNode,xnew);
            sampledPoints.push_back(xnew);
            sampleTree.push_back(make_pair(xnew,minNode));
        } else continue;

        Rewire(xnew,nearestK);
        j++;
        if(!foundSolution){
            if(map->distance(xnew,goalNode)<4*stepsize_){
                int iterMax=80;
                while(iterMax){
                    auto nearGoal=map->uniform_sample_area(xnew->pw[0],xnew->pw[1],4*stepsize_,80);
                    if(nearGoal!= nullptr && map->ValidConnect(nearGoal,goalNode)&&map->ValidConnect(nearGoal,xnew)){
                        if(map->distance(nearGoal,goalNode)<2*stepsize_){
                            nearGoal->Parent=xnew;
                            nearGoal->gn=xnew->gn+map->distance(xnew,nearGoal);
                            sampledPoints.push_back(nearGoal);

                            goalNode->Parent=nearGoal;
                            goalNode->gn=nearGoal->gn+map->distance(nearGoal,goalNode);
                            sampledPoints.push_back(goalNode); //TODO
                            sampleTree.push_back(make_pair(nearGoal,goalNode));

                            //show path
                            {
                                vector<Vector2f> path;
                                path.push_back(Vector2f(goalNode->pw[0],goalNode->pw[1]));
                                Node::Ptr temp=goalNode->Parent;
                                lastAverage+=map->distance(temp,goalNode);
                                while(temp!= nullptr){
                                    if(temp->Parent)lastAverage+=map->distance(temp,temp->Parent);
                                    length.push_back(lastAverage);
                                    path.push_back(Vector2f(temp->pw[0],temp->pw[1]));
                                    temp=temp->Parent;
                                }

                                vis_path(path,path_pub);
                            }

                            foundSolution= true;

                            cout<<"RRT sovled "<<j<<" times to find a path,staring optimization"<<endl;
                            break;
                        }
                        j++;

                    }
                    iterMax--;
                }

            }
        }
        if(foundSolution){
            vector<Vector2f> path;
            path.push_back(Vector2f(goalNode->pw[0],goalNode->pw[1]));
            Node::Ptr temp=goalNode->Parent;
            float pathLength=0;
            pathLength+=map->distance(temp,goalNode);

            while(temp!= nullptr){
                if(temp->Parent)pathLength+=map->distance(temp,temp->Parent);
                path.push_back(Vector2f(temp->pw[0],temp->pw[1]));
                temp=temp->Parent;
            }

            length.push_back(pathLength);
            vis_path(path,path_pub);

        }


        if(foundSolution && length.size()>10){
            float currenAverage=0;
            for (auto iter=length.end()-1;iter!=length.end()-11;iter--) {
                currenAverage+=(*iter);
            }
            currenAverage=currenAverage/10.0;
            //cout<<"ave minus "<<(lastAverage-currenAverage)<<"lasteave "<<lastAverage<<" curr ave "<<currenAverage<<endl;
            if((abs(lastAverage-currenAverage))<0.01){
                times++;
            } else times--;
            lastAverage=currenAverage;
        }

        if(times>800){
            cout<<"optimization done!"<<endl;
            return true;
        }

        Vis_sample(sampledPoints,sample_point_pub);
        Vis_sample_tree(sampleTree,sampletreePub);
    }
    cout<<"RRT couldnt find a path "<<endl;
    return false;
}

