//
// Created by unicorn on 2019/12/30.
//

#include "rrt/HybirdRRTstar.h"

void HybirdRRTstar::setStart(Vector2f start){
    startNode=make_shared<Node>(start);
    startNode->gn=0;
    startNode->Parent= nullptr;
}
void HybirdRRTstar::setStepSize(float stepsize){
    stepsize_=stepsize;
}

void HybirdRRTstar::setGoal(Vector2f goal){
    goalNode=make_shared<Node>(goal);
    minDis=map->distance(startNode,goalNode);
}
void HybirdRRTstar::setMap(OccMap *occMap){
    map=occMap;
}
bool HybirdRRTstar::isInitilized(){
    return map!= nullptr&& goalNode!= nullptr&& startNode!= nullptr;
}

Node::Ptr HybirdRRTstar::Near(Node::Ptr n){
    float mindis=INT_MAX; Node::Ptr minNode= nullptr;
    for(auto iter=sampledPointsWithFn.begin();iter!=sampledPointsWithFn.end();iter++){
        if(iter->second==n) continue;
        float dis=map->distance(n,iter->second);
        if(dis<mindis && dis>0.05 ) {
            mindis=dis;
            minNode=iter->second;
        }

    }
    return minNode;
}
float HybirdRRTstar::getGC(Node::Ptr p,bool foundSolv){
    if(foundSolv) return 0;
    int count=0;
    float minx=INT_MAX,miny=INT_MAX,maxx=-INT_MAX,maxy=-INT_MAX;
    for (auto iter=sampledPointsWithFn.begin();iter!=sampledPointsWithFn.end();iter++) {
        if(map->distance(p,iter->second)<2*stepsize_){
            count++;
            maxx=iter->second->pw[0]>maxx?iter->second->pw[0]:maxx;
            maxy=iter->second->pw[1]>maxy?iter->second->pw[1]:maxy;
            minx=iter->second->pw[0]<minx?iter->second->pw[0]:minx;
            miny=iter->second->pw[0]<miny?iter->second->pw[1]:miny;
        }
    }
    float area=(maxx-minx)*(maxy-miny);
    if(count*(stepsize_*stepsize_)/area<=2) return -stepsize_;
    return 4*count/area*stepsize_;

}
bool HybirdRRTstar::Steer(Node::Ptr n1,Node::Ptr n2,Node::Ptr &xnew){
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
multimap<float,Node::Ptr> HybirdRRTstar::getNearestR(Node::Ptr cur,float radius){
    multimap<float,Node::Ptr> nearKMap;
    for (auto iter=sampledPointsWithFn.begin();iter!=sampledPointsWithFn.end();iter++) {
        float dis=map->distance(iter->second,cur);
        if(dis<radius && map->ValidConnect(cur,iter->second)){
            nearKMap.insert(make_pair(dis,iter->second));
        }
    }
    return nearKMap;

}
void HybirdRRTstar::Rewire(Node::Ptr cur,multimap<float,Node::Ptr> nearkmap){
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

bool HybirdRRTstar::solve(int iter_max,ros::Publisher sampletreePub,ros::Publisher sample_point_pub,ros::Publisher path_pub,ros::Publisher ellipse_pub){
    auto start=chrono::steady_clock::now();
    auto findSolutionTime=start;
    auto OptSolved=start;

    bool foundSolution=false;
    float lastAverage=0;
    float currenAverage=INT_MAX;//path length in average
    int N=1;
    int times=0;
    vector<float> length;//record path length

    startNode->Visted=true;
    sampledPointsWithFn.clear();
    sampledPointsWithFn.insert(make_pair(map->distance(startNode,goalNode),startNode));


    Node::Ptr xrand;
    //init the tree,put start node and a vaild child in the tree
    while(1){
        auto xrandSet=map->ellipse_sample_star(startNode,goalNode,stepsize_);
        //cout<<xrandSet.size()<<endl;
        //vis_Xrand(xrandSet,map,ellipse_pub);
        bool init=false;
        for (auto iter=xrandSet.begin();iter!=xrandSet.end();iter++) {
            xrand=iter->second;
            if(map->ValidConnect(xrand,startNode)){
                xrand->Parent=startNode;
                xrand->gn=map->distance(xrand,startNode);
                sampledPointsWithFn.insert(make_pair(xrand->gn+map->distance(xrand,goalNode)+getGC(xrand,foundSolution),xrand));
                sampleTree.push_back(make_pair(xrand,startNode));
                init=true;
                break;
            }
        }
        if(init) break;
    }

    int j=0;
    while(j<iter_max){
        //for the first time ,xrand exist

        Node::Ptr Xbest;
        for (auto iter=sampledPointsWithFn.begin();iter!=sampledPointsWithFn.end();iter++) {
            if(!iter->second->Visted){
                Xbest=iter->second;
                break;
            }
        }
        if(Xbest== nullptr) continue;
        auto XrandSet=map->ellipse_sample_star(Xbest,goalNode,stepsize_);
        Xbest->Visted=true;

        while(XrandSet.size()!=0){
            if(j!=0 && !foundSolution){
                xrand=XrandSet.begin()->second;
                XrandSet.erase(XrandSet.begin());
                if(xrand== nullptr) continue;
            }
            else if(foundSolution){
                int NodeInPathSize=pathNode.size();
                //sample from goal to start
                Node::Ptr KstartNode=pathNode[N];
                if(times>(2*N) && N<NodeInPathSize-1){
                    KstartNode=pathNode[N];
                    N++;
                    times=0;
                } else if(N==(NodeInPathSize-1)){
                    KstartNode=startNode;
                    //cout<<"Run Global Optimization"<<endl;
                }
                if(!KstartNode) continue;
                float Klength=0;
                if(KstartNode!=startNode){
                    Node::Ptr temp=goalNode->Parent;
                    Klength+=map->distance(temp,goalNode);
                    while(temp!= KstartNode && temp!= nullptr){
                        if(temp->Parent)Klength+=map->distance(temp,temp->Parent);
                        temp=temp->Parent;
                    }
                } else{
                    Klength=lastAverage;
                }
                xrand=map->ellipse_sample(KstartNode,goalNode,Klength);
                vis_ellipse(KstartNode,goalNode,map->distance(KstartNode,goalNode),Klength,ellipse_pub);
                if(!xrand) continue;
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
            //new add
            if(map->ValidConnect(minNode,xnew)){
                xnew->Parent=minNode;
                xnew->gn=minNode->gn+map->distance(minNode,xnew);
                sampledPointsWithFn.insert(make_pair(xnew->gn+map->distance(xnew,goalNode)+getGC(xnew,foundSolution),xnew));
                sampleTree.push_back(make_pair(xnew,minNode));
            } else continue;

            if(foundSolution){
                Rewire(xnew,nearestK);
            }
            j++;
            if(!foundSolution){
                if(map->distance(xnew,goalNode)<4*stepsize_){
                    int iterMax=30;
                    while(iterMax){
                        auto nearGoal=map->uniform_sample_area(xnew->pw[0],xnew->pw[1],4*stepsize_,80);
                        if(nearGoal!= nullptr && map->ValidConnect(nearGoal,goalNode) &&map->ValidConnect(nearGoal,xnew)){
                            if(map->distance(nearGoal,goalNode)<2*stepsize_){
                                nearGoal->Parent=xnew;
                                nearGoal->gn=xnew->gn+map->distance(xnew,nearGoal);
                                //sampledPoints.push_back(nearGoal);
                                sampledPointsWithFn.insert(make_pair(nearGoal->gn+map->distance(nearGoal,goalNode)+getGC(nearGoal,foundSolution),nearGoal));

                                goalNode->Parent=nearGoal;
                                goalNode->gn=nearGoal->gn+map->distance(nearGoal,goalNode);
                                sampledPointsWithFn.insert(make_pair(goalNode->gn+getGC(goalNode,foundSolution),goalNode)); //TODO
                                sampleTree.push_back(make_pair(nearGoal,goalNode));

                                //show path
                                {
                                    vector<Vector2f> path;
                                    path.push_back(Vector2f(goalNode->pw[0],goalNode->pw[1]));
                                    Node::Ptr temp=goalNode->Parent;
                                    lastAverage+=map->distance(temp,goalNode);
                                    pathNode.push_back(goalNode);
                                    while(temp!= nullptr){
                                        if(temp->Parent)lastAverage+=map->distance(temp,temp->Parent);
                                        pathNode.push_back(temp);
                                        length.push_back(lastAverage);
                                        path.push_back(Vector2f(temp->pw[0],temp->pw[1]));
                                        temp=temp->Parent;
                                    }

                                    vis_path(path,path_pub);
                                }

                                foundSolution= true;

                                //cout<<"Infromed RRT* sovled "<<j<<" times to find a path,staring local optimization"<<endl;
                                findSolutionTime=chrono::steady_clock::now();
                                break;
                            }
                            j++;

                        }
                        iterMax--;
                    }

                }
            }
            if(foundSolution){
                pathNode.clear();
                vector<Vector2f> path;
                path.push_back(Vector2f(goalNode->pw[0],goalNode->pw[1]));
                Node::Ptr temp=goalNode->Parent;
                pathNode.push_back(goalNode);
                float pathLength=0;
                pathLength+=map->distance(temp,goalNode);
                while(temp!= nullptr){
                    if(temp->Parent)pathLength+=map->distance(temp,temp->Parent);
                    pathNode.push_back(temp);
                    path.push_back(Vector2f(temp->pw[0],temp->pw[1]));
                    temp=temp->Parent;
                }

                length.push_back(pathLength);
                vis_path(path,path_pub);

            }


            if(foundSolution && length.size()>3){
                currenAverage=0;
                for (auto iter=length.end()-1;iter!=length.end()-4;iter--) {
                    currenAverage+=(*iter);
                }
                currenAverage=currenAverage/3.0;

                if((abs(lastAverage-currenAverage))<0.01){
                    times++;
                } else times--;
                lastAverage=currenAverage;
            }

            if(times>300){
                cout<<"optimization done!"<<endl;
                OptSolved=chrono::steady_clock::now();
                auto find_duration = chrono::duration_cast<chrono::microseconds>(findSolutionTime - start);
                auto opt_duration=chrono::duration_cast<chrono::microseconds>(OptSolved - findSolutionTime);
                cout <<  "InformedRRT* cost "
                     << double(find_duration.count()) * chrono::microseconds::period::num / chrono::microseconds::period::den
                     << "second to find a valiable path " << endl;
                cout <<  "InformedRRT* cost "
                     << double(opt_duration.count()) * chrono::microseconds::period::num / chrono::microseconds::period::den
                     << "second to optimaze a path " << endl;
                cout<<"the shortest path is "<<length[length.size()-1]<<" m "<<endl;
                return true;
            }

            //Vis_sample(sampledPoints,sample_point_pub);
            Vis_sample_tree(sampleTree,sampletreePub);
        }

    }
    cout<<"RRT couldnt find a path "<<endl;
    return false;
}

