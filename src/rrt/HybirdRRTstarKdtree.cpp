//
// Created by unicorn on 2020/1/2.
//
#include "rrt//HybirdRRTstarKdtree.h"
HybirdRRTstarKdtree::HybirdRRTstarKdtree() {
    sampleTreeCloud=boost::make_shared<pcl::PointCloud<pcl::PointXY>>();
}

void HybirdRRTstarKdtree::setStart(Vector2f start){
    startNode=make_shared<Node>(start);
    startNode->gn=0;
    startNode->Parent= nullptr;

}
void HybirdRRTstarKdtree::setStepSize(float stepsize){
    stepsize_=stepsize;
}

void HybirdRRTstarKdtree::setGoal(Vector2f goal){
    goalNode=make_shared<Node>(goal);
    minDis=map->distance(startNode,goalNode);
}
void HybirdRRTstarKdtree::setMap(OccMap *occMap){
    map=occMap;
}
bool HybirdRRTstarKdtree::isInitilized(){
    return map!= nullptr&& goalNode!= nullptr&& startNode!= nullptr;
}

Node::Ptr HybirdRRTstarKdtree::Near(Node::Ptr n){
    pcl::PointXY point;
    point.x=n->pw[0];
    point.y=n->pw[1];
    kdtree.setInputCloud(sampleTreeCloud);
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if(kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance)>0){
        return searchChart[pointIdxNKNSearch[0]]->second;
    } else return nullptr;


}
float HybirdRRTstarKdtree::getHeuristic( Node::Ptr p) {
    kdtree.setInputCloud(sampleTreeCloud);
    pcl::PointXY point;
    point.x=p->pw[0];
    point.y=p->pw[1];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius =stepsize_;
    int count=0;
    vector<multimap<float,Node::Ptr>::iterator> fliterset;
    if ( kdtree.radiusSearch (point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
        for (int j = 0; j <pointIdxRadiusSearch.size() ; ++j) {
            count++;
            if(count>2){
                fliterset.push_back(searchChart[pointIdxRadiusSearch[j]]);
            }
        }
    }

    //remove the node that has high density
    for (int j = 0; j <fliterset.size() ; ++j) {
        fliterset[j]->second->Visted=true;
    }
    float penalty=pow(count,5)*stepsize_;
    float dis=map->distance(p,goalNode);
    float compensate=-(p->gn)/5.0;
    if(count<=2) return p->gn+dis+compensate;
    else return p->gn+penalty+dis;
}
Node::Ptr HybirdRRTstarKdtree::sampleWithLowDensity(Node::Ptr curr, Node::Ptr goal, float length) {
    kdtree.setInputCloud(sampleTreeCloud);
    std::uniform_real_distribution<double> dist {0, 2 * M_PI};
    std::random_device rd;
    std::default_random_engine rng {rd()};
    std::uniform_real_distribution<double> distribution {-1.0, 1.0};

    float c = map->distance(curr, goal) / 2.0;
    float a = length / 2.0;
    float b = sqrt(a * a - c * c);
    float x0 = (curr->pw[0] + goal->pw[0]) / 2.0;
    float y0 = (curr->pw[1] + goal->pw[1]) / 2.0;
    float rot = atan((curr->pw[1] - goal->pw[1]) / (curr->pw[0] - goal->pw[0]));
    Eigen::Matrix3f R = Eigen::AngleAxisf(rot, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    vector<Node::Ptr> randset;
    pcl::PointCloud<pcl::PointXY>::Ptr randSetCloud(new pcl::PointCloud<pcl::PointXY>);
    for (int j = 0; j < 20; ++j) {
        float alpha = distribution(rng);
        float theta = dist(rng);
        float x = a * cos(theta) * alpha;
        float y = b * sin(theta) * alpha;
        Vector3f ell = R * Vector3f(x, y, 0) + Vector3f(x0, y0, 0);
        if (map->isValid(ell.x(), ell.y())){
            randset.push_back(make_shared<Node>(Vector2f(ell.x(),ell.y())));
            pcl::PointXY p;
            p.x=ell.x();p.y=ell.y();
            randSetCloud->points.push_back(p);
        }

    }
    if(randset.empty()) return nullptr;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius =stepsize_;
    int countMin=INT_MAX;
    int indexMin=0;
    for (int l = 0; l <randset.size() ; ++l) {
        if(int ct=kdtree.radiusSearch (randSetCloud->points[l], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)<countMin){
            countMin=ct;
            indexMin=l;
            //to make sample more average on low density area
            for (int j = 0; j <pointIdxRadiusSearch.size() ; ++j) {
                searchChart[pointIdxRadiusSearch[j]]->second->Visted=true;
            }
        }
    }
    return randset[indexMin];
}

bool HybirdRRTstarKdtree::Steer(Node::Ptr n1,Node::Ptr n2,Node::Ptr &xnew){
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
multimap<float,Node::Ptr> HybirdRRTstarKdtree::getNearestR(Node::Ptr cur,float radius){
    kdtree.setInputCloud(sampleTreeCloud);
    pcl::PointXY point;
    point.x=cur->pw[0];
    point.y=cur->pw[1];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    multimap<float,Node::Ptr> nearKMap;
    if(kdtree.radiusSearch (point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)>0){
        for (int j = 0; j <pointIdxRadiusSearch.size() ; ++j) {
            float dis=map->distance(searchChart[pointIdxRadiusSearch[j]]->second,cur);
            if(dis<radius && map->ValidConnect(cur,searchChart[pointIdxRadiusSearch[j]]->second)){
                nearKMap.insert(make_pair(dis,searchChart[pointIdxRadiusSearch[j]]->second));
            }
        }
    }
    return nearKMap;
}
void HybirdRRTstarKdtree::Rewire(Node::Ptr cur,multimap<float,Node::Ptr> nearkmap){
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

bool HybirdRRTstarKdtree::solve(int iter_max,ros::Publisher sampletreePub,ros::Publisher sample_point_pub,ros::Publisher path_pub,ros::Publisher ellipse_pub){
    auto start=chrono::steady_clock::now();
    auto findSolutionTime=start;
    auto OptSolved=start;

    bool foundSolution=false;
    bool globalOptimization=false;
    float lastAverage=0;
    float lastaveragecost=INT_MAX;
    float currenAverage=INT_MAX;//path length in average
    int N=1;
    int times=0;
    vector<float> length;//record path length

    startNode->Visted=true;
    sampledPointsWithFn.clear();
    sampleTreeCloud->clear();
    searchChart.clear();
    //we must carefully arrange their relationship
    auto iter=sampledPointsWithFn.insert(make_pair(map->distance(startNode,goalNode),startNode));
    pcl::PointXY startPoint;
    startPoint.x=startNode->pw[0];
    startPoint.y=startNode->pw[1];
    sampleTreeCloud->points.push_back(startPoint);
    searchChart.insert(make_pair(sampleTreeCloud->points.size()-1,iter));


    Node::Ptr xrand;
    int child=0;
    //init the tree,put start node and 10 vaild child in the tree
    while(1){
        auto xrandSet=map->ellipse_sample_star(startNode,goalNode,stepsize_);
        bool init=false;
        for (auto iter=xrandSet.begin();iter!=xrandSet.end();iter++) {
            xrand=iter->second;
            //target may lies directly on a collision free line from start,this is the shorest path without optimization
            if(xrand==goalNode){
                //naive operation
                {
                    vector<Vector2f> path;
                    path.push_back(Vector2f(goalNode->pw[0],goalNode->pw[1]));
                    path.push_back(Vector2f(startNode->pw[0],startNode->pw[1]));
                    vis_path(path,path_pub);
                    auto find_duration = chrono::duration_cast<chrono::microseconds>(findSolutionTime - start);
                    cout <<  "Hybird RRT* cost "
                         << double(find_duration.count()) * chrono::microseconds::period::num / chrono::microseconds::period::den
                         << "second to find a valiable path " << endl;
                    cout <<  "Hybird RRT* cost 0 second to optimaze a path " << endl;
                    cout<<"the shortest path is "<<map->distance(startNode,goalNode)<<" m "<<endl;
                }
                return true;
            }
            //if xrand can be connected with start,the tree is inited
            if(map->ValidConnect(xrand,startNode)){
                xrand->Parent=startNode;
                xrand->gn=map->distance(xrand,startNode);
                //we must carefully arrange their relationship
                auto iter=sampledPointsWithFn.insert(make_pair(getHeuristic(xrand),xrand));
                pcl::PointXY xrandPoint;
                xrandPoint.x=xrand->pw[0];
                xrandPoint.y=xrand->pw[1];
                sampleTreeCloud->points.push_back(xrandPoint);
                searchChart.insert(make_pair(sampleTreeCloud->points.size()-1,iter));
                sampleTree.push_back(make_pair(xrand,startNode));
                child++;
                if(child>10){
                    init=true;
                    break;
                }

            }
        }
        if(init) break;
    }
// we use j to count iteration
    int j=0;
    while(j<iter_max){
        if(!foundSolution){
            //choose unvisited node of minimum heuristic cost in sample points to grow the tree
            //this Xbest should be:
            //1.has not been visted
            //2.it has a small density(the number of neighbors)
            //3.it is closer to goal point
            Node::Ptr Xbest;
            int spSize=0;
            for (auto iter=sampledPointsWithFn.begin();iter!=sampledPointsWithFn.end();iter++) {
                if(!iter->second->Visted){
                    Xbest=iter->second;
                    spSize++;
                    break;
                }
            }
            //sample points are all invalid,we choose a visted but low cost node
            if(spSize==sampledPointsWithFn.size()){
                Xbest=sampledPointsWithFn.begin()->second;
            }
            if(Xbest== nullptr) continue;
            //use Xbest to sample,then Xbest would never to be used to sample/grow
            auto XrandSet=map->ellipse_sample_star(Xbest,goalNode,stepsize_);
            Xbest->Visted=true;
            //vis_Xrand(XrandSet,map,sample_point_pub);

            while(XrandSet.size()!=0 &&!foundSolution){
                //for the first time ,xrand exist
                xrand=XrandSet.begin()->second;
                XrandSet.erase(XrandSet.begin());
                //we get the shortes path from xbest
                if(xrand==goalNode){
                    goalNode->Parent=Xbest;
                    goalNode->gn=Xbest->gn+map->distance(Xbest,goalNode);
                    //we must carefully arrange their relationship
                    auto iter=sampledPointsWithFn.insert(make_pair(getHeuristic(Xbest),goalNode));
                    pcl::PointXY goalPoint;
                    goalPoint.x=goalNode->pw[0];
                    goalPoint.y=goalNode->pw[1];
                    sampleTreeCloud->points.push_back(goalPoint);
                    searchChart.insert(make_pair(sampleTreeCloud->points.size()-1,iter));

                    sampleTree.push_back(make_pair(Xbest,goalNode));
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

                    cout<<"Hybird RRT* sovled "<<j<<" times to find a path,staring local optimization"<<endl;
                    findSolutionTime=chrono::steady_clock::now();
                    break;
                }
                if(xrand== nullptr) continue;

                shared_ptr<Node> xnear=Near(xrand);
                if(xnear== nullptr) continue;
                shared_ptr<Node> xnew;
                if(!Steer(xrand,xnear,xnew)) continue;

                //find nodes that near xnew
                auto nearestK=getNearestR(xnew,2*stepsize_);
                if(nearestK.empty())continue;

                //find a node that has minimum cost grow to xnew
                Node::Ptr minNode;
                float minGn=INT_MAX;
                for (auto iter=nearestK.begin();iter!=nearestK.end();iter++) {
                    float dis=iter->second->gn+map->distance(iter->second,xnew);
                    if(dis<minGn){
                        minGn=dis;
                        minNode=iter->second;
                    }
                }
                //if minNode is valid connection with xnew,add it to the tree
                if(map->ValidConnect(minNode,xnew)){
                    xnew->Parent=minNode;
                    xnew->gn=minNode->gn+map->distance(minNode,xnew);

                    //we must carefully arrange their relationship
                    auto iter=sampledPointsWithFn.insert(make_pair(getHeuristic(xnew),xnew));
                    pcl::PointXY xnewPoint;
                    xnewPoint.x=xnew->pw[0];
                    xnewPoint.y=xnew->pw[1];
                    sampleTreeCloud->points.push_back(xnewPoint);
                    searchChart.insert(make_pair(sampleTreeCloud->points.size()-1,iter));

                    sampleTree.push_back(make_pair(xnew,minNode));
                } else continue;

                //if we cannot get goal node by the sample method,we do this by short distance
                //this may happen in a full of obsatcle case
                if(map->distance(xnew,goalNode)<4*stepsize_ &&!foundSolution){
                    //we try to iterate 30 times to generate a point that connect both xnew and goal without collision
                    //if we dont find contine;
                    int iterMax=30;
                    while(iterMax){
                        auto nearGoal=map->uniform_sample_area(xnew->pw[0],xnew->pw[1],4*stepsize_,80);
                        if(nearGoal!= nullptr && map->ValidConnect(nearGoal,goalNode) &&map->ValidConnect(nearGoal,xnew)){
                            if(map->distance(nearGoal,goalNode)<2*stepsize_){
                                nearGoal->Parent=xnew;
                                nearGoal->gn=xnew->gn+map->distance(xnew,nearGoal);


                                //we must carefully arrange their relationship
                                auto iter=sampledPointsWithFn.insert(make_pair(getHeuristic(nearGoal),nearGoal));
                                pcl::PointXY nearGoalPoint;
                                nearGoalPoint.x=nearGoal->pw[0];
                                nearGoalPoint.y=nearGoal->pw[1];
                                sampleTreeCloud->points.push_back(nearGoalPoint);
                                searchChart.insert(make_pair(sampleTreeCloud->points.size()-1,iter));


                                goalNode->Parent=nearGoal;
                                goalNode->gn=nearGoal->gn+map->distance(nearGoal,goalNode);


                                //we must carefully arrange their relationship
                                iter=sampledPointsWithFn.insert(make_pair(goalNode->gn,goalNode)); //TODO
                                pcl::PointXY goalNodePoint;
                                goalNodePoint.x=goalNode->pw[0];
                                goalNodePoint.y=goalNode->pw[1];
                                sampleTreeCloud->points.push_back(nearGoalPoint);
                                searchChart.insert(make_pair(sampleTreeCloud->points.size()-1,iter));

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
                                //cout<<"Hybrid  RRT* sovled "<<j<<" times to find a path,staring local optimization"<<endl;
                                findSolutionTime=chrono::steady_clock::now();
                                break;
                            }
                        }
                        iterMax--;
                    }
                }

                Rewire(xnew,nearestK);
                j++;
                float averagecost=0;
                //remove the the node that cost is too large(high density)
                for(auto iter=sampledPointsWithFn.begin();iter!=sampledPointsWithFn.end();iter++){
                    averagecost+=iter->first;
                    if(iter->first>1.2*lastaveragecost){
                        iter->second->Visted=true;
                    }
                }
                averagecost=averagecost/sampledPointsWithFn.size();
                lastaveragecost=averagecost;
                //cout<<"average is "<<averagecost<<endl;
                Vis_sample_tree(sampleTree,sampletreePub);
            }
        }
        else if(foundSolution){
            //pathNode [goal x x x x x start]
            int NodeInPathSize = pathNode.size();
            //sample from goal to start
            //N start from 1,the latter node by goal
            Node::Ptr KstartNode = pathNode[N];
            if (!KstartNode) continue;
            float Klength = 0;
            //if KstartNode is not goal,we need to calculate path length
            if (KstartNode != startNode) {
                Node::Ptr temp = goalNode -> Parent;
                Klength += map -> distance(temp, goalNode);
                while (temp != KstartNode && temp != nullptr) {
                    if (temp -> Parent)Klength += map -> distance(temp, temp -> Parent);
                    temp = temp -> Parent;
                }
            } else {
                Klength = lastAverage;
            }
            //if path length is too short,enlarge the ellipse
            if(Klength<5*stepsize_){
                N++;
                continue;
            }
            //KstartNode would optimize abs(X-lengthStep) times,
            // that is if KstartNode is closer to goal
            //the less would KstartNode optimize
            if (times > abs((lastAverage/stepsize_-(Klength/stepsize_))) && N < NodeInPathSize - 1) {
                N++;
                times = 0;
            } else if (N == (NodeInPathSize - 1)) {
                KstartNode = startNode;
                globalOptimization= true;
                //cout<<"Run Global Optimization"<<endl;
            }

            //we use informed rrt* sample method,what differs is that we change the ellipse from goal to start until start
            xrand = sampleWithLowDensity(KstartNode, goalNode, Klength);
            if (!xrand) continue;
            vis_ellipse(KstartNode, goalNode, map -> distance(KstartNode, goalNode), Klength, ellipse_pub);
            //same as above
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
            //if minNode is valid connection with xnew,add it to the tree
            if(map->ValidConnect(minNode,xnew)){
                xnew->Parent=minNode;
                xnew->gn=minNode->gn+map->distance(minNode,xnew);



                //we must carefully arrange their relationship
                iter=sampledPointsWithFn.insert(make_pair(getHeuristic(xnew),xnew));
                pcl::PointXY xnewPoint;
                xnewPoint.x=xnew->pw[0];
                xnewPoint.y=xnew->pw[1];
                sampleTreeCloud->points.push_back(xnewPoint);
                searchChart.insert(make_pair(sampleTreeCloud->points.size()-1,iter));

                sampleTree.push_back(make_pair(xnew,minNode));
            } else continue;

            Rewire(xnew,nearestK);
            j++;

            Vis_sample_tree(sampleTree,sampletreePub);
            //vis path
            {
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


            if(length.size()>3){
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

            if(times>100 && globalOptimization){
                cout<<"optimization done!"<<endl;
                OptSolved=chrono::steady_clock::now();
                auto find_duration = chrono::duration_cast<chrono::microseconds>(findSolutionTime - start);
                auto opt_duration=chrono::duration_cast<chrono::microseconds>(OptSolved - findSolutionTime);
                cout <<  "Hybird RRT* cost "
                     << double(find_duration.count()) * chrono::microseconds::period::num / chrono::microseconds::period::den
                     << "second to find a valiable path " << endl;
                cout <<  "Hybird RRT* cost "
                     << double(opt_duration.count()) * chrono::microseconds::period::num / chrono::microseconds::period::den
                     << "second to optimaze a path " << endl;
                cout<<"the shortest path is "<<length[length.size()-1]<<" m "<<endl;
                return true;
            }

        }

    }
    cout<<"RRT couldnt find a path "<<endl;
    return false;
}






