//
// Created by unicorn on 2019/12/27.
//
#include <map/OccMap.h>
void OccMap::setMap(nav_msgs::OccupancyGridConstPtr map){
    height_=map->info.height;
    width_=map->info.width;
    res_=map->info.resolution;
    orix_=map->info.origin.position.x;
    oriy_=map->info.origin.position.y;
    gridmap=*map;
    minx_=orix_;
    maxx_=width_*res_+orix_;
    miny_=oriy_;
    maxy_=height_*res_+oriy_;
    cout<<"map loaded,please set target in rviz"<<endl;
    int t=gridmap.data.size();
    array<int,16> inc={0,0,1,-1,1,1,-1,-1,1,-1,0,0,1,-1,1,-1};
    int* modified=new int[t];
    memset(modified, 0, sizeof(modified));

    for (int j = 0; j <gridmap.data.size() ; ++j) {
        int index=j;
        int inflation=inflation_radius_/res_;
        int icy=ceil((index/width_));
        int icx=index-width_*icy;
        if(!isValid(index) && !modified[index]){
            for (int k = 1; k <inflation ; ++k) {
                for (int l = 0; l <8 ; ++l) {
                    int x=icx+k*inc[l%8];
                    int y=icy+k*inc[l%8+8];
                    if(x<0||y<0||x+width_*y>=gridmap.data.size()) continue;
                    if(gridmap.data[x+width_*y]!= static_cast<int8_t >(100)) {
                        gridmap.data[x+width_*y]= static_cast<int8_t >(100);
                        costmap.push_back(x+width_*y);
                        modified[x+width_*y]=true;
                    }
                }
            }
        }
    }

    //cout<<"cost map is "<<costmap.size()<<endl;
}

pair<double,double> OccMap::inx2coor(int index){
    int icy=ceil((index/width_));
    int icx=index-width_*icy;
    double cx=(icx*res_+orix_);
    double cy=(icy*res_+oriy_);
    return make_pair(cx,cy);
}

Vector2f OccMap::inx2coor(Vector2i index){
    int icy=index[1];
    int icx=index[0];
    double cx=(icx*res_+orix_);
    double cy=(icy*res_+oriy_);
    return Vector2f(cx,cy);
}

long int OccMap::coor2inx(double cx, double cy){

    int mapx=ceil((cx-orix_)/res_);
    int mapy=ceil((cy-oriy_)/res_);
    return mapx+width_*(mapy);
}

Vector2i OccMap::coor2XYPixel(Vector2f Pw){
    int mapx=ceil((Pw[0]-orix_)/res_);
    int mapy=ceil((Pw[1]-oriy_)/res_);
    return Vector2i(mapx,mapy);
}

long int OccMap::XYPixel2index(Vector2i p){
    return p.x()+width_*p.y();
}



bool OccMap::isValid(double cx,double cy){
    int index=coor2inx(cx,cy);
    if(index<0 || index>gridmap.data.size()-1) return false;
    else return static_cast<int>(gridmap.data.at(index))==0;
}



bool OccMap::isValid(int index){
    if(index<=0 || index>=gridmap.data.size())
        return false;
    else
        return static_cast<int>(gridmap.data.at(index))==0;
}

bool OccMap::isValid(Vector2i pixel){
    return isValid(pixel[0]+pixel[1]*width_);
}

//if non-free points in a pixel circle of r around p is over limit,then p is non-free
bool OccMap::isValidPoint(Vector2f p,int r){
    auto pixel=coor2XYPixel(p);
    int count=0;
    if(!isValid(pixel)) count++;
    for (int j = 0; j <r ; ++j) {
        for (int k = 0; k <rules.size() ; ++k) {
            if(!isValid(pixel+j*rules[k])) count++;
        }
    }
    return float(count)/(rules.size()*r)<0.02*r;
}

bool OccMap::isValid(double Acx,double Acy,double Bcx,double Bcy){
    double costh=abs((Bcx-Acx))/sqrt(pow(Bcy-Acy,2)+pow(Bcx-Acx,2));
    double sinth=sqrt(1-pow(costh,2));
    int segments=round(sqrt(pow(Bcy-Acy,2)+pow(Bcx-Acx,2))/step_);
    double cx=Acx,cy=Acy;
    double xsign=1.0,ysign=1.0;
    if(Acx>Bcx){
        xsign=-1.0;
    }
    if(Acy>Bcy){
        ysign=-1.0;
    }
    for(int i=0;i<segments;i++){
        double tcx=cx+double(i)*step_*costh*xsign;
        double tcy=cy+double(i)*step_*sinth*ysign;
        if(!isValid(tcx,tcy)){
            return false;
        }
    }
    return true;
}
bool OccMap::ValidConnect(Node::Ptr p1,Node::Ptr p2){
    return isValid(p1->pw[0],p1->pw[1],p2->pw[0],p2->pw[1]);
}
//p1 is frist node ,it links goal or not
bool OccMap::ValidConnect(Node::Ptr p1,Node::Ptr goal,Node::Ptr &pnew){
    float Acx=p1->pw[0];float Acy=p1->pw[1];float Bcx=goal->pw[0];float Bcy=goal->pw[1];
    double costh=abs((Bcx-Acx))/sqrt(pow(Bcy-Acy,2)+pow(Bcx-Acx,2));
    double sinth=sqrt(1-pow(costh,2));
    int segments=round(sqrt(pow(Bcy-Acy,2)+pow(Bcx-Acx,2))/step_);
    double cx=Acx,cy=Acy;
    double xsign=1.0,ysign=1.0;
    if(Acx>Bcx){
        xsign=-1.0;
    }
    if(Acy>Bcy){
        ysign=-1.0;
    }
    Vector2f lastpoint;
    for(int i=0;i<segments;i++){
        double tcx=cx+double(i)*step_*costh*xsign;
        double tcy=cy+double(i)*step_*sinth*ysign;
        if(!isValid(tcx,tcy)&& i>1&& isValid(lastpoint.x(),lastpoint.y())){
            pnew=make_shared<Node>(lastpoint);
            pnew->gn=p1->gn+distance(p1,pnew);
            return true;
        }
        if(i==1 && !isValid(tcx,tcy))
            return false;
        lastpoint=Vector2f(tcx,tcy);
    }
    pnew=goal;
    return true;
}

bool OccMap::uniform_sample(Node::Ptr & result){
    std::uniform_real_distribution<double > distx{minx_,maxx_};
    std::uniform_real_distribution<double > disty{miny_,maxy_};
    std::random_device rd;
    std::default_random_engine rng {rd()};
    double x=distx (rng);
    double y=disty(rng);
    if(isValid(x,y)){
        result=make_shared<Node>(Vector2f(x,y));
        return true;
    } else{
        result= nullptr;
        return false;

    }


}

Node::Ptr OccMap::uniform_sample_area(double originx, double originy,double r,int times){
    double minx=max(minx_,originx-r);
    double maxx=min(maxx_,originx+r);
    double miny=max(miny_,originy-r);
    double maxy=min(maxy_,originy+r);
    std::uniform_real_distribution<double > distx{minx,maxx};
    std::uniform_real_distribution<double > disty{miny,maxy};
    std::random_device rd;
    std::default_random_engine rng {rd()};
    for (int j = 0; j <times ; ++j) {
        double x=distx (rng);
        double y=disty(rng);
        if(isValid(x,y)){
            return make_shared<Node>(Vector2f(x,y));
        }
    }
    return nullptr;
}
//a is half of distance between n1 and n2
Node::Ptr OccMap::ellipse_sample(Node::Ptr n1,Node::Ptr n2,float pathlength){
    std::uniform_real_distribution<double > dist{0,2*M_PI};
    std::random_device rd;
    std::default_random_engine rng {rd()};
    std::uniform_real_distribution<double > distribution{-1.0,1.0};

    float c=distance(n1,n2)/2.0;
    float a=pathlength/2.0;
    float b=sqrt(a*a-c*c);
    float x0=(n1->pw[0]+n2->pw[0])/2.0;
    float y0=(n1->pw[1]+n2->pw[1])/2.0;
    float rot=atan((n1->pw[1]-n2->pw[1])/(n1->pw[0]-n2->pw[0]));
    Eigen::Matrix3f R=Eigen::AngleAxisf(rot,Eigen::Vector3f::UnitZ()).toRotationMatrix();
    for (int j = 0; j <20 ; ++j) {
        float alpha=distribution(rng);
        float theta=dist(rng);
        float x=a*cos(theta)*alpha;
        float y=b*sin(theta)*alpha;
        Vector3f ell=R*Vector3f(x,y,0)+Vector3f(x0,y0,0);
        if(isValid(ell.x(),ell.y()))
            return make_shared<Node>(Vector2f(ell.x(),ell.y()));
    }
    return nullptr;
}
//sample a line toward goal(may not link the whole path but most collision free) and at most 10 rand point in the ellipse of goal and start
multimap<float,Node::Ptr> OccMap::ellipse_sample_star(Node::Ptr curr,Node::Ptr goal,float min_step){
    multimap<float,Node::Ptr> OpenSet;
    float c=distance(curr,goal)/2.0;
    float a=5.0*c;
    float b=sqrt(a*a-c*c);
    float rot=atan((curr->pw[1]-goal->pw[1])/(curr->pw[0]-goal->pw[0]));
    Eigen::Matrix3f R=Eigen::AngleAxisf(rot,Eigen::Vector3f::UnitZ()).toRotationMatrix();
    std::uniform_real_distribution<double > dist{0,2*M_PI};
    std::uniform_real_distribution<double > distribution{-1.0,1.0};
    std::random_device rd;
    std::default_random_engine rng {rd()};

    float x0=(curr->pw[0]+goal->pw[0])/2.0;
    float y0=(curr->pw[1]+goal->pw[1])/2.0;

    //sample in a ellipse
    for (int j = 0; j <500 ; ++j) {
        float theta=dist(rng);
        float alpha=distribution(rng);
        float x=a*cos(theta)*alpha;
        float y=b*sin(theta)*alpha;
        if((abs(x-curr->pw[0])+abs(y-curr->pw[1]))<min_step) continue;
        Vector3f ell=R*Vector3f(x,y,0)+Vector3f(x0,y0,0);
        float dis=sqrt(pow(x-goal->pw[0],2)+pow(y-goal->pw[1],2));
        auto node=make_shared<Node>(Vector2f(ell.x(),ell.y()));
        if(!isValid(ell.x(),ell.y())) continue;
        //low density sample
        OpenSet.insert(make_pair(curr->gn+dis,node));
        if(OpenSet.size()>50) break;
    }

//choose a best node toward goal
    Node::Ptr bestNode;
    if(ValidConnect(curr,goal,bestNode)){
        float dis=distance(curr,bestNode)+((bestNode==goal)?-curr->gn:distance(bestNode,goal));
        OpenSet.insert(make_pair(curr->gn+dis,bestNode));
    }
    return OpenSet;
}

float OccMap::distance(Node::Ptr n1,Node::Ptr n2){
    return sqrt(pow(n1->pw[0]-n2->pw[0],2)+pow(n1->pw[1]-n2->pw[1],2));
}
