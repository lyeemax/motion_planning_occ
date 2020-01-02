//
// Created by unicorn on 2019/12/28.
//

#include "tools/Viewer.h"
void Vis_sample(vector<Node::Ptr> points , ros::Publisher sampler_pub) {
    static long int id = 0;
    visualization_msgs::Marker Points;
    Points.header.frame_id = "map";
    Points.header.stamp = ros::Time::now();
    Points.ns = "Points";
    Points.action = visualization_msgs::Marker::ADD;
    Points.pose.orientation.w = 1.0;
    Points.id = 0;
    Points.type = visualization_msgs::Marker::POINTS;
    Points.scale.x = 0.2;
    Points.scale.y = 0.2;
    Points.color.r = 1.0f;
    Points.color.a = 0.5;
    for (size_t k = 0; k < points.size(); ++k) {

        geometry_msgs::Point pt1;
        pt1.x = points[k]->pw[0];
        pt1.y = points[k]->pw[1];
        Points.points.push_back(pt1);
    }
    sampler_pub.publish(Points);

}
void Vis_sample_tree(vector<pair<Node::Ptr,Node::Ptr>> trees , ros::Publisher sampler_tree_pub){
    int id=0;
    visualization_msgs::MarkerArray array;
    for (int j = 0; j <trees.size() ; ++j) {
        visualization_msgs::Marker edge;
        edge.header.frame_id = "map";
        edge.header.stamp = ros::Time::now();
        edge.ns = "tree";
        edge.action = visualization_msgs::Marker::ADD;
        edge.pose.orientation.w = 1.0;
        edge.id = id++;
        edge.type = visualization_msgs::Marker::LINE_STRIP;
        edge.scale.x = 0.1;
        edge.color.g= 0.8f;
        edge.color.a = 0.2f;
        geometry_msgs::Point pt1;
        pt1.x = trees[j].first->pw[0];
        pt1.y =trees[j].first->pw[1];
        geometry_msgs::Point pt2;
        pt2.x = trees[j].second->pw[0];
        pt2.y =trees[j].second->pw[1];
        edge.points.push_back(pt1);
        edge.points.push_back(pt2);
        array.markers.push_back(edge);
    }
    sampler_tree_pub.publish(array);
}
void vis_testmap(vector<Vector2f> testmap , ros::Publisher test_pub){
    static long int id = 0;
    visualization_msgs::Marker CoverPoints;
    CoverPoints.header.frame_id = "map";
    CoverPoints.header.stamp = ros::Time::now();
    CoverPoints.ns = "test";
    CoverPoints.action = visualization_msgs::Marker::ADD;
    CoverPoints.pose.orientation.w = 1.0;
    CoverPoints.id = 3;
    CoverPoints.type = visualization_msgs::Marker::POINTS;
    CoverPoints.scale.x = 0.2;
    CoverPoints.scale.y = 0.2;
    CoverPoints.color.r = 0.2f;
    CoverPoints.color.g = 0.5f;
    CoverPoints.color.a = 1;

    for (auto iter=testmap.begin();iter!=testmap.end();iter++) {
        geometry_msgs::Point cpoint;
        cpoint.x = iter->x();
        cpoint.y = iter->y();
        CoverPoints.points.push_back(cpoint);
    }
    test_pub.publish(CoverPoints);
}


void vis_costmap(OccMap *occmap , ros::Publisher costmap_pub) {
    static long int id = 0;
    visualization_msgs::Marker CostPoints;
    CostPoints.header.frame_id = "map";
    CostPoints.header.stamp = ros::Time::now();
    CostPoints.ns = "cost";
    CostPoints.action = visualization_msgs::Marker::ADD;
    CostPoints.pose.orientation.w = 1.0;
    CostPoints.id = 1;
    CostPoints.type = visualization_msgs::Marker::POINTS;
    CostPoints.scale.x = 0.2;
    CostPoints.scale.y = 0.2;
    CostPoints.color.r = 1.0f;
    CostPoints.color.g = 1.0f;
    CostPoints.color.a = 0.1;

    for (int m = 0; m < occmap->costmap.size(); ++m) {
        auto f = occmap->inx2coor(occmap->costmap[m]);
        geometry_msgs::Point costpoint;
        costpoint.x = f.first;
        costpoint.y = f.second;
        CostPoints.points.push_back(costpoint);
    }
    costmap_pub.publish(CostPoints);
}

void vis_path(vector<Vector2f> path , ros::Publisher path_pub) {
    visualization_msgs::Marker Line;
    Line.header.frame_id = "map";
    Line.header.stamp = ros::Time::now();
    Line.ns = "line";
    Line.action = visualization_msgs::Marker::ADD;
    Line.pose.orientation.w = 1.0;
    Line.id = 2;
    Line.type = visualization_msgs::Marker::LINE_STRIP;
    Line.scale.x = 0.3;
    Line.color.r = 1.0;
    Line.color.a = 1.0;
    for (int j = 0; j < path.size(); ++j) {
        geometry_msgs::Point pt2;
        pt2.x = path[j].x();
        pt2.y = path[j].y();
        pt2.z = 0;
        Line.points.push_back(pt2);

    }
    path_pub.publish(Line);
}


extern void vis_coverage(list<Vector2f> coverlist,OccMap *occmap,ros::Publisher cover_pub){
    static long int id = 0;
    visualization_msgs::Marker CoverPoints;
    CoverPoints.header.frame_id = "map";
    CoverPoints.header.stamp = ros::Time::now();
    CoverPoints.ns = "cover";
    CoverPoints.action = visualization_msgs::Marker::ADD;
    CoverPoints.pose.orientation.w = 1.0;
    CoverPoints.id = 3;
    CoverPoints.type = visualization_msgs::Marker::POINTS;
    CoverPoints.scale.x = 0.2;
    CoverPoints.scale.y = 0.2;
    CoverPoints.color.r = 1.0f;
    CoverPoints.color.g = 0.5f;
    CoverPoints.color.a = 1;

    for (auto iter=coverlist.begin();iter!=coverlist.end();iter++) {
        geometry_msgs::Point cpoint;
        cpoint.x = iter->x();
        cpoint.y = iter->y();
        CoverPoints.points.push_back(cpoint);
    }
    cover_pub.publish(CoverPoints);

}


extern void vis_Xrand(multimap<float,Node::Ptr> coverlist,OccMap *occmap,ros::Publisher cover_pub){
    static long int id = 0;
    visualization_msgs::Marker CoverPoints;
    CoverPoints.header.frame_id = "map";
    CoverPoints.header.stamp = ros::Time::now();
    CoverPoints.ns = "cover";
    CoverPoints.action = visualization_msgs::Marker::ADD;
    CoverPoints.pose.orientation.w = 1.0;
    CoverPoints.id = 3;
    CoverPoints.type = visualization_msgs::Marker::POINTS;
    CoverPoints.scale.x = 0.2;
    CoverPoints.scale.y = 0.2;
    CoverPoints.color.r = 1.0f;
    CoverPoints.color.g = 0.5f;
    CoverPoints.color.a = 1;

    for (auto iter=coverlist.begin();iter!=coverlist.end();iter++) {
        geometry_msgs::Point cpoint;
        cpoint.x = iter->second->x;
        cpoint.y = iter->second->y;
        CoverPoints.points.push_back(cpoint);
    }
    cover_pub.publish(CoverPoints);
    cout<<"test"<<endl;

}


void vis_ellipse(Node::Ptr n1,Node::Ptr n2,float focal, float pathlength,ros::Publisher ellipse_pub){
    visualization_msgs::Marker ellipse;
    ellipse.header.frame_id = "map";
    ellipse.header.stamp = ros::Time::now();
    ellipse.ns = "ellipse";
    ellipse.action = visualization_msgs::Marker::ADD;
    ellipse.pose.orientation.w = 1.0;
    ellipse.id = 3;
    ellipse.type = visualization_msgs::Marker::POINTS;
    ellipse.scale.x = 0.2;
    ellipse.scale.y = 0.2;
    ellipse.color.r = 1.0f;
    ellipse.color.g = 0.5f;
    ellipse.color.a = 1;
    float a=pathlength/2.0;
    float c=focal/2.0;
    float b=sqrt(a*a-c*c);
    float x0=(n1->pw[0]+n2->pw[0])/2.0;
    float y0=(n1->pw[1]+n2->pw[1])/2.0;
    double theta=atan((n1->pw[1]-n2->pw[1])/(n1->pw[0]-n2->pw[0]));
    Eigen::Matrix3f R=Eigen::AngleAxisf(theta,Eigen::Vector3f::UnitZ()).toRotationMatrix();
    float res=M_PI/180.0;
    for (int j = 0; j <180 ; ++j) {
        float x=a*cos(2.0*float(j)*res);
        float y=b*sin(2.0*float(j)*res);
        Vector3f ell=R*Vector3f(x,y,0)+Vector3f(x0,y0,0);
        geometry_msgs::Point cpoint;
        cpoint.x = ell.x();
        cpoint.y = ell.y();
        ellipse.points.push_back(cpoint);
    }
    ellipse_pub.publish(ellipse);

}
