/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Author: Rex Jomy Joseph, University of Southern California         //
//  Email: jomyjose@usc.edu                                            //
//                                                                     // 
/////////////////////////////////////////////////////////////////////////


/*******************************************/
//ROS HEADERS
/********************************************/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <std_srvs/Trigger.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "visualization_msgs/MarkerArray.h"
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <highgui.h>

/***********************************************/
//STANDARD HEADERS
/************************************************/
#include <iostream>
#include <cmath>

/***************************************************/
//OTHER HEADERS
/*************************************************/
#include <boost/make_shared.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "point_cloud_utilities/pcl_utilities.hpp"
#include "human_tracking/human_body.hpp"

using namespace std;
using namespace pcl;
using namespace cv;

/************************************************/
//Global Variables..
/************************************************/
ros::Publisher vis_pub;

inline void modifyMarker(visualization_msgs::MarkerArray& msg,int id, float r, float g, float b)
{
    msg.markers[id].color.r=r;
    msg.markers[id].color.g=g;
    msg.markers[id].color.b=b;
    // msg.marker.lifetime = ros::Duration(0);
}

inline visualization_msgs::Marker getLine(vector<float> pt1,vector<float> pt2,string frame_id,int id)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id=frame_id;
    line_list.header.stamp = ros::Time::now();
    line_list.pose.orientation.w = 1.0;
    line_list.id=id;
    line_list.type=visualization_msgs::Marker::ARROW;
    line_list.scale.x=0.035;
    line_list.scale.y=0.015;
    line_list.color.r=1;
    line_list.color.g=0;
    line_list.color.b=0;
    line_list.color.a=1;
    geometry_msgs::Point p1,p2;
    p1.x=pt1[0];
    p1.y=pt1[1];
    p1.z=pt1[2];
    p2.x=pt2[0];
    p2.y=pt2[1];
    p2.z=pt2[2];
    line_list.points.push_back(p1);
    line_list.points.push_back(p2);
    // line_list.lifetime=0.25;
    return line_list;
}

#define INSERT_LINKS(pts,msg,body,id)\
do\
{\
    for(int i=0;i<pts.size()-1;i++)\
    {\
        vector<float> pt1={msg->markers[pts[i]].pose.position.x,msg->markers[pts[i]].pose.position.y,msg->markers[pts[i]].pose.position.z,msg->markers[pts[i]].pose.orientation.x,msg->markers[pts[i]].pose.orientation.y,msg->markers[pts[i]].pose.orientation.z};\
        vector<float> pt2={msg->markers[pts[i+1]].pose.position.x,msg->markers[pts[i+1]].pose.position.y,msg->markers[pts[i+1]].pose.position.z,msg->markers[pts[i+1]].pose.orientation.x,msg->markers[pts[i+1]].pose.orientation.y,msg->markers[pts[i+1]].pose.orientation.z};\
        string frame_id=msg->markers[0].header.frame_id;\
        visualization_msgs::Marker line_list=getLine(pt1,pt2,frame_id,id++);\
        body.markers.push_back(line_list);\
    }\
}while(0)

void test(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    visualization_msgs::MarkerArray modified;
    if(msg->markers.size()<32)
    {
        cout<<"Corrupt points..."<<endl;
        return;
    }
    int id=0;
    vector<int> trunk={0,1,2,3};
    INSERT_LINKS(trunk,msg,modified,id);
    vector<int> left_leg={0,18,19,20,21};
    INSERT_LINKS(left_leg,msg,modified,id);
    vector<int> right_leg={0,22,23,24,25};
    INSERT_LINKS(right_leg,msg,modified,id);
    vector<int> left_hand={4,5,6,7,8};
    INSERT_LINKS(left_hand,msg,modified,id);
    vector<int> right_hand={11,12,13,14,15};
    INSERT_LINKS(right_hand,msg,modified,id);
    vector<int> neck={3,26};
    INSERT_LINKS(neck,msg,modified,id);
    vector<int> left_fingers={7,8,7,9,7,10};
    INSERT_LINKS(left_fingers,msg,modified,id);
    vector<int> right_fingers={14,16,14,15,14,17};
    INSERT_LINKS(right_fingers,msg,modified,id);

    // vector<int> non_joints={8,9,10,15,16,17,26,27,28};

   for(int i=0;i<msg->markers.size();i++)
   {
    auto joint=msg->markers[i];
    visualization_msgs::Marker modified_point=joint;
    modified_point.scale.x=0.035;
    modified_point.scale.y=0.035;
    modified_point.scale.z=0.035;
    modified_point.color.r=1;
    modified_point.color.g=0;
    modified_point.color.b=0;
    modified.markers.push_back(modified_point);
   }
    vis_pub.publish(modified);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_human_joints");
    ros::NodeHandle pnh("~");
    vis_pub = pnh.advertise<visualization_msgs::MarkerArray>( "/human_tracking_joints_and_links",0);
    ros::Subscriber array_sub = pnh.subscribe ("/body_tracking_data",1,&test);
    ros::spin();
    return 0;
}

// inline float distance_points(vector<float> pt1,vector<float> pt2)
// {
//     float dx=pt1[0]-pt2[0];
//     float dy=pt1[1]-pt2[1];
//     float dz=pt1[2]-pt2[2];
//     return sqrt(dx*dx+dy*dy+dz*dz);
// }

// inline visualization_msgs::Marker getLine(vector<float> pt1,vector<float> pt2,string frame_id,int id)
// {
//     visualization_msgs::Marker line_list;
//     line_list.header.frame_id=frame_id;
//     line_list.header.stamp = ros::Time::now();
//     line_list.pose.orientation.w = 1.0;
//     line_list.pose.position.x=pt1[0]+pt1[0]/2.0;
//     line_list.pose.position.y=pt1[1]+pt1[1]/2.0;
//     line_list.pose.position.z=pt1[2]+pt1[2]/2.0;
//     line_list.pose.orientation.x=0;
//     line_list.pose.orientation.y=0;
//     line_list.pose.orientation.z=0;

//     line_list.id=id;
//     line_list.type=visualization_msgs::Marker::CYLINDER;
//     line_list.scale.x=0.01;
//     line_list.scale.y=0.01;
//     line_list.scale.z=distance_points(pt1,pt2);
//     line_list.color.r=1;
//     line_list.color.g=0;
//     line_list.color.b=0;
//     line_list.color.a=1;
//     // line_list.lifetime=0.25;
//     return line_list;
// }

// inline visualization_msgs::Marker getLine(vector<float> pt1,vector<float> pt2,string frame_id,int id)
// {
//     visualization_msgs::Marker line_list;
//     line_list.header.frame_id=frame_id;
//     line_list.header.stamp = ros::Time::now();
//     line_list.pose.orientation.w = 1.0;
//     line_list.id=id;
//     line_list.type=visualization_msgs::Marker::LINE_LIST;
//     line_list.scale.x=0.03;
//     line_list.color.r=1;
//     line_list.color.g=0;
//     line_list.color.b=0;
//     line_list.color.a=1;
//     geometry_msgs::Point p1,p2;
//     p1.x=pt1[0];
//     p1.y=pt1[1];
//     p1.z=pt1[2];
//     p2.x=pt2[0];
//     p2.y=pt2[1];
//     p2.z=pt2[2];
//     line_list.points.push_back(p1);
//     line_list.points.push_back(p2);
//     // line_list.lifetime=0.25;
//     return line_list;
// }

