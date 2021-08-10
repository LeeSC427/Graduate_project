#include "ros/ros.h"
#include "pointcloud_header.h"
#include <iostream>


int main(int argc, char **argv)
{
    Point_cloud pnt_c;

    ros::init(argc,argv,"lidar_pointcloud");

    ros::NodeHandle nh;

    ros::Subscriber node = nh.subscribe("scan",1000, &Point_cloud::point_cloud, &pnt_c);

    ros::spin();

    return 0;
}