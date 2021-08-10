#include "pointcloud_header.h"
#include "ros/ros.h"
#include <iostream>

int main(int argc, char **argv)
{
    Point_cloud pnt;

    ros::init(argc, argv, "point_track");
    ros::NodeHandle nh;

    ros::Subscriber node = nh.subscribe("scan",1000,&Point_cloud::Tracking, &pnt);

    ros::spin();

    return 0;
}