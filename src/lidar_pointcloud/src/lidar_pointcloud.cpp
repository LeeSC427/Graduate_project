#include "ros/ros.h"
#include "pointcloud_header.h"
#include <iostream>


int main(int argc, char **argv)
{

    std::string topic_name = "scan1";
    std::string name_win = "Pointcloud1";

    Point_cloud pnt_c;
    
    ros::init(argc,argv,"lidar_pointcloud");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param<std::string>("topic_name", topic_name, "scan1"); 
    nh_private.param<std::string>("name_win", name_win, "Pointcloud1");

    ros::Subscriber node = nh.subscribe(topic_name,1000, &Point_cloud::point_cloud, &pnt_c);

    tf::TransformListener tf_ira;

    ros::Rate rate(10.0);

    while(ros::ok())
    {
        tf::StampedTransform transform;

        try
        {
            tf_ira.lookupTransform("/base_link", "laser_multi",
                                        ros::Time(0), pnt_c.tf_val.transform);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        // pnt_c.tf_val.get_tf(transform);
        pnt_c.get_name(name_win);
        // pnt_c.Draw(transform);
        ros::spinOnce();
    }

    return 0;
}