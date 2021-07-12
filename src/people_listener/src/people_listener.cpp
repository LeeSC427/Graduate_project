#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

void nameCallback(const std_msgs::String::ConstPtr& msg_name)
{
    ROS_INFO("name: %s", msg_name->data.c_str());
}

void ageCallback(const std_msgs::Int32::ConstPtr& msg_age)
{
    ROS_INFO("age: %d", msg_age->data);
}

void heightCallback(const std_msgs::Int32::ConstPtr& msg_height)
{
    ROS_INFO("height: %d", msg_height->data);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"listener");

    ros::NodeHandle n;

    ros::Subscriber name_sub = n.subscribe("chatter_name",1000,nameCallback);
    ros::Subscriber age_sub = n.subscribe("chatter_age",1000,ageCallback);
    ros::Subscriber height_sub = n.subscribe("chatter_height",1000,heightCallback);
    
    ros::spin();

    return 0;
}