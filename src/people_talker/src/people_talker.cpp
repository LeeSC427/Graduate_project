#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <iostream>
#include <string>
#include <sstream>

class IDENTIFICATION
{
    public:
        int age;
        int height;
        std::string name;
};

int main(int argc, char **argv)
{
    IDENTIFICATION id;
    
    std::cout << "name: ";
    std::cin >> id.name;
    std::cout << "age: " ;
    std::cin >> id.age;
    std::cout << "height: " ;
    std::cin >> id.height;
	
    ros::init(argc, argv, "people_talker");

	ros::NodeHandle n;

	ros::Publisher name_pub = n.advertise<std_msgs::String>("chatter_name",1000);
    ros::Publisher age_pub = n.advertise<std_msgs::Int32>("chatter_age",1000);
    ros::Publisher height_pub = n.advertise<std_msgs::Int32>("chatter_height",1000);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{
		std_msgs::String msg_name;
        std_msgs::Int32 msg_age;
        std_msgs::Int32 msg_height;

		msg_name.data = id.name;
        msg_age.data = id.age;
        msg_height.data = id.height;

		ROS_INFO("\nname: %s\nage: %d\nheight: %d", msg_name.data.c_str(), msg_age.data, msg_height.data);

		name_pub.publish(msg_name);
        age_pub.publish(msg_age);
        height_pub.publish(msg_height);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}