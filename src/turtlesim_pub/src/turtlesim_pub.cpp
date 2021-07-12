#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include <cstring>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlesim_pub");        // specify the name of node

	ros::NodeHandle n;                      // first NodeHandle => initialization of the node, last NodeHandle => cleanup any resources the node was using

	ros::Publisher turtlesim_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",100);

	ros::Rate loop_rate(1);                // specify a frequency that you would like to loop at. (this case = 10Hz)

	char input[0];
	// int count = 0;
	while (ros::ok())
	{
		std::cin >> input;
		geometry_msgs::Twist mov;

		if(strcmp(input,"w")==0)
			mov.linear.x = 1.0;
		else if(strcmp(input,"a")==0)
            mov.angular.z = 1.0;
		else if(strcmp(input,"s")==0)
			mov.linear.x = -1.0;
		else if(strcmp(input,"d")==0)
            mov.angular.z = -1.0;
	    
		turtlesim_pub.publish(mov);
		ros::spinOnce();                    // recieving callbacks
		loop_rate.sleep();                  // sleep for the time remaining to let us hit our 10Hz publish rate.
		// ++count;
	}

	return 0;

}