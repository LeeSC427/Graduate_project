#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");        // specify the name of node

	ros::NodeHandle n;                      // first NodeHandle => initialization of the node, last NodeHandle => cleanup any resources the node was using

	/* 
	   publishing message of type: std_msgs/String on the topic "chatter"
	   master tell any nodes listening on chatter that we are gonna publish data on that topic.
	   1000 = size of our publishing queue.
	   NodeHandle::advertise() returns a ros::Publisher object.    
	 */
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",100);

	ros::Rate loop_rate(1000);                // specify a frequency that you would like to loop at. (this case = 10Hz)

	int count = 0;
	while (ros::ok())
		/*
		   case of ros::ok() to be false
		   - SIGINT received Ctrl+c
		   - kicked off the network by another node with the same name
		   - ros::shutdown() has been called by another part of the application
		   - all ros::NodeHandles have been destroyed
		 */
	{
		std_msgs::String msg;

		std::stringstream ss;

		ss << "hello world" << count;

		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());   // =printf()

		chatter_pub.publish(msg);           // actually broadcast the message

		ros::spinOnce();                    // recieving callbacks

		loop_rate.sleep();                  // sleep for the time remaining to let us hit our 10Hz publish rate.
		++count;
	}

	return 0;
}

/*
- Initialize the ROS system
- Advertise that we are gonna publish std_msgs/String messages on the chatter topic to the master
- Loop while publishing messages to chatter 10 times a second
*/