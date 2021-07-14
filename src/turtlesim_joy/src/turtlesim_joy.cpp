#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <iostream>
#include <sstream>


class body
{
    public:
        geometry_msgs::Twist mov;
        float joy_FB = 0.0f;
        float joy_LR = 0.0f;
        float joy_act = 0.0f;
        float joy_trigger = 0.0f;
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
        {    
            joy_FB = msg->axes[1];
            joy_LR = msg->axes[0];
            joy_act = msg->buttons[8];
            joy_trigger = msg->axes[5];
            // for (unsigned i = 0; i < msg->axes.size(); i++)
            //     ROS_INFO("Axis %d is now at position %f",i,msg->axes[i]);
        }
    
    body()
    {

    }
    ~body()
    {}

};


int main(int argc, char **argv)
{
    body test;
	ros::init(argc, argv, "turtlesim_joy");        // specify the name of node
    ros::NodeHandle n;                      // first NodeHandle => initialization of the node, last NodeHandle => cleanup any resources the node was using

	// int count = 0;
	ros::Rate loop_rate(10);                // specify a frequency that you would like to loop at. (this case = 10Hz)
    ros::Publisher turtlesim_joy = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
    ros::Subscriber sub=n.subscribe("joy",1, &body::joyCallback, &test);

	while (ros::ok())
	{
        if(test.joy_trigger == -1.0f && test.joy_act == 1.0f)
        {
            test.mov.linear.x = 5.0*test.joy_FB;
            test.mov.angular.z = 1.0*test.joy_LR;
        }
        else
        {
            test.mov.linear.x = 0.0;
            test.mov.angular.z = 0.0;
        }
        turtlesim_joy.publish(test.mov);

		ros::spinOnce();                    // recieving callbacks
		
        loop_rate.sleep();                  // sleep for the time remaining to let us hit our 10Hz publish rate.
		// ++count;
	}
    ros::spin();
   	return 0;
}