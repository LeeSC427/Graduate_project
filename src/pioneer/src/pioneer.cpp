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
        int joy_act = 0;
        ros::NodeHandle n;                      // first NodeHandle => initialization of the node, last NodeHandle => cleanup any resources the node was using
        ros::Publisher turtlesim_joy = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",10);
        
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
        {    
            joy_FB = msg->axes[1];
            joy_LR = msg->axes[0];
            joy_act = msg->buttons[6];

            //for (unsigned i = 0; i < msg->axes.size(); i++)
              //  ROS_INFO("Axis %d is now at position %f",i,msg->axes[i]);
        }
        void Move()
        {
            if(joy_act)
            {
                mov.linear.x = joy_FB * 0.5;
                mov.angular.z = joy_LR * 0.5;
                ROS_INFO("joy_act = 1, linear.x = %f, linear.z = %f", mov.linear.x, mov.angular.z);
            }
            else
            {
                mov.linear.x = 0.0;
                mov.angular.z = 0.0;
                ROS_INFO("joy_act = 0");

            }
            
            turtlesim_joy.publish(mov);

        }
       
    body()
    {
    }
    ~body()
    {}

};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlesim_joy");        // specify the name of node
    
    body test;
	// int count = 0;
    ros::Subscriber sub = test.n.subscribe("joy",10, &body::joyCallback, &test);

	ros::Rate loop_rate(10);                // specify a frequency that you would like to loop at. (this case = 10Hz)
	while (ros::ok())
	{

        test.Move();
		ros::spinOnce();                    // recieving callbacks
		
        loop_rate.sleep();                  // sleep for the time remaining to let us hit our 10Hz publish rate.
		// ++count;
	}
    ros::spin();
   	return 0;
}