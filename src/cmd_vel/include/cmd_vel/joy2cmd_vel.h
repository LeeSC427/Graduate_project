#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <iostream>
#include "sensor_msgs/Joy.h"

class Cmd_vel
{
    public:
    geometry_msgs::Twist mov;
    float joy_FB = 0.0f;
    float joy_LR = 0.0f;
    float joy_ACT = 0.0f;

    void joy2cmd(const sensor_msgs::Joy::ConstPtr& msg)
    {    
        joy_FB = msg->axes[1];
        joy_LR = msg->axes[0];
        joy_ACT = msg->buttons[6];
    }

    void cmd_vel()
        {
            if(joy_ACT == 1.0f)
            {
                mov.linear.x = joy_FB;
                mov.linear.z = joy_ACT;
                mov.angular.z = joy_LR;
            }
            else
            {
                mov.linear.x = 0.0;
                mov.linear.z = 0.0;
                mov.angular.z = 0.0;
            }
        }
};