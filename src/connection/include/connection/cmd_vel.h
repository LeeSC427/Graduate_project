#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include <iostream>
#include <sstream>
#include "connection/connection.h"

class CMD_vel
{
    public:
        
        int run_time = 0;
        float T_vel = 0.0f;
        float R_vel = 0.0f;
        int joy_ACT = 0;
        CONNECTION con;

        void CMD_VEL(const geometry_msgs::Twist::ConstPtr& msg)
        {
            T_vel = msg->linear.x;
            R_vel = msg->angular.z;
            joy_ACT = msg->linear.z;
        }

        
        void MOV()
        {
            con.receive_task();
            con.send_task(T_vel, R_vel, joy_ACT);
            
            if(run_time == 0)
            {
                run_time = con.CONNECT();
            }
        }
};
