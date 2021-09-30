#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include <iostream>
#include <sstream>
#include "connection/connection.h"

//#define speed 0.25
//#define T_RPM 148
// #define speed 1

#define T_RPM 146


class CMD_vel
{
    public:
        
        int run_time = 0;
        float T_vel = 0.0f;
        float R_vel = 0.0f;
        float Track_T_vel = 0.0;
        float Track_R_vel = 0.0;
        float joy_ACT = 0;
        float joy_run = 0;
        int rpm_1 = 0;
        int rpm_2 = 0;
        CONNECTION con;

        void CMD_VEL(const geometry_msgs::Twist::ConstPtr& msg)
        {
            T_vel = msg->linear.x;
            R_vel = msg->angular.z;
            joy_ACT = msg->linear.z;
            joy_run = msg->linear.y;
            
            std::cout << "joy_ACT = " << joy_ACT << std::endl;
        }

        void TRACK_CMD(const geometry_msgs::Twist::ConstPtr& msg)
        {
            Track_T_vel = msg->linear.x;
            Track_R_vel = msg->angular.z;
            std::cout << "Track_T_vel: " << Track_T_vel << std::endl;
        }

        
        void MOV(double speed, double R_SPEED, double wheel_dist, std::string port_name, ros::NodeHandle nh)
        {
            if(joy_run == 1.0)
            {
                rpm_1 = speed * T_RPM * (Track_T_vel + Track_R_vel*1.72*R_SPEED);
                rpm_2 = speed * T_RPM * (Track_T_vel - Track_R_vel*1.72*R_SPEED);
		        std::cout << "----------------------------" << std::endl;
                std::cout << "rpm1 = " << rpm_1 << std::endl;
		        std::cout << "----------------------------" << std::endl;
		    }

            else if(joy_run != 1.0)
            {    
                if(joy_ACT == 1.0)
                {
                    rpm_1 = speed * T_RPM * (T_vel + R_vel*1.72*R_SPEED);
                    rpm_2 = speed * T_RPM * (T_vel - R_vel*1.72*R_SPEED);
                }

                else if (joy_ACT != 1.0)
                {
                    rpm_1 = 0.0;
                    rpm_2 = 0.0;
                }

            }

            con.receive_task(wheel_dist);
            con.send_task(rpm_1, rpm_2);
            con.publish(nh);

            if(run_time == 0)
            {
                run_time = con.CONNECT(port_name);
            }
        }
};
