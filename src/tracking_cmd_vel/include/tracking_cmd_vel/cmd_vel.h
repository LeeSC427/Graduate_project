#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <cmath>

class ODOM
{
    public:
        double pos_x;
        double pos_y;
        double roll;
        double pitch;
        double yaw;

        void get_odom(const nav_msgs::Odometry::ConstPtr& msg)
        {
            pos_x = msg->pose.pose.position.x;
            pos_y = msg->pose.pose.position.y;

            tf::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
            tf::Matrix3x3 m(q);

            m.getRPY(roll, pitch, yaw);
        }

    ODOM()
    {
        pos_x = 0.0;
        pos_y = 0.0;
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
    }

    ~ODOM()
    {
    }
};

class TARGET
{
    public:
        float pos_x;
        float pos_y;

        void get_target(const nav_msgs::Odometry::ConstPtr& msg)
        {
            pos_x = msg->pose.pose.position.x;
            pos_y = msg->pose.pose.position.y;
        }

    TARGET()
    {
        pos_x = 0.0;
        pos_y = 0.0;
    }

    ~TARGET()
    {
    }
};



class CMD_VEL
{
    public:
 
        double R_vel;
        double T_vel;
        ros::Publisher cmd_pub;

        double get_angle(float x, float y)
        {
            float angle = atan2(y, x);
        
            return angle;
        }

        float distance(float x, double y)
        {
            double dist = sqrt(pow(x,2)+pow(y,2));

            return dist;
        }

        void publisher(ros::NodeHandle nh)
        {
            cmd_pub = nh.advertise<geometry_msgs::Twist>("/track_cmd", 100);
        }

        void cmd_vel(ODOM odom, TARGET target, ros::NodeHandle nh, double t_vel, double r_vel)
        {
            publisher(nh);

            geometry_msgs::Twist cmd;
            double dist = distance(target.pos_x, target.pos_y);
            double angle = get_angle(target.pos_x, target.pos_y);
            
            if(angle < -0.05)
                R_vel = angle / 3.0;

            else if(angle > 0.05)
                R_vel = angle / 3.0;

            if(dist >= 0.8)
            {
                T_vel = dist * cos(angle) / 1.5;
            }

            else if(dist < 0.8)
            {
                T_vel = 0.0;
            }

            cmd.linear.x = T_vel;
            cmd.angular.z = R_vel;

            cmd_pub.publish(cmd);

        }

    CMD_VEL()
    {
        R_vel = 0.0;
        T_vel = 0.0;
    }

    ~CMD_VEL()
    {
    }
};
