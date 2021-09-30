#include "tracking_cmd_vel/cmd_vel.h"
#include <iostream>

int main(int argc, char** argv)
{
    ODOM odom;
    TARGET target;
    CMD_VEL cmd;
    double t_vel = 0.0;
    double r_vel = 10.0;

    ros::init(argc, argv, "connection");        // specify the name of node

	ros::NodeHandle n;
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = n.subscribe("/odom", 10, &ODOM::get_odom, &odom);
    ros::Subscriber tar_pos_subs = nh.subscribe("/tar_pos", 10, &TARGET::get_target, &target);

    ros::NodeHandle n_private("~");
    
    n_private.param<double>("t_vel", t_vel, 0.0);
    n_private.param<double>("r_vel", r_vel, 10.0);

    ros::Rate loop_rate(15);  

    while(ros::ok())
    {
        cmd.cmd_vel(odom,target,nh,t_vel,r_vel);

        loop_rate.sleep();
        ros::spinOnce();  
    }

    return 0;
}