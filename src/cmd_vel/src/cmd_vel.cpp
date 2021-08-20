#include <iostream>
#include "cmd_vel/joy2cmd_vel.h"

int main(int argc, char**argv)
{
    Cmd_vel cmd;

    ros::init(argc, argv, "cmd_vel");

    ros::NodeHandle n;

    ros::Rate loop_rate(10);
    ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel/cmd_vel", 10);
    
    ros::Subscriber sub = n.subscribe("joy", 10, &Cmd_vel::joy2cmd, &cmd);

    while(ros::ok())
    {
        cmd.cmd_vel();

        cmd_vel.publish(cmd.mov);

        ros::spinOnce();

        loop_rate.sleep();
    }

    ros::spin();

    return 0;
}