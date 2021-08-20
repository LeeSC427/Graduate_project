#include <iostream>
#include "connection/cmd_vel.h"


int main(int argc, char** argv)
{
    CMD_vel cmd;

    // CONNECTION con;

    ros::init(argc, argv, "connection");        // specify the name of node

	ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("/cmd_vel/cmd_vel", 10, &CMD_vel::CMD_VEL, &cmd);
    
    ros::Rate loop_rate(10);  

    while(ros::ok())
    {
        // con.CONNECT(joy);
        cmd.MOV();
        ros::spinOnce();  

        loop_rate.sleep();
    }
    ros::spin();

    return 0;
}