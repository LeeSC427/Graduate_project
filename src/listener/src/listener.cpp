#include "ros/ros.h"
#include "std_msgs/String.h"

/*
callback function
get called when new message has arrived on the chatter topic.
*/
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());       // store value of data.c_str() at msg
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"listener");

    ros::NodeHandle n;

    /*
    subscribe to the chatter topic.
    call chatterCallback() whenever new message arrives.
    queue reaches 1000 messages, start throwing away old messages as new ones arrive.

    NodeHandle::subscribe() returns a ros::Subscriber object.
    */
    ros::Subscriber sub = n.subscribe("chatter",1000,chatterCallback);

    /*
    enters a loop, calling message callbacks as fast as possible.
    exit when ros::ok() returns false.(ros::shutdown() has been called / Ctrl-C)
    */
    ros::spin();

    return 0;
}

/*
- Initialize the ROS system
- Subscribe to the chatter topic
- Spin, waiting for messages to arrive
- when a message arrives, the chatterCallback() function is called
*/
