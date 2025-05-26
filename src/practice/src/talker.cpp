#include "ros/ros.h"
#include "std_msgs/Int64.h" //include Int64.msg, which is under std_msgs package, into node

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
    
    int number = 0;
    std_msgs::Int64 msg; // declare variable, called msg, a standard message in the type of Int64
    
    while(ros::ok())
    {
        msg.data = number;
        ROS_INFO("%d", msg.data);
        number++;
        loop_rate.sleep();
    }
    return 0;
}