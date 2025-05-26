#include "ros/ros.h"

int main(int argc, char ** argv){
   ros::init(argc, argv, "demo"); 
   ros::NodeHandle nh;

   while(ros::ok()){ 
      ROS_INFO("Hello World"); 
      ros::Duration(1).sleep(); 
   }
}