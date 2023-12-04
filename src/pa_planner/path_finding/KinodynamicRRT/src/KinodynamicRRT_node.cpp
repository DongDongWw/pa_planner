#include <ros/ros.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "KinodynamicRRT_node");
    ros::NodeHandle nh;
    
    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}