#include <ros/ros.h>
#include <pa_planner_fsm.h>
using namespace pa_planner;


int main(int argc, char** argv) {
    ros::init(argc, argv, "pa_planner_node");
    ros::NodeHandle nh("~");
    
    PAPlannerFSM plan_fsm;
    plan_fsm.init(nh);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}