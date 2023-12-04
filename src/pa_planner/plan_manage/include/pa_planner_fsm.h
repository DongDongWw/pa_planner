#ifndef _PA_PLANNER_FSM_H_
#define _PA_PLANNER_FSM_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Path.h>

namespace pa_planner {
    
    class PAPlannerFSM {
        private:
            enum FSM_EXEC_STATE {
                INIT,
                WAIT_TARGET,
                GEN_NEW_TRAJ,
                REPLAN_TRAJ,
                EXEC_TRAJ,
                EMERGENCY_STOP
            };
            
            FSM_EXEC_STATE  exec_state_;
            ros::Timer      exec_timer_;
            bool            have_trigger_;
            bool            have_odom_;

            ros::Subscriber target_sub_;
            ros::Subscriber odom_sub_;

            Eigen::Vector3d odom_pt_;
            Eigen::Vector3d odom_vec_;

            Eigen::Vector3d start_pt_;
            Eigen::Vector3d start_vec_;
            Eigen::Vector3d start_acc_; 

            Eigen::Vector3d end_pt_;
            Eigen::Vector3d end_vec_;
            Eigen::Vector3d end_acc_; 


            
            /*function member*/ 
            void execFSMCallback (const ros::TimerEvent &e);
            void printFSMExecState ();
            void changeFSMExecState (FSM_EXEC_STATE next_state);
            void targetCallback (const nav_msgs::PathConstPtr& msg);
            bool callMotionPlan();
       
        public:
            PAPlannerFSM(){}
            ~PAPlannerFSM(){}

            void init(ros::NodeHandle &nh);
    };
}

#endif