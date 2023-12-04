#include <pa_planner_fsm.h>

namespace pa_planner{
    
    void PAPlannerFSM::init(ros::NodeHandle &nh) {

        have_trigger_   = false;
        have_odom_      = false;
        exec_state_     = INIT;    
        // call back
        exec_timer_ = nh.createTimer(ros::Duration(0.01), &PAPlannerFSM::execFSMCallback, this);
        target_sub_ = nh.subscribe("/target/manual_target", 1, &PAPlannerFSM::targetCallback, this);
    }

    void PAPlannerFSM::execFSMCallback(const ros::TimerEvent &e) {
        static int call_num;
        if (call_num == 100) {
            printFSMExecState();
            call_num = 0;
        }
        else {
            ++call_num;
        }

        switch (exec_state_) {
            case INIT: {
                if(!have_odom_ || !have_trigger_) {
                    break;
                }
                else {
                    changeFSMExecState(WAIT_TARGET);
                    break;
                }
            }
            case WAIT_TARGET: {
                if(have_trigger_) {
                    changeFSMExecState(GEN_NEW_TRAJ);
                    have_trigger_ = false;
                    break;
                }
                else {
                    break;
                }
            }
            case GEN_NEW_TRAJ: {
                start_pt_ = odom_pt_;
                start_vec_ = odom_vec_;
                start_acc_.setZero();

                bool traj_sta = callMotionPlan();
                if(!traj_sta) {
                    break;
                }
                else {
                    changeFSMExecState(EXEC_TRAJ);
                }
            }
            case REPLAN_TRAJ: {

            }
            case EXEC_TRAJ: {

            }
            case EMERGENCY_STOP: {

            }
        }
    }
    bool PAPlannerFSM::callMotionPlan(){}; 
    void PAPlannerFSM::changeFSMExecState(FSM_EXEC_STATE new_state) {
        exec_state_ = new_state;
    }

    void PAPlannerFSM::printFSMExecState() {
        static std::string state_str[7] = {"INIt", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
        std::cout << "[FSM]: state: " + state_str[int(exec_state_)] << std::endl;
    }
    void PAPlannerFSM::targetCallback(const nav_msgs::PathConstPtr& msg){};

}