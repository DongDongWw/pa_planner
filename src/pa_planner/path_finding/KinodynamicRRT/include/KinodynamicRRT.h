#ifndef _KinodynamicRRT_
#define _KinodynamicRRT_

#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <Eigen/Eigen>
#include <grid_map.h>
#include <polynomial_traj.h>

namespace pa_planner {
    struct SampleNode {
        double                  time_one_segment;
        double                  time_from_start;
        double                  cost_from_start;
        Eigen::Vector3d         x, y, z;
        Eigen::Vector3d         coef_x, coef_y, coef_z;
        SampleNode*             last_node;
        vector<SampleNode*>     next_nodes;

    };
    struct RRTParam {
            typedef pair<double,double> axis_range;
 
            GridMap::Ptr        grid_map_;
            axis_range          x_range, y_range, z_range;
            Eigen::Vector3d     start_pos_, start_vec_, start_acc_;
            Eigen::Vector3d     end_pos_, end_vec_, end_acc_;
            double              neighbor_radius_;

            // dynamic feasibility constraints
            double              x_d_limit_, y_d_limit_, z_d_limit_;
            double              x_dd_limit_, y_dd_limit_, z_dd_limit_;

    };
    class KinodynamicRRT {
        private:
            RRTParam                param_; 
            SampleNode*             end_node_;           
            vector<SampleNode*>     rrt_nodes_;

            PolynomialTraj          poly_traj_;

            void startPathSearch();
            bool checkCollision();
            bool checkDynamicFeasibility();

            inline bool checkOccupancy(const Eigen::Vector3d& pos) {return (bool)param_.grid_map_->getOccupancy(pos);};

        public:
            Eigen::Matrix3d solveOptimalBvpFixedPos(SampleNode* last_node, SampleNode* node, Eigen::Vector3d T); // return the coefficient(3)
            Eigen::Matrix3d solveOptimalBvpFixedPosVecAcc(SampleNode* last_node, SampleNode* node, Eigen::Vector3d T);

    };


}




#endif