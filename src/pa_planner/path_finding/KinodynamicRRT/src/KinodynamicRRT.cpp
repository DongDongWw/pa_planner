#include <KinodynamicRRT.h>

namespace pa_planner {
    uniform_real_distribution<double>   _rand_x;
    uniform_real_distribution<double>   _rand_y;
    uniform_real_distribution<double>   _rand_z;
    random_device rd;
    default_random_engine eng(rd());

    void KinodynamicRRT::startPathSearch() {
        Eigen::Vector3d start_pos = param_.start_pos_, 
                        start_vec = param_.start_vec_, 
                        start_acc = param_.start_acc_, 
                        end_pos   = param_.end_pos_,
                        end_vec   = param_.end_vec_, 
                        end_acc   = param_.end_acc_;
        double x_min = param_.x_range.first, 
               x_max = param_.x_range.second, 
               y_min = param_.y_range.first, 
               y_max = param_.y_range.second, 
               z_min = param_.z_range.first, 
               z_max = param_.z_range.second; 
        _rand_x = uniform_real_distribution<double>(x_min,x_max);
        _rand_y = uniform_real_distribution<double>(y_min,y_max);
        _rand_z = uniform_real_distribution<double>(z_min,z_max);
       
        ros::Time   time_start = ros::Time::now();
        double      time_cost = (ros::Time::now() - time_start).toSec()*1000, //ms
                    search_limit = 10; 
        
        Eigen::Vector3d sample_pt;

        while(time_cost < search_limit) {
            sample_pt << _rand_x(eng), _rand_y(eng), _rand_z(eng);
            
        }
        

    }
    // integral model, jerk as input, only fix pos of end states
    Eigen::Matrix3d KinodynamicRRT::solveOptimalBvpFixedPos (SampleNode* last_node, SampleNode* node, Eigen::Vector3d T) {
        Eigen::Vector3d x_coef, y_coef, z_coef;
        Eigen::Matrix3d xyz_coef;
        
        // calculate an approriate duration
        double Tx = T(0); 
        double Ty = T(1);
        double Tz = T(2);
        double dp_x = node->x(0) - last_node->x(0) - 0.5*last_node->x(2)*pow(Tx,2) - last_node->x(1)*Tx;
        double dp_y = node->y(0) - last_node->y(0) - 0.5*last_node->y(2)*pow(Ty,2) - last_node->y(1)*Ty;
        double dp_z = node->z(0) - last_node->z(0) - 0.5*last_node->z(2)*pow(Tz,2) - last_node->z(1)*Tz;

        x_coef << 20/pow(Tx,5) * dp_x, -20/pow(Tx,4) * dp_x, 10/pow(Tx,3) * dp_x; 
        y_coef << 20/pow(Ty,5) * dp_y, -20/pow(Ty,4) * dp_y, 10/pow(Ty,3) * dp_y; 
        z_coef << 20/pow(Tz,5) * dp_z, -20/pow(Tz,4) * dp_z, 10/pow(Tz,3) * dp_z; 

        xyz_coef.col(0) = x_coef;
        xyz_coef.col(1) = y_coef;
        xyz_coef.col(2) = z_coef;

        return xyz_coef;
    }
    // only fix position and accelaration
    Eigen::Matrix3d KinodynamicRRT::solveOptimalBvpFixedPosVecAcc (SampleNode* last_node, SampleNode* node, Eigen::Vector3d T) {
        Eigen::Vector3d x_coef, y_coef, z_coef;
        Eigen::Matrix3d xyz_coef;
        
        // calculate an approriate duration
        double Tx = T(0); 
        double Ty = T(1);
        double Tz = T(2);
        double dp_x = node->x(0) - last_node->x(0) - 0.5*last_node->x(2)*pow(Tx,2) - last_node->x(1)*Tx;
        double dp_y = node->y(0) - last_node->y(0) - 0.5*last_node->y(2)*pow(Ty,2) - last_node->y(1)*Ty;
        double dp_z = node->z(0) - last_node->z(0) - 0.5*last_node->z(2)*pow(Tz,2) - last_node->z(1)*Tz;
        double dv_x = node->x(1) - last_node->x(1) - last_node->x(2)*Tx;
        double dv_y = node->y(1) - last_node->y(1) - last_node->y(2)*Ty;
        double dv_z = node->z(1) - last_node->z(1) - last_node->z(2)*Tz;
        double da_x = node->x(2) - last_node->x(2);
        double da_y = node->y(2) - last_node->y(2);
        double da_z = node->z(2) - last_node->z(2);
        
        x_coef <<   720/pow(Tx,5)*dp_x - 360/pow(Tx,4)*dv_x + 60/pow(Tx,3)*da_x,
                   -360/pow(Tx,4)*dp_x + 168/pow(Tx,3)*dv_x - 24/pow(Tx,2)*da_x,
                    60 /pow(Tx,3)*dp_x - 24 /pow(Tx,2)*dv_x + 3 /pow(Tx,1)*da_x;
       
        y_coef <<   720/pow(Ty,5)*dp_y - 360/pow(Ty,4)*dv_y + 60/pow(Ty,3)*da_y,
                   -360/pow(Ty,4)*dp_y + 168/pow(Ty,3)*dv_y - 24/pow(Ty,2)*da_y,
                    60 /pow(Ty,3)*dp_y - 24 /pow(Ty,2)*dv_y + 3 /pow(Ty,1)*da_y;
        
        z_coef <<   720/pow(Tz,5)*dp_z - 360/pow(Tz,4)*dv_z + 60/pow(Tz,3)*da_z,
                   -360/pow(Tz,4)*dp_z + 168/pow(Tz,3)*dv_z - 24/pow(Tz,2)*da_z,
                    60 /pow(Tz,3)*dp_z - 24 /pow(Tz,2)*dv_z + 3 /pow(Tz,1)*da_z;

        xyz_coef.col(0) = x_coef;
        xyz_coef.col(1) = y_coef;
        xyz_coef.col(2) = z_coef;

        return xyz_coef; 
    }

}