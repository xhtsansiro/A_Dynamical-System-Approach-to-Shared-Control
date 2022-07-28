#ifndef KUKA_LWR_SHARED_CONTROL_H
#define KUKA_LWR_SHARED_CONTROL_H

#include <ros/ros.h>

// #include "gaussian_process_regression.h"   // intialize an object of gpr class
#include "lwr_ros_action/base_action.h"
#include "lwr_ros_interface/switch_controller.h"
#include "lwr_ros_interface/ros_ee_j.h"
#include <nav_msgs/Path.h>   
#include <geometry_msgs/Quaternion.h>   
#include <geometry_msgs/PoseStamped.h>    
#include <std_msgs/Float64MultiArray.h>
#include <robot_motion_generation/CDDynamics.h>
#include <memory>

#include "ncurses.h"  // for non ansi_kbhit() and _getch()

#include "MotionGeneration.h"    // for vsds part

namespace simple_actions {


class Shared_control_action : public ros_controller_interface::Ros_ee_j, public ac::Base_action {   //inherit from Base_action, two methods bool update() and bool stop()

public:
    
    Shared_control_action(ros::NodeHandle& nh);  

    bool update();

    bool stop();

private:

    void simple_line_policy(Eigen::Vector3d& linear_velocity,
                                  Eigen::Vector3d& angular_velocity,
                            const    tf::Vector3&  current_origin,
                            const    tf::Quaternion& current_orient,
                            double rate);


public:

    double              loop_rate_hz;
    bool                b_reached;  // extra added if the target is reached 
    bool                b_run;
    bool                b_position;
    nav_msgs::Path      path_covered;       // extra added 
    ros::Publisher      path_pub;  // a publisher structurre
    std::list<tf::Vector3>  buffer;

    Mat AA;  // matrix A of linear dynamics
    Mat train_pos;   // here define the matrix contaning train_position,
    Mat train_vel;  // here define the matrixx containing train_velocity
    Vec  attractor;   // here define the gloabl attractor, 2x1
    Vec x0;
    Vec hd0;

    Mat Damping;  // damping matrix
    
    // for intialization an object of MyDs Class
    realtype sigmascale;
    realtype dt; 
    realtype threshold;
    realtype omega_threshold;
    realtype velLimit;
    realtype Br;
    realtype l;
    realtype sigma_f;
    realtype sigma_n;
    realtype last_ratio;
    int M;  // dimension of data
    int N;   // number of sampling points of VSDS
    MyDS *myds_profile; // a pointer of the class MyDS, must it be std::shared_ptr ?


private:

    tf::Vector3     target_origin;   //tf Vector3 represents space coordinates (x,y,z) defined in ROS documentation.
    tf::Vector3     target_p1, target_p2;
    tf::Vector3     first_origin;
    tf::Quaternion  target_R_p1, target_R_p2;  // (x,y,z,w)
    tf::Quaternion  target_orientation;

    double          dist_target;

    std::unique_ptr<motion::CDDynamics> linear_cddynamics;   //
    std::unique_ptr<motion::CDDynamics> angular_cddynamics;

    ros_controller_interface::Switch_controller switch_controller; //


    bool bSwitch;
    int target_id;
    int target_id_tmp;

    bool bFirst;


};

}






#endif
