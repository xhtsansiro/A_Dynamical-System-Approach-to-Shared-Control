#ifndef KUKA_LWR_LFD_RECORD_H
#define KUKA_LWR_LFD_RECORD_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "eigen3/Eigen/Dense"

#include "lwr_ros_action/base_action.h"
#include "lwr_ros_interface/switch_controller.h"
#include "lwr_ros_interface/ros_ee_j.h"
#include "ncurses.h"      // for non ansi _kbhit() and _getch()

#include <std_msgs/Float64MultiArray.h>
#include <robot_motion_generation/CDDynamics.h>
#include <memory>

namespace simple_actions {


class LfD_record_action : public ros_controller_interface::Ros_ee_j, public ac::Base_action {

public:


    LfD_record_action(ros::NodeHandle& nh);

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
    Eigen::Matrix <double,2,1>  target;  // target
    Eigen::Matrix <double,2,1>  cur_pos; // current position

    nav_msgs::Path      path_covered;       // extra added 
    ros::Publisher path_pub;  // a publisher structurre
    std::list<tf::Vector3>  buffer;
   
private:

    tf::Vector3     target_origin;
    tf::Vector3     target_p1, target_p2;
    tf::Vector3     first_origin;
    tf::Quaternion  target_R_p1, target_R_p2;
    tf::Quaternion  target_orientation;

    double          dist_target;

    std::unique_ptr<motion::CDDynamics> linear_cddynamics;
    std::unique_ptr<motion::CDDynamics> angular_cddynamics;

    ros_controller_interface::Switch_controller switch_controller;


    bool bSwitch;
    int target_id;
    int target_id_tmp;

    bool bFirst;


};


}




#endif
