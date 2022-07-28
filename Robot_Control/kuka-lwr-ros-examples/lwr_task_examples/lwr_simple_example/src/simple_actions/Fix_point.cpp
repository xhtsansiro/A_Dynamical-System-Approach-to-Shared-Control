#include "simple_actions/Fix_point.h"
#include "robot_motion_generation/angular_velocity.h"
#include <tf/transform_broadcaster.h>
#include <math.h>

#include <fstream>
#include <iostream>
#include <iomanip>


namespace simple_actions {



    Fix_point_action::Fix_point_action(ros::NodeHandle& nh):
    Ros_ee_j(nh),
    switch_controller(nh)
{

    linear_cddynamics   = std::unique_ptr<motion::CDDynamics>( new motion::CDDynamics(3,0.01,4) );
    angular_cddynamics  = std::unique_ptr<motion::CDDynamics>( new motion::CDDynamics(3,0.01,1) );

    motion::Vector velLimits(3);
    for(std::size_t i = 0; i < 3; i++){
        velLimits(i)  = 2; // x ms^-1
    }
    linear_cddynamics->SetVelocityLimits(velLimits);

    for(std::size_t i = 0; i < 3; i++){
        velLimits(i)  = 0.04; // x ms^-1
    }
    angular_cddynamics->SetVelocityLimits(velLimits);

    b_reached   = false;
    b_run       = false;
    b_position  = false;
        
    bFirst		= false;
    bSwitch		= false;

    target_id = 0;
    target_id_tmp = 0;
    dist_target = 0;

    loop_rate_hz = 100;

}

bool Fix_point_action::update(){

    Vec attractor(2); 
    // attractor << 0.6755, 0.2442;  // original 
    attractor << 0.5949, 0.177739; // where escape from the guidance, where to start incremental learning


    Vec x_cur(2); 

    if(!switch_controller.activate_controller("joint_controllers"))
    {
        ROS_WARN_STREAM("failed to start controller [Joint_action::update()]!");
        return false;
    }
    ros::spinOnce();

    tf::Vector3     current_origin  = ee_pose_current.getOrigin();
    x_cur << current_origin.getY(), current_origin.getZ(); 
   // std::cout <<"xxxx: " << current_origin.getX() << "  xx " <<  current_origin.getY() <<"  " << current_origin.getZ() <<std::endl;

    tf::Quaternion  current_orient  = ee_pose_current.getRotation();

    static tf::TransformBroadcaster br1;
    tf::Transform transform;

    transform.setOrigin(current_origin);
    transform.setRotation(current_orient);
    br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "x_"));

    //first_origin        = current_origin+ tf::Vector3(0.1,0.1,0.1);
    target_p1           = first_origin + tf::Vector3(0,0.1,0);
    target_p2           = first_origin - tf::Vector3(0,0.1,0);
    //std::cout<<(first_origin==tf::Vector3(0.3,0.4,0.35))<<std::endl;
    tf::Matrix3x3 tmp1,tmp2;
    double roll, pitch, yaw;


    tmp2.setRPY(M_PI/10,0,0);
    tmp1.setRotation(current_orient);
    tmp1 = tmp2 * tmp1;
    tmp1.getRPY(roll,pitch,yaw);
    target_R_p1.setRPY(roll,pitch,yaw);

    tmp2.setRPY(-M_PI/10,0,0);
    tmp1.setRotation(current_orient);
    tmp1 = tmp2 * tmp1;
    tmp1.getRPY(roll,pitch,yaw);
    target_R_p2.setRPY(roll,pitch,yaw);

    target_origin       = target_p1;
    target_orientation  = target_R_p1;

    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d angular_velocity;

    double control_rate = 500;
    ros::Rate loop_rate(control_rate);
    bool success = true;

    static tf::TransformBroadcaster br;

    motion::Vector filter_vel(3);
    filter_vel.setZero();

    linear_cddynamics->SetState(filter_vel);
    linear_cddynamics->SetDt(1.0/control_rate);

    angular_cddynamics->SetState(filter_vel);
    angular_cddynamics->SetDt(1.0/control_rate);

    transform.setOrigin(target_origin);
    transform.setRotation(target_orientation);

    ROS_INFO("Move Robot to a pre-defined position");
    b_run   = true;
    bSwitch = true;
    target_id = 0;
    target_id_tmp = target_id;
 
    double Rob_x, Rob_y, Rob_z;

    double Rob_x_init, Rob_y_init, Rob_z_init;

    Eigen::Vector3d delta_hd;
    Eigen::Vector3d delta_rob;
    Eigen::Matrix3d Rot;
    Rot << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;

    //initial positions

    Rob_x_init = current_origin.getX();
    Rob_y_init = current_origin.getY();
    Rob_z_init = current_origin.getZ();
   

    ros::Time begin = ros::Time::now();

    b_reached = false;
    while(b_run && !b_reached) {
        
        if ((attractor-x_cur).norm() < 0.0010){ 
            b_reached  = true; 
        }

        current_origin = ee_pose_current.getOrigin(); // current_orient = ee_pose_current.getRotation();
        Rob_x = current_origin.getX();
        Rob_y = current_origin.getY();
        Rob_z = current_origin.getZ();

    
        Rob_x = -0.419664; Rob_y = 0.6755; Rob_z = 0.2442;  // for incremental learning, starting from another pos.
        // Rob_x = -0.419664; Rob_y = 0.4; Rob_z = 0.4;  // for user study.
       // Rob_x = -0.419664; Rob_y = 0.5949; Rob_z = 0.177739;   // for avoiding pull force

        target_origin.setX(Rob_x);
        target_origin.setY(Rob_y);
        target_origin.setZ(Rob_z);

        std::cout<< "Robot positions: "<<Rob_x<<" "<<x_cur(0)<<" "<< x_cur(1)<<std::endl;

        current_origin = ee_pose_current.getOrigin();
        current_orient = ee_pose_current.getRotation();
            //tmp2.setRPY(M_PI/10,0,0);
        tmp2.setRPY(0,0,0);
           
        tmp1.setRotation(current_orient);
            //tmp1 = tmp2 * tmp1;
        tmp1.getRPY(roll,pitch,yaw);
        target_R_p1.setRPY(roll,pitch,yaw);
        target_orientation  = target_R_p1;
        transform.setOrigin(target_origin);
        transform.setRotation(target_orientation);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "target"));


        transform.setOrigin(current_origin);
        transform.setRotation(current_orient);
        br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "x_"));

        simple_line_policy(linear_velocity,angular_velocity,current_origin,current_orient,control_rate);
     
            // check where the robot is after movement
 	    current_origin = ee_pose_current.getOrigin(); // current_orient = ee_pose_current.getRotation();
        x_cur(0) = current_origin.getY(); x_cur(1) = current_origin.getZ();
                
     //   std::cout <<"b_position:" << b_position << std::endl;

        if(b_position) // change b_position to 1
        {
            if(bSwitch){
                ros_controller_interface::tf2msg(target_origin,target_orientation,ee_pos_msg);
                sendCartPose(ee_pos_msg);
            }
        }
        else{
                sendCartVel(ee_vel_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
     
    
    	ee_vel_msg.linear.x  = 0;
        ee_vel_msg.linear.y  = 0;
        ee_vel_msg.linear.z  = 0;
        ee_vel_msg.angular.x = 0;
        ee_vel_msg.angular.y = 0;
        ee_vel_msg.angular.z = 0;
        sendCartVel(ee_vel_msg);
            
        b_run   = false;
    

	    ROS_INFO("Arrived at the desired initial position");
        return true; 
}

bool Fix_point_action::stop(){
    ee_vel_msg.linear.x  = 0;
    ee_vel_msg.linear.y  = 0;
    ee_vel_msg.linear.z  = 0;
    ee_vel_msg.angular.x = 0;
    ee_vel_msg.angular.y = 0;
    ee_vel_msg.angular.z = 0;
    sendCartVel(ee_vel_msg);
    b_run   = false;
//  dhdClose();
//  std::cout << "hhhhhhhhhhhhhhhhhhhhhhhhhhhhhh" <<std::endl;
    return true;
}

void Fix_point_action::simple_line_policy(Eigen::Vector3d& linear_velocity,
                                            Eigen::Vector3d& angular_velocity,
                                            const tf::Vector3 &current_origin,
                                            const tf::Quaternion &current_orient,
                                            double rate)
{

   tf::Vector3 velocity = (target_origin - current_origin);
              velocity  = (velocity.normalize()) * 0.05; // 0.05 ms^-1

     linear_velocity(0) = velocity.x();
     linear_velocity(1) = velocity.y();
     linear_velocity(2) = velocity.z();

     tf::Quaternion qdiff =  target_orientation - current_orient;
     Eigen::Quaternion<double>  dq (qdiff.getW(),qdiff.getX(),qdiff.getY(),qdiff.getZ());
     Eigen::Quaternion<double>   q(current_orient.getW(),current_orient.getX(),current_orient.getY(), current_orient.getZ());

     angular_velocity   = motion::d2qw<double>(q,dq);
     dist_target = (current_origin - target_origin).length();
     ROS_INFO_STREAM_THROTTLE(1.0,"distance: " << dist_target);

}

}
