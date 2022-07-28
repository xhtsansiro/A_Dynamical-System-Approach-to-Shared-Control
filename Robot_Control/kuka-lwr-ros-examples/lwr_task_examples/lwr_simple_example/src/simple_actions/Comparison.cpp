#include "simple_actions/Comparison.h"
#include "robot_motion_generation/angular_velocity.h"
#include <tf/transform_broadcaster.h>
#include <math.h>

#include <fstream>
#include <iostream>
#include <iomanip>


namespace simple_actions {



    Comparison_action::Comparison_action(ros::NodeHandle& nh):
    Ros_ee_j(nh),
    switch_controller(nh)
{

    path_pub = nh.advertise<nav_msgs::Path>("trajectory",2,true);  // extra added, aiming publishing the position of robot.
    linear_cddynamics   = std::unique_ptr<motion::CDDynamics>( new motion::CDDynamics(3,0.002,4) );  // the second variable is time, 
    angular_cddynamics  = std::unique_ptr<motion::CDDynamics>( new motion::CDDynamics(3,0.002,1) );

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
    
    sigmascale = 1;
    dt = 0.002;
    threshold = 0.00001;
    omega_threshold = 0.77;
    velLimit = 0.20;
    Br = 0.15;
    l = 0.001;
    sigma_f = 1;
    sigma_n = 0.1;  // it will be KXX + sigma_n * sigma_n
    last_ratio = 1;
    N = 20;

    M = 2;
    AA = -0.4 * Mat::Identity(M, M);
    Damping.resize(2,2);
    Damping << 20,0,
               0,30;   // 20 and 30, 
  //   Damping = 50 * Mat::Identity(M, M); //damping matrix  is [5,0; 0,5]
    // intialize mat, first step is resize
    train_pos.resize(2, 0);
    train_vel.resize(2, 0);
    
    // intialize the starting position
    x0.resize(M,1);  x0 << -0.0468, 0.1508;

    // intialize starting pos. of haptic device
    hd0.resize(M,1); hd0 << -0.08, -0.03;

    // initialize global attractor
    attractor.resize(M,1);  // resize it into a 2x1 vector
    attractor << 0.4, 0.1;   

    // when constructing the object, read the data from orignial file, give it to gp_points
    ifstream read_p, read_v;
    int idx = 0;
    realtype pos_y, pos_z, vel_y, vel_z;   // store the value read from txt file.
    read_p.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/backup_GP_dataset/org/pos_train.txt");
    while (!read_p.eof()){
        idx ++ ;  
        read_p >> pos_y; read_p >> pos_z;
        // save the values in the row of matrix gp_points
        train_pos.conservativeResize(Eigen::NoChange, idx); 
        train_pos.col(idx-1) << pos_y, pos_z;    // for each row
        read_p.get();
        if(read_p.peek() == '\n') break;

    }
    read_p.close();
    idx = 0;
    read_v.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/backup_GP_dataset/org/vel_train.txt"); 
    while (!read_v.eof()){
        idx ++;
        read_v >> vel_y; read_v >> vel_z; 
        // save the values in the row of matrix gp_points
        train_vel.conservativeResize(Eigen::NoChange, idx);
        train_vel.col(idx-1) << vel_y, vel_z;    // for each row
        read_v.get();
        if(read_v.peek() == '\n') break;
    }
    read_v.close();

    
}

bool Comparison_action::update(){

    if(!switch_controller.activate_controller("joint_controllers"))
    {
        ROS_WARN_STREAM("failed to start controller [Joint_action::update()]!");
        return false;
    }
    ros::spinOnce();
    
    int idx = 0;
    realtype j[DHD_MAX_DOF];  // receive joint angles
    realtype inertia[6][6];   // cartesian inertial matrix.
    // "Experiment stuffs, each participants: four trails for each mode"
    char choice; int control_type; string controller_name; 
    cout << "---------------- PLEASE Choose Control Mode ---------------------" << endl;
    cout << "N/n: no guidance-------------------------------------------------" << endl;
    cout << "I/i: open-loop impedance control---------------------------------" << endl;
    cout << "F/f: closed-loop flow control------------------------------------" << endl;
    cout << "V/v: closed-loop vsds control------------------------------------" << endl;
    cin >> choice;
    switch (choice){
        case 'N':
        case 'n':
            control_type = 1;
            controller_name = "no_controller";
            break;
        case 'I':
        case 'i':
            control_type = 2;
            controller_name = "impedance";
            break;
        case 'F':
        case 'f':
            control_type = 3;
            controller_name = "flow";
            break;
        case 'V':
        case 'v':
            control_type = 4;
            controller_name = "vsds";
            break;
        default:
            control_type = -1;
            break;

    }
    if (control_type == -1){
        cout << "Invalid control mode!" << endl;
        return 0;
    }
    string name; string trial;
    cout << "---Please enter the name of participants, and the trail number---" << endl;
    cin >> name; cin >> trial; 
    cout << name <<"  " << trial << endl;
    // string s;
    // create files to save data here
    std::ofstream outfile1;
   
    outfile1.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/user_study/" + name + "_" + controller_name + "_" + trial + "_robot_desired.txt");
    if( !outfile1 ) { // file couldn't be opened
          std::cerr << "Error: file could not be opened" << std::endl;
          exit(1);
    }

    std::ofstream outfile2;
    outfile2.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/user_study/" + name + "_" + controller_name + "_" + trial + "_robot_real.txt");
    if( !outfile2 ) { // file couldn't be opened
          std::cerr << "Error: file could not be opened" << std::endl;
          exit(1);
    }

    std::ofstream outfile3;  //save the force 
    outfile3.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/user_study/" + name + "_" + controller_name + "_" + trial + "_control_force.txt");
    if( !outfile3 ) { // file couldn't be opened
        std::cerr << "Error: file could not be opened" << std::endl;
        exit(1);
    } 

    std::ofstream outfile4;  //save the real position of haptic device 
    outfile4.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/user_study/" + name + "_" + controller_name + "_" + trial + "_hd_real.txt");
    if( !outfile4 ) { // file couldn't be opened
        std::cerr << "Error: file could not be opened" << std::endl;
        exit(1);
    } 

    std::ofstream outfile5;  // save the velocity of haptic device
    outfile5.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/user_study/" + name + "_" + controller_name + "_" + trial + "_velocity.txt");
    if( !outfile5 ) { // file couldn't be opened
        std::cerr << "Error: file could not be opened" << std::endl;
        exit(1);
    } 
    std::ofstream outfile6;  // save the velocity and filtered linear velocity
    outfile6.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/user_study/" + name + "_" + controller_name + "_" + trial + "_inertia.txt");
    if( !outfile6 ) { // file couldn't be opened
          std::cerr << "Error: file could not be opened" << std::endl;
          exit(1);
    }


    tf::Vector3     current_origin  = ee_pose_current.getOrigin();  // ee_pose_current
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
    double h_y, h_z;  // haptic device position

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

    double control_rate = 500;  // original is 500.
    ros::Rate loop_rate(control_rate);
    bool success = true;

    static tf::TransformBroadcaster br;

    motion::Vector filter_vel(3);
    filter_vel.setZero();

    linear_cddynamics->SetState(filter_vel);
    linear_cddynamics->SetDt(1.0/control_rate);  // set dt based on the frequency.

    angular_cddynamics->SetState(filter_vel);
    angular_cddynamics->SetDt(1.0/control_rate);


    transform.setOrigin(target_origin);
    transform.setRotation(target_orientation);

    ROS_INFO("starting Linear Action");
    b_run   = true;
    bSwitch = true;
    target_id = 0;
    target_id_tmp = target_id;

    /*Check the current pos of the robot and set the intial position of haptic device correspondingly*/
    if (current_origin.getY() > 0.4 && current_origin.getZ() > 0.2){
        h_y =  0.08; h_z = 0.04;
    }
    else if (current_origin.getY() > 0.4 && current_origin.getZ() < 0.2) {
        h_y = 0.08; h_z = -0.04;
    }
    else if (current_origin.getY() < 0.4 && current_origin.getZ() < 0.2) {
        h_y = -0.08; h_z = -0.04;
    }
    else {
        h_y = -0.08; h_z = 0.04;
    }

    //Haptic device initialization
    haptic_initial(h_y, h_z); 
    hd0 << h_y, h_z;  // initial position of haptic device
    // set haptic device to expert mode.
    int ss_1 = dhdEnableExpertMode();  
    if (ss_1 == 0) {
        cout << "Success set haptic device to Expert Mode !" << endl;
    }
    else{
        cout << "Fail set haptic device to Expert Mode !" << endl;
    }

    // intialize the object, 
    x0 << current_origin.getY(), current_origin.getZ();
     /*Set a new object*/
    try {
        myds_profile = new MyDS(sigmascale, dt, threshold, velLimit, Br, l, sigma_f, sigma_n, attractor, x0, M, AA, train_pos, train_vel, N, last_ratio, omega_threshold);    
    }
    catch (std::bad_alloc){
        cout << "Generation of object failed, please check!" << endl;
    }
   
    // map the slave traj. to master traj.
    myds_profile -> Map_Robot_Hd(x0, hd0); 
    myds_profile -> omega_threshold = 0.1;

  //    dhdEnableForce (DHD_ON);
    double HD_x, HD_y, HD_z;
    double hd_vx, hd_vy, hd_vz;  // store the velocity
    double Rob_x, Rob_y, Rob_z;
    double HD_x_init, HD_y_init, HD_z_init;
    double Rob_x_init, Rob_y_init, Rob_z_init;
    //    double delta_hd_x, delta_hd_y, delta_hd_z;
    Mat new_pos; // new_vel;   // used for incremental learning part to store the data
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
    dhdGetPosition(&HD_x_init, &HD_y_init, &HD_z_init);
    
    ros::Time begin = ros::Time::now();

    path_covered.header.stamp = ros::Time::now();  // addtionally added for visualization in RVIZ
    path_covered.header.frame_id = "world";  // addtionally added for visualization in RVIZ

    // toDo: before executing the operation,  simulation the robot trajectory based on GP(), GP() gives back prediction of mean and variance. 
    x0(0) = Rob_y_init;  x0(1) = Rob_z_init;  // rewrite this varibale 
    Vec x_cur = x0;  // current position of robot end effector
    Vec x_des = x0;    // desired position of robot end effector
    Vec h_cur(M) ; // current position of haptic device
    Vec vh_cur(M); // current velocity of haptic device

    Vec Force;   
    realtype weighting;
    
    myds_profile->GetABMatrix();
    myds_profile->StiffSave();    // save the stiffness file
   
    int iNumber = 0;
    initscr();
    nodelay(stdscr, TRUE);
    noecho();
    realtype scale_ratio;
    b_reached = false;

    int c = 0;
    cout <<" Starting the movement" << endl;

    double t_begin, t_end; // save execution time.
    t_begin = drdGetTime();
    while(b_run && !b_reached) {  
        
	    clear();      //this part is for directly capturing the input without press enter. 
        iNumber = getch();
        // press any key when the target is reached.
        if (iNumber != -1){
            b_reached = true;
            clear();
 	    } 
     
        // is threshold too small, too restrict here ? in pratical yes, maybe set 0.01
        if ((attractor-x_cur).norm() < 0.010){ 
            b_reached  = true; 
        }
       
        dhdGetPosition (&HD_x, &HD_y, &HD_z);   // get the haptic device position
        dhdGetLinearVelocity(&hd_vx, &hd_vy, &hd_vz) ;  // get the velocity of the haptic device
        h_cur << HD_y, HD_z; 
        vh_cur << hd_vy, hd_vz;
          
        outfile4 << HD_x << "  " << HD_y << "  " << HD_z << "  " << endl;    // real pos of haptic device
        outfile5 << hd_vx <<"  " << hd_vy <<"  " << hd_vz << endl;

        current_origin = ee_pose_current.getOrigin(); // current_orient = ee_pose_current.getRotation();
        Rob_x = current_origin.getX();
        Rob_y = current_origin.getY();
        Rob_z = current_origin.getZ();
        outfile2 << Rob_x << "  " << Rob_y << "  " << Rob_z <<endl;
            
        delta_hd(0) = HD_x - HD_x_init;
        delta_hd(1) = HD_y - HD_y_init;
        delta_hd(2) = HD_z - HD_z_init;

        delta_rob = 3.0*Rot*delta_hd;
 
	    Rob_x = -0.419664;
        Rob_y = Rob_y_init + delta_rob(1);
        Rob_z = Rob_z_init + delta_rob(2);  // ideal position

        outfile1 << Rob_x << "  " << Rob_y << "  " << Rob_z <<std::endl;
        x_des(0) = Rob_y; x_des(1) = Rob_z;
        
        // set the desired position of robot.
        target_origin.setX(Rob_x);
        target_origin.setY(Rob_y);
        target_origin.setZ(Rob_z);

        std::cout<< "Haptic positions: "<<HD_x<<" "<<HD_y<<" "<< HD_z<<std::endl;
        std::cout<< "Robot positions: "<<Rob_x<<" "<<Rob_y<<" "<< Rob_z<<std::endl;

        // Retrieve inertia matrix of the haptic device here. 
        int ss_2 = inertia_matrix_hd(j, inertia);
        if (ss_2 == 1) {
            outfile6 << inertia[0][0] << "  "<< inertia[0][1] <<"  "<< inertia[0][2] << "  "<< inertia[1][0] << "  "<< inertia[1][1] <<"  "<< inertia[1][2] << "  " << inertia[2][0] << "  "<< inertia[2][1] <<"  "<< inertia[2][2] << endl;
        }
        else{
            cout << "Failed retrieve inertia matrix" << endl;
        }

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

        if(b_position){   // change b_position to 1.
                if(bSwitch){
                    ros_controller_interface::tf2msg(target_origin,target_orientation,ee_pos_msg);
                    sendCartPose(ee_pos_msg);  //defined in ros_ee_j.h,
                }
        }
        else{
            sendCartVel(ee_vel_msg);
        }
    
        // check where the robot is after movement
 	    current_origin = ee_pose_current.getOrigin(); // current_orient = ee_pose_current.getRotation();
        x_cur << current_origin.getY(), current_origin.getZ();
        // add here, save the robot position in geometry_msgs::PoseStamped, and push it in the path 
        geometry_msgs::PoseStamped this_pose_stamped;
        // position
        this_pose_stamped.pose.position.x =  current_origin.getX();
        this_pose_stamped.pose.position.y =  current_origin.getY();
        this_pose_stamped.pose.position.z =  current_origin.getZ();
        // quaternion, not necesvh_curback(this_pose_stamped);
        path_pub.publish(path_covered);
    

        //check the current position of haptic device, and calculate the force.
        if (control_type == 1){
            // free mode
            Force = myds_profile -> GetControlForce_free();
        }
        else if (control_type == 2){
            // open-loop PD
            Force = myds_profile -> GetControlForce_impe(h_cur, vh_cur, idx);
            idx ++ ;
           // if(idx == 1) {b_reached = true;}
        }
        else if (control_type == 3){
            // closed_loop flow controller
            Force = myds_profile -> GetControlForce_flow(x_cur, vh_cur);
        }
        else{
            // VSDS controller mode
            Force = myds_profile -> GetControlForce_vsds(h_cur, vh_cur, Damping); 
        }

      
        outfile3 << Force(0) <<"  " << Force(1) << "  " << std::endl;
            // apply the Force, no mapping applied
        if (dhdSetForce(0, Force(0), Force(1)) < DHD_NO_ERROR) {  
            printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
            return false ;
        }
        //      te = drdGetTime();  // ending time  
        //      outfile4 << te-t0 << endl;

        ros::spinOnce();
        loop_rate.sleep();  // loop_rate is 500 Hz, every 0.002 seconds, 
    }
    
    endwin();  // change back to the standard cout out

    ee_vel_msg.linear.x  = 0;
    ee_vel_msg.linear.y  = 0;
    ee_vel_msg.linear.z  = 0;
    ee_vel_msg.angular.x = 0;
    ee_vel_msg.angular.y = 0;
    ee_vel_msg.angular.z = 0;
    sendCartVel(ee_vel_msg);
            
    int ss_3;
    ss_3 = dhdDisableExpertMode(); // enable the expert mode for more advanced module
    if (ss_3 == 0){
        // ROS_INFO("Disable expert mode of haptic device");
        cout << "Disable expert mode of haptic device" << endl;
    }
    else{ 
        // ROS_INFO("Cannot disable expert mode!");
        cout << "Cannot disable expert mode!" << endl;
        return false;
    } 

    b_run   = false;
    t_end = drdGetTime();
    outfile4 <<"The execution time is:" << t_end - t_begin << endl;

    outfile1.close();
    outfile2.close();
    outfile3.close();
    outfile4.close();
    outfile5.close();
    outfile6.close();

    dhdClose();
    ROS_INFO("Arrived at target");

    delete myds_profile->gpmds_; // delete the pointer inside this object.
    delete myds_profile;   // delete the dynamic created object of this class.
    return true;
  
}

bool Comparison_action::stop(){
    ee_vel_msg.linear.x  = 0;
    ee_vel_msg.linear.y  = 0;
    ee_vel_msg.linear.z  = 0;
    ee_vel_msg.angular.x = 0;
    ee_vel_msg.angular.y = 0;
    ee_vel_msg.angular.z = 0;
    sendCartVel(ee_vel_msg);
    b_run   = false;
    return true;
}

void Comparison_action::simple_line_policy(Eigen::Vector3d& linear_velocity,
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
