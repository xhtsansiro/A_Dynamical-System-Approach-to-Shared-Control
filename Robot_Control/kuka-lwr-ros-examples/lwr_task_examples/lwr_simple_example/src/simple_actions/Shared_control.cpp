#include "simple_actions/Shared_control.h"
#include "robot_motion_generation/angular_velocity.h"
#include <tf/transform_broadcaster.h>
#include <math.h>

#include <fstream>
#include <iostream>
#include <iomanip>


namespace simple_actions {



    Shared_control_action::Shared_control_action(ros::NodeHandle& nh):
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
    
}

bool Shared_control_action::update(){

    if(!switch_controller.activate_controller("joint_controllers"))
    {
        ROS_WARN_STREAM("failed to start controller [Joint_action::update()]!");
        return false;
    }
    ros::spinOnce();
    // re-read the data for GPR
        
    realtype pos_y, pos_z, vel_y, vel_z;   // store the value read from txt file.
    int idx = 0;
    std::ifstream read_p, read_v; 
    read_p.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/pos_train.txt");

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
    read_v.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/vel_train.txt"); 
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

    // create files to save data here
    std::ofstream outfile;
    outfile.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/shared_control/robot_desired.txt");
    if( !outfile ) { // file couldn't be opened
          std::cerr << "Error: file could not be opened" << std::endl;
          exit(1);
    }

    std::ofstream outfile2;
    outfile2.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/shared_control/robot_real.txt");
    if( !outfile2 ) { // file couldn't be opened
          std::cerr << "Error: file could not be opened" << std::endl;
          exit(1);
    }

    std::ofstream outfile3;  //save the force 
    outfile3.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/shared_control/vsds_force.txt");
    if( !outfile3 ) { // file couldn't be opened
        std::cerr << "Error: file could not be opened" << std::endl;
        exit(1);
    } 

    std::ofstream outfile4;  //save the real position of haptic device 
    outfile4.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/shared_control/hd_real.txt");
    if( !outfile4 ) { // file couldn't be opened
        std::cerr << "Error: file could not be opened" << std::endl;
        exit(1);
    } 

    std::ofstream outfile5;  // save the velocity and filtered linear velocity
    outfile5.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/shared_control/velocity.txt");
    if( !outfile5 ) { // file couldn't be opened
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
    
    // intialize the object, 
    x0 << current_origin.getY(), current_origin.getZ();
     /*Set a new object*/
    try {
        myds_profile = new MyDS(sigmascale, dt, threshold, velLimit, Br, l, sigma_f, sigma_n, attractor, x0, M, AA, train_pos, train_vel, N, last_ratio, omega_threshold);    
    }
    catch (std::bad_alloc){
        cout << "Generation of object failed, please check!" << endl;
    }
   
    myds_profile -> Map_Robot_Hd(x0, hd0);
 
    // calculate the variance, remove the tail side of the variance_rec, since they are all set to 1e-5.
    realtype sum_v = 0, num = 0;
    for(int i = 0; i < myds_profile -> variance_rec.size(); i++){
        if (myds_profile->variance_rec(i) != 1e-5){
            sum_v += myds_profile->variance_rec(i);
            num += 1;
        }
    }
  //  myds_profile -> ave_variance =  myds_profile -> variance_rec.sum() / myds_profile -> variance_rec.size(); 
    myds_profile -> ave_variance = sum_v / num; 
    myds_profile -> omega_threshold =  smooth_rise_threshold(myds_profile -> ave_variance, 0, 1, 0.5, 0.2); //upper 0.5 + 0.2, lower: 0.5 - 0.2 
    // myds_profile -> omega_threshold = 0.4;

    outfile << "threshold is: " <<  myds_profile -> omega_threshold << endl;   // save the threshold
    // get the omega_threshold based on the average variance. 0 is the lowest variance, 1 is highest variance,
    
  //    dhdEnableForce (DHD_ON);
    double HD_x, HD_y, HD_z;
    double hd_vx, hd_vy, hd_vz;  // store the velocity
    double Rob_x, Rob_y, Rob_z;
    double HD_x_init, HD_y_init, HD_z_init;
    double Rob_x_init, Rob_y_init, Rob_z_init;
    
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
    double t0, t1, t2, t3, t4;
    int c = 0;
    cout <<" Starting the movement" << endl;
  
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
        t0 = drdGetTime(); // starting point

        dhdGetPosition (&HD_x, &HD_y, &HD_z);   // get the haptic device position
        dhdGetLinearVelocity(&hd_vx, &hd_vy, &hd_vz) ;  // get the velocity of the haptic device
        h_cur << HD_y, HD_z; 
        vh_cur << hd_vy, hd_vz;
          
        outfile4 << HD_x << "  " << HD_y << "  " << HD_z << "  " ;    // real pos of haptic device
        outfile5 << hd_vy <<"  " << hd_vz << "  " << vh_cur(0) <<"  " << vh_cur(1) << endl;

        current_origin = ee_pose_current.getOrigin(); // current_orient = ee_pose_current.getRotation();
        Rob_x = current_origin.getX();
        Rob_y = current_origin.getY();
        Rob_z = current_origin.getZ();
        outfile2 << Rob_x << "  " << Rob_y << "  " << Rob_z <<endl;
            
        delta_hd(0) = HD_x - HD_x_init;
        delta_hd(1) = HD_y - HD_y_init;
        delta_hd(2) = HD_z - HD_z_init;

        delta_rob = 3.0*Rot*delta_hd;
 
          //  Rob_x = Rob_x_init + delta_rob(0); // set delta_rob(0) to zero such that there is no movement in the x-direction.
	    Rob_x = -0.419664;
        Rob_y = Rob_y_init + delta_rob(1);
        Rob_z = Rob_z_init + delta_rob(2);  // ideal position

        outfile << Rob_x << "  " << Rob_y << "  " << Rob_z <<std::endl;
        x_des(0) = Rob_y; x_des(1) = Rob_z;
        
        // set the desired position of robot.
        target_origin.setX(Rob_x);
        target_origin.setY(Rob_y);
        target_origin.setZ(Rob_z);

        std::cout<< "Haptic positions: "<<HD_x<<" "<<HD_y<<" "<< HD_z<<std::endl;
        std::cout<< "Robot positions: "<<Rob_x<<" "<<Rob_y<<" "<< Rob_z<<std::endl;

        current_origin = ee_pose_current.getOrigin();
        current_orient = ee_pose_current.getRotation();
        tmp2.setRPY(0,0,0);
        tmp1.setRotation(current_orient);
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
        // the above part is additionally added. 
            
        weighting  = myds_profile -> PosCheck(h_cur);

        // based on weighting, generate a scaling factor on the force
        //    scale_ratio = smooth_rise(weighting, 0.60, 0.75);
        cout << "Weighting:" << weighting << endl;
        outfile4 << weighting << endl;
        if (weighting < myds_profile->omega_threshold && c != 1){
            c = 1;
            dhdSetForce(0, 0, 0);
        }
            
            // check the current position of the robot, and calculate the force

            //check the current position of haptic device, and calculate the force.
        Force = myds_profile -> GetControlForce_vsds (h_cur, vh_cur, Damping);
    
        if (c == 1){
            Force(0) = 0; Force(1) = 0;
            cout << "Press any key when starting incremental learning." << endl;
        }
        outfile3 << Force(0) <<"  " << Force(1) << "  " << std::endl;
            // apply the Force, no mapping applied
        if (dhdSetForce(0, Force(0), Force(1)) < DHD_NO_ERROR) {  
            printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
            return false ;
        }

        ros::spinOnce();
        loop_rate.sleep();  // loop_rate is 500 Hz, every 0.002 seconds, 
    }

    b_reached = false;  // reset b_reached
    // entered only when escape from the guidance, and start incremental learning part
    if (c == 1){
        drdMoveToPos(0, 0, 0);  // go to initial position
        // Get the position of the current 
        current_origin  = ee_pose_current.getOrigin();
        Rob_x_init = current_origin.getX();
        Rob_y_init = current_origin.getY();
        Rob_z_init = current_origin.getZ();
        dhdGetPosition(&HD_x_init, &HD_y_init, &HD_z_init);
        x_cur << Rob_y_init, Rob_z_init;   // intialize current robot position.

        while (!b_reached && b_run){

            if ((attractor-x_cur).norm() < 0.010){ 
                b_reached  = true; 
            }

            dhdGetPosition (&HD_x, &HD_y, &HD_z);   // get the haptic device position
            // figure out the movement of haptic device
            delta_hd(0) = HD_x - HD_x_init;
            delta_hd(1) = HD_y - HD_y_init;
            delta_hd(2) = HD_z - HD_z_init;

            // calculate the position of robot
            delta_rob = 3.0*Rot*delta_hd;
 
            //  Rob_x = Rob_x_init + delta_rob(0); // set delta_rob(0) to zero such that there is no movement in the x-direction.
	        Rob_x = -0.419664;
            Rob_y = Rob_y_init + delta_rob(1);
            Rob_z = Rob_z_init + delta_rob(2);  // ideal position

             // set the desired position of robot.
            target_origin.setX(Rob_x);
            target_origin.setY(Rob_y);
            target_origin.setZ(Rob_z);

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

            // send the target position message to 
            ros_controller_interface::tf2msg(target_origin,target_orientation,ee_pos_msg);
            sendCartPose(ee_pos_msg);  //defined in ros_ee_j.h,

            // get current position of the robot   
            current_origin = ee_pose_current.getOrigin(); // current_orient = ee_pose_current.getRotation();
            x_cur << current_origin.getY(), current_origin.getZ();

            // what I am interested is the movement of x_cur, the real position of x_cur, 
            new_pos = PushBackColumn(new_pos, x_cur);
            ros::spinOnce();
            loop_rate.sleep();  // loop_rate is 500 Hz, every 0.002 seconds, 

        }
    }
    
      
    endwin();  // change back to the standard cout out

    ee_vel_msg.linear.x  = 0;
    ee_vel_msg.linear.y  = 0;
    ee_vel_msg.linear.z  = 0;
    ee_vel_msg.angular.x = 0;
    ee_vel_msg.angular.y = 0;
    ee_vel_msg.angular.z = 0;
    sendCartVel(ee_vel_msg);
            
    b_run   = false;
    outfile.close();
    outfile2.close();
    outfile3.close();
    outfile4.close();
    outfile5.close();
    dhdClose ();
  
    ROS_INFO("Arrived at target");


    // start data processing for the new traj.
    if (c == 1 ){
        std::ofstream data_pos, fil_pos, data_vel, fil_vel; 
        data_pos.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/incremental_learning/raw_position.txt");
        if( !data_pos ) { // file couldn't be opened
            std::cerr << "Error: file could not be opened" << std::endl;
            exit(1);    
        } 

        fil_pos.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/incremental_learning/filter_position.txt");
        if( !fil_pos ) { // file couldn't be opened
            std::cerr << "Error: file could not be opened" << std::endl;
            exit(1);    
        } 

        data_vel.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/incremental_learning/velocity.txt");
        if( !data_vel ) { // file couldn't be opened
            std::cerr << "Error: file could not be opened" << std::endl;
            exit(1);    
        } 

        fil_vel.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/incremental_learning/filter_velocity.txt");
        if( !fil_vel ) { // file couldn't be opened
            std::cerr << "Error: file could not be opened" << std::endl;
            exit(1);    
        } 
           for (size_t i=0; i < new_pos.cols(); i++){
            data_pos<< new_pos(0,i) <<"  "<< new_pos(1,i) << endl;
        }

        //filter the position with one-order zero phase lag butterworth filter
        Mat new_pos_filter = butterworth(new_pos.transpose()); // the input should be a row vector

        // save the filtered pos data
        for (size_t i=0; i < new_pos_filter.cols(); i++){
            fil_pos<< new_pos_filter(0,i) <<"  "<< new_pos_filter(1,i) << endl;
        }
        
        //calculate velocity based on new_pos_filter
        Mat new_vel = Mat::Zero(new_pos_filter.rows(), new_pos_filter.cols() );
        for (size_t i=0; i < new_vel.cols(); i++){
            if (i != new_vel.cols()-1){
                new_vel.col(i) = (new_pos_filter.col(i+1) - new_pos_filter.col(i)) * 500;  // multiply 500 is equal to divide 0.002
            }
            else{
                new_vel.col(i) = new_vel.col(i-1);
            }
        }

        Mat new_vel_filter = butterworth(new_vel.transpose());
        // save vel and filtered vel
        for (size_t i=0; i < new_vel.cols(); i++){
            data_vel<< new_vel(0,i) <<"  "<< new_vel(1,i) << endl;
        }

        for (size_t i=0; i < new_vel_filter.cols(); i++){
            fil_vel<< new_vel_filter(0,i) <<"  "<< new_vel_filter(1,i) << endl;
        }
        
        data_pos.close(); fil_pos.close(); data_vel.close(); fil_vel.close();

        cout << "Start incremental learning? press y for starting, press other keys to stop." << endl;
        char inc;
        cin >> inc; 
        if (inc == 'y'){
            cout << "Start incremental learning!" << endl;
            t1  =  drdGetTime();
            // segmente new_pos_filter and new_vel_filter, remove the starting part and the end part  
            Vec index = segement(new_vel_filter.transpose(), new_pos_filter.transpose()); // Get when the traj starts and when it ends
           // cout << "index are" << index.transpose() << endl;
            Mat pos_i = new_pos_filter.block(0,index(0), 2,index(1)-index(0));
            Mat vel_i = new_vel_filter.block(0,index(0), 2,index(1)-index(0)); 

            t2 = drdGetTime() ;  cout << "Segement time: " << t2 -t1 << endl;

            cout << "index " << index.transpose() << endl;
            // resample the data, the same as spline in matlab
                // 1. check the total length of the traj, figure out how many points needed to sample from this trajectory.
            realtype length = (pos_i.block(0,0,2,pos_i.cols()-1) - pos_i.block(0,1,2,pos_i.cols()-1)).colwise().norm().sum();

            realtype master_l = 0.794728217992968;  // the original demonstration length
            cout << "length is: " << length << endl;

            int N = (int) ceil(100 * length / master_l);
            if (N < 10){
                N = 10; // set minimum number of samples
            }
                // 2. use spline function to downsample the data, 
            Mat pos_f = Mat::Zero(2,N); Mat vel_f = Mat::Zero(2,N);
            cout << "Number of samples should be: " << N << endl;
            for (size_t i=0; i< pos_f.rows(); i++){
            //    pos_f.row(i) = spline(pos_i.row(i), N);
            //    vel_f.row(i) = spline(vel_i.row(i), N);
                pos_f.row(i) = spline_tk(pos_i.row(i), N);
                vel_f.row(i) = spline_tk(vel_i.row(i), N);
            }

            t3 = drdGetTime() ;  cout << "downsampling time: " << t3 -t2 << endl;


               // 3. save the downsampling data.
            std::ofstream downsample;
            downsample.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/incremental_learning/downsample.txt");
            if( !downsample ) { // file couldn't be opened
                std::cerr << "Error: file could not be opened" << std::endl;
                exit(1);    
            } 
            for (size_t i = 0; i< N; i++){
                downsample << pos_f(0,i) <<" "<< pos_f(1,i) << " " << vel_f(0,i) << " " << vel_f(1,i) << endl;
            } 
            downsample.close();
            
            // step_1: read the data from the file， the knowledge data is in train_pos and train_vel, 

            // step_2: check the distance between the new_points and the old points, loop the old points
            Mat pos_kept = Mat::Zero(2,0); Mat vel_kept = Mat::Zero(2,0);
            
            for (int i = 0; i < train_pos.cols(); i++){
                if ((pos_f.colwise() - train_pos.col(i)).colwise().norm().minCoeff() > 0.08){
                    // if the old data is 0.08m far away from the new_data, then add it in the new_list, 
                    pos_kept.conservativeResize(Eigen::NoChange, pos_kept.cols()+1);
                    vel_kept.conservativeResize(Eigen::NoChange, vel_kept.cols()+1);
                    // add the old position 
                    pos_kept.col(pos_kept.cols() - 1) = train_pos.col(i);
                    vel_kept.col(vel_kept.cols() - 1) = train_vel.col(i);

                }
            } 

            // step_3: write the old kept data, position and velocity in the corresponding txt file.
            std::ofstream update_p, update_v;
            update_p.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/pos_train.txt");
            update_v.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/vel_train.txt");
            
            for (int i = 0 ; i < pos_kept.cols(); i++){         // adapt i to start from different point
                // check if the points are close enough to the target
                update_p << pos_kept(0,i) <<"  "<< pos_kept(1,i) << endl;
                update_v << vel_kept(0,i) <<"  "<< vel_kept(1,i) << endl;
                
            }


            /*sparsity criterion check*/
            // clean the training data and add the old kept data as training data
            myds_profile -> gpmds_->ClearData();
            myds_profile -> gpmds_-> AddDataBatch(pos_kept, vel_kept);

            // check the sparsity, add the new points one by one.
            for (size_t i=0; i < vel_f.cols(); i++){
                int c1 = 0, c2 = 0; // this is to check if the point is needed to be added.
                // only the velocity is greater than 1e-3
              
                if ((attractor-pos_f.col(i)).norm() > 0.15 ) {
                    Vec v_pred = Vec::Zero(vel_f.rows());
                    v_pred = myds_profile -> GlobalDS(pos_f.col(i)) ; //check what is the prediction based on current dataset
                    Vec v_act = vel_f.col(i);
                    // compare the prediction and the actual value, if out of boundary, then will be added. 
                    if (fabs(v_pred.norm()-v_act.norm())/v_act.norm() < 0.02){
                        c1 = 1;
                    }

                    if (acos(v_act.dot(v_pred)/ (v_act.norm() * v_pred.norm())) < 0.015* pi ){
                        c2 = 1;
                    }
                 //   kk ++ ;
                    // if c1= 1 and c2 = 1, the prediction is within the range of deviation
                    if(!(c1 && c2) ) { 
              //    if(!(c1 && c2) && kk > 8) {         // exclude the starting part of the traj.
                        update_p << pos_f(0,i) <<" "<< pos_f(1,i) << endl;
                        update_v << vel_f(0,i) <<" "<< vel_f(1,i) << endl;
                        // add this data into gpmds_
                        myds_profile->gpmds_ ->AddData(pos_f.col(i), vel_f.col(i));
                    }
                }
        
            }
            
            update_p.close(); update_v.close();

            cout << "Incremental learning is done." << endl;
        }
        else {
            cout << "Discard the data for Incremental learning" << endl;
        }
       
    }
    
    ROS_INFO("Action Finished");
    delete myds_profile->gpmds_; // delete the pointer inside this object.
    delete myds_profile;   // delete the dynamic created object of this class.

    return true;
  
}

bool Shared_control_action::stop(){
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

void Shared_control_action::simple_line_policy(Eigen::Vector3d& linear_velocity,
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
