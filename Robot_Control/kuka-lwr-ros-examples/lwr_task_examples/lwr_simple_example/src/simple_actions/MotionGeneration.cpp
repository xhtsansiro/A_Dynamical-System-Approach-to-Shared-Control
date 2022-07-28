#include "simple_actions/MotionGeneration.h"
#include "dhdc.h"
#include "drdc.h"

MyDS::MyDS(realtype sigmascale_, realtype dt_, realtype threshold_, realtype velLimit_, realtype Br_, realtype l_, realtype sigma_f_, realtype sigma_n_, Vec att_, Vec x0_, int M_,  Mat AA_, Mat training_pos_batch_, Mat training_vel_batch_, int N_, realtype last_ratio_, realtype omega_threshold_):
    x0(x0_),
    x_rec(x0_),
    sigmascale(sigmascale_),
    dt(dt_),
    threshold(threshold_),
    att(att_),
    M(M_),
    velLimit(velLimit_),
    N(N_),
    N_init(N_),
    last_ratio(last_ratio_),
    omega_threshold (omega_threshold_),
    AA(AA_),
    training_pos_batch (training_pos_batch_),
    training_vel_batch (training_vel_batch_),
    Br(Br_),
    l(l_),
    sigma_f(sigma_f_),
    sigma_n(sigma_n_),
    straightline_field(att, AA, velLimit)


{

    // starting from, implement on one's own
    gpmds_ = new GaussianProcessModulatedDS <realtype> (straightline_field); // a pointer, dynamically allocated
    gpmds_-> get_gpr()->SetHyperParams(l, sigma_f, sigma_n);
    
    gpmds_-> AddDataBatch(training_pos_batch, training_vel_batch);    // add the data, which are used for GPR

    Mat xtemp;
    xtemp = GetTempPoints(x0, dt, threshold, true);
    GetViaPointsReduced(xtemp,x0);
   
    Vec h0(2); h0 << -0.08, -0.04;
    GetDesireTraj(h0);
    cout <<" Initailization is ok" <<endl;
}

// Implemente the global DS here. have to define the class in advance 
Vec MyDS::GlobalDS (Vec x){
  
    Vec v = gpmds_ -> GetOutput(x);   // or gpmds_.Getoutput(), method Getoutput() is inherited from the father class
    // After invoking Getoutput method, the variance is also calculated and saved in a public member of 
    // the object gpmds_
    return v;
}

// Simulate the trajectory, starting from x0, end at attractor, save the variance also
Mat MyDS::GetTempPoints(Vec x0, realtype dt, realtype threshold, bool first_gen){
    Vec xnow = x0;
    Vec xd = Vec::Zero(M);  // M is data dimension, and initilaize xd as a zero vector
  //  Vec v_final = Vec::Zero(M);
    Mat x_temp; 
    realtype l_sum;
    int idx = 0;  // intialize the vector variance

    while ((xnow - att).norm() > threshold) {
        xd = GlobalDS (xnow);
        x_temp = PushBackColumn(x_temp, xnow);
        xnow = xnow + dt * xd; 
        
        variance.conservativeResize(idx+1, Eigen::NoChange);   // 
        variance(idx) = gpmds_ -> variance;
        idx ++;
    }
    x_temp = PushBackColumn(x_temp, att);  // push back the attractor
    h_x = x_temp ;  // copy to the haptic device
    variance.conservativeResize(idx+1, Eigen::NoChange); // must resize before intialzing
    variance(idx) = 1e-5;  // last point, the attractor

    l_sum = (x_temp.block(0,0,M,x_temp.cols()-1) - x_temp.block(0,1,M,x_temp.cols()-1)).colwise().norm().sum(); //(starting position, matrix size)
    std::cout <<"l_sum:" << l_sum << std::endl;
    th_begin = 0.1 * l_sum;

    if (first_gen){  
        traj_len = l_sum;
        N = (int) ceil (N_init * (l_sum/ 0.792612));   
    }
    else{
        N = (int) ceil (N_init * (l_sum/ traj_len));   
    }

    return x_temp;
}

// Get the points ,
void MyDS::GetViaPointsReduced(Mat x_temp, Vec x0) {   
    
    int temp_size = x_temp.cols();
    
    realtype dis = 0; 
    int i = 1;
    int j = 0;
    Vec xnow = x0;  // also needs to add the variance x0 in the step.
    variance_rec.conservativeResize(j+1, Eigen::NoChange);  // first resize, then initialization
    variance_rec(j) = variance(0);

    Vec len(N);
    Vec x_len_temp =  (x_temp.block(0,0,M,x_temp.cols()-1) - x_temp.block(0,1,M,x_temp.cols()-1)).colwise().norm();
    realtype l_sum = x_len_temp.sum();
    realtype len_sub = l_sum/(N+last_ratio-1);   // last_ration is 1
    for (int j=0; j<N-1; j++){
        len(j) = (j+1)*len_sub;
    }
    len(N-1) = l_sum;
    
    j = 0;
    while (i < temp_size){
        if (dis < len(j)){
            dis = dis + x_len_temp(i-1);  // plus this i-1 item, then dis satisfy the situation.
            i++;
        }
        else {
            xnow = x_temp.block(0,i-1,2,1);
            x_rec = PushBackColumn(x_rec, xnow);
            variance_rec.conservativeResize(j+2, Eigen::NoChange);  // first resize, then initialization
            variance_rec(j+1) = variance(i-1); 
            
            // come into the circle, where the original dynamics takes over, then 
            // the calcualted variance is 2, set it to a samll value, because it is 
            if ((xnow - att).norm() < 0.15) {
                variance_rec(j+1) = 1e-5; 
            }
            dis = dis + x_len_temp(i-1);
            i++;
            j++;
        }
    }
    //cout << "j:" << j << endl;
    // First resize, then give the value
    variance_rec.conservativeResize(j+2, Eigen::NoChange);
    variance_rec(j+1) = 1e-5; // last sample points should be very accurate 
    x_rec = PushBackColumn(x_rec, att);
    
  //  cout << variance_rec.transpose() <<endl;

    K = x_rec.cols()-1;   // number of sampling points - 1 
    x_cen = (x_rec.block(0,0,M,x_rec.cols()-1) + x_rec.block(0,1,M,x_rec.cols()-1))/2;  // need to be mapped to haptic device
    x_len = (x_rec.block(0,0,M,x_rec.cols()-1) - x_rec.block(0,1,M,x_rec.cols()-1)).colwise().norm();
   //  cout << "total length is: " << x_len << endl;
    x_len /= 3;   // map to haptic device, it is a row vector, added later 
    cout << "N:" << x_rec.cols() << std::endl;
    
    std::ofstream outfile;
    outfile.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/stiffness_plot/pos_field.txt");
    if( !outfile ) { // file couldn't be opened
          std::cerr << "Error: file could not be opened" << std::endl;
          exit(1);
    }

    //  save the pos field of the Robot VSDS at sampled points
    for (size_t i = 0; i < x_rec.cols(); i++){
        outfile << x_rec.coeff(0,i) << "  " << x_rec.coeff(1,i) << endl;
    }
   
    outfile.close();

}

// calculate the weigths of each spring of VSDS
Vec MyDS::Omega(Vec x){

    Vec omega = Vec::Zero(K);  // spring number: K
    Vec sigma = sigmascale * x_len;
    Vec x_t(M);

    // if input is three dimensional, one just take (y,z)
    if (x.size() ==3 && M == 2){
        x_t(0) = x(1);
        x_t(1) = x(2);
    }
    else{
        x_t = x;
    }

    for (int i=0; i < K; i++){
        omega(i) = exp(-(1/(2*sigma(i)*sigma(i)))*(x_t-x_cen.block(0,i,M,1)).transpose()*(x_t-x_cen.block(0,i,M,1)));
    }

    realtype omega_sum = omega.sum();
    omega = omega/omega_sum;
    return omega;
}

// At the beginning of the trajectory, the f_vsds mutiply a scaling factor, 
// This function defines the scaling factor
realtype MyDS::StartActivation(Vec x){

    realtype b = 0.6;
    realtype temp;
    Vec x_t(M);
    // If the input dimension is 3, then take (y,z)
    if (x.size() == 3 && M == 2){
        x_t(0) = x(1);
        x_t(1) = x(2);
    }
    else{
        x_t = x;
    }

    if ((x_t-x0).norm() > th_begin){   // change th_begin to 0.07
        return 1;
    }
    else{
        temp = asin(1-b) * ((x_t-x0).norm()/th_begin);   // change th_begin to 0.07
        return sin(temp) + b;
    }
}

// Damping basis
Mat MyDS::FindDampingBasis(Vec x){

    // two dimensional vector
    if (M == 2){
        Vec y(M);
        y << 1,
             -x(0)/(x(1)+eps());
        Mat B = Mat::Zero(2,2);
        B.col(0) = x/x.norm();
        B.col(1) = y/y.norm();
        return B;
    } 

    // three dimensional vector
    else {
        Vector3d x_ = x;
        Vector3d y(1,1,(-x(0)-x(1))/(x(2)+eps()));
        Vector3d z;
        z = x_.cross(y);  // find the third direction
        Mat B = Mat::Zero(3,3);
        B.col(0) = x/x.norm();
        B.col(1) = y/y.norm();
        B.col(2) = z/z.norm();
        return B;
    }
}

// Get the desired stiffness at each VSDS point, which is the diagonal one of 
// equation 3 in the paper
Mat MyDS::GetStiffness(Vec x, realtype variance){
    Mat K_des(M,M);
    if (M == 2) {
        realtype k11, k22;
       // k22 = 200 + 200/variance;  
       k22 = smooth_fall_stiffness(variance, 0, 0.85, 1100, 700);  // upper 1100+700, lower 1100-700, (0,0.85) for sigma_f = 1
        k11 = 250;
        K_des << k11, 0,
                 0, k22;
    }
    else{
        K_des << 20, 0, 0,
                 0, 50, 0,
                 0, 0, 100;
    }
    return K_des;
}

// VSDS part
Vec MyDS::VSDS(Vec x){
    Vec g = Omega(x);

    realtype act = StartActivation(x);

    Vec x_t(M);
    // check if it is 2D or 3D
    if (x.size() == 3 && M == 2){
        x_t(0) = x(1);
        x_t(1) = x(2);
    }
    else{
        x_t = x;
    }

    Mat fl = Mat::Zero(M,K);

    for (int i = 0; i<K; i++){
        fl.col(i) = g(i) * A.block(0, M*i, M,M) * (x_t - x_rec.block(0,i+1,M,1));
    }

    Vec fl_sum = act * fl.rowwise().sum();
    return fl_sum;
}

// Get matrix A = B * des * B_t
void MyDS::GetABMatrix(){

    Mat A_temp; 
    Mat B_temp;
    Mat Stiffness_temp;
    cout <<"the number of K: " <<K << endl;

    for (int i=0; i<K; i++){
        B_temp = FindDampingBasis(x_rec.block(0,i+1, M,1) - x_rec.block(0,i, M,1));
        Stiffness_temp = GetStiffness(x_rec.block(0,i, M,1), variance_rec(i));
        A_temp = -B_temp * Stiffness_temp * B_temp.transpose();
        
        // push back the information.
        Stiffness = PushBackColumnMat(Stiffness, Stiffness_temp);
        B = PushBackColumnMat(B, B_temp);
        A = PushBackColumnMat(A, A_temp);
    }
}

void MyDS::StiffSave(){
    std::ofstream outfile, outfile1; //, outfile2; 

    outfile.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/stiffness_plot/stiffness.txt");
    if( !outfile ) { // file couldn't be opened
          std::cerr << "Error: file could not be opened" << std::endl;
          exit(1);
    }

    outfile1.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/stiffness_plot/vel_field.txt");
    if( !outfile1 ) { // file couldn't be opened
          std::cerr << "Error: file could not be opened" << std::endl;
          exit(1);
    }

    // save the stiffness along the traj
    for (size_t i = 0; i < A.cols(); i++) {
        outfile << A.coeff(0,i) <<"  "<< A.coeff(1,i) << endl;
    }
    // save the velocity field of the VSDS at sampled points
    for (size_t i = 0; i < B.cols(); i++) {
        outfile1 << B.coeff(0,i) <<"  "<< B.coeff(1,i) << endl;
    }

    outfile.close();
    outfile1.close();
}

realtype MyDS::PosCheck(Vec x){
    Vec omega = Vec::Zero(K);
    Vec sigma = sigmascale * x_len;
    Vec x_t(M);

    if (x.size() == 3 && M ==2){
        x_t(0) = x(1);
        x_t(1) = x(2);
    }
    else{
        x_t = x;
    }
    
    for (int i=0; i < K; i++){
        omega(i) = exp(-(1/(2*sigma(i)*sigma(i)))*(x_t-x_cen.block(0,i,M,1)).transpose()*(x_t-x_cen.block(0,i,M,1)));
    }

    realtype omega_sum = omega.sum();
    return omega.maxCoeff();
    // return omega_sum;

}

void MyDS::MotionRegenerate(Vec x){
    // new starting point
    Vec x0_new(M);   
    Vec hds(M); hds(0) = -0.08; hds(1) = -0.03; 
    Vec s(M); s(0) = -0.0468; s(1) = 0.1508;
    if (x.size() == 3 && M ==2){
        x0_new(0) = x(1);
        x0_new(1) = x(2);
    }
    else{
        x0_new = x;
    }
    x0 = x0_new;
    Mat xtemp;
    Mat A_new;
    Mat B_new;

    GetViaPointsReduced(xtemp, x0);
    for (size_t i =0; i < x_rec.cols(); i++) {
        x_rec.col(i) = (x_rec.col(i) - s) * 0.33333 + hds;  // *0.1333 is equal to / 3
    }
   // x_rec.col(0) =  hds; 
    th_begin /= 3.0; 

    x_cen = (x_rec.block(0,0,M,x_rec.cols()-1) + x_rec.block(0,1,M,x_rec.cols()-1))/2;
    
  //  t0 = drdGetTime();
    A = A_new; B = B_new;
    GetABMatrix();    // Get the A, B Matrix 

}

// implement passive contoller, u_c = f_vsds - D*v, v is the velocity of haptic device.

// vsds controller mode
Vec MyDS::GetControlForce_vsds(Vec pos, Vec vel, Mat damping){
    // depends on the control type; 
    // 1: no controller, gravity compensation mode; 2: 
    Vec F(2); 

    Vec vsds = VSDS(pos);    // intialize vsds, Q, D in declaration part.
    Mat Q = FindDampingBasis(vsds) ;
    Mat D = Q * damping * Q.transpose();
    F = vsds - D*vel;
    
    return F; 
} 

// free mode
Vec MyDS::GetControlForce_free(){
    Vec F(2); 
    F(0) = 0; F(1) = 0;
    return F;
}

Vec MyDS::GetControlForce_flow(Vec pos, Vec vel){
    // implement flow_controller, F = -D(x' -x_d');
    // the position here is the position of the robot.
    Vec F(2);
    Vec x_d = GlobalDS(pos); 
    Mat Q = FindDampingBasis(x_d);
    Mat damping_base(2,2);
    damping_base << 45, 0,
                    0, 20;
    Mat D = Q * damping_base * Q.transpose();
    
    // x_d is the robot velocity, one wants control haptic device, 
    // the scale factor is 3. the velocity of haptic device is x_d/3;
    F = -D * (vel- x_d/3); 
    return F;
}

Vec MyDS::GetControlForce_impe (Vec pos, Vec vel, int index){
    // implemente the open-loop impedance controller.
    Vec F(2);
    if (index >= h_x.cols()){
        index = h_x.cols()-1;
    }
    // Vec v_des = h_v.col(index);
    Vec x_des = h_x.col(index);
    // set Kp and Kd for critically damping
    
    Vec x_d = GlobalDS(pos); 
    Mat Q = FindDampingBasis(x_d);

    Mat Kp_base(2,2);
    Mat Kd_base(2,2);
    Kp_base << 250, 0,
               0, 1800;
    Kd_base << 18, 0,
               0, 46.47;
    Mat Kp = Q * Kp_base * Q.transpose() ;
    Mat Kd = Q * Kd_base * Q.transpose() ; 

    // PD controller
    F =   - Kp * (pos - x_des) -  Kd *vel ;
    return F;
}


realtype MyDS::low_pass(realtype signal, realtype prev_filt, realtype cutt_off, realtype cycle_time){


    return   ( ( signal*cutt_off*cycle_time)+ prev_filt )/(1+ cutt_off*cycle_time) ;


}

void MyDS::Map_Robot_Hd(Vec rs, Vec hds){
    // rs is the starting position of the robot, hs is the starting position of the haptic device
    // save the pos of robot vsds traj and pos of hd vsds traj 
    
    std::ofstream outfile;  //save the real position of haptic device 
    outfile.open("/home/karldhri/catkin_ws/src/Project-Laboratory/data/shared_control/vsds.txt");
    if( !outfile ) { // file couldn't be opened
        std::cerr << "Error: file could not be opened" << std::endl;
        exit(1);
    } 

    for (size_t i =0; i < x_rec.cols(); i++) {
        outfile << x_rec.coeff(0,i) << "  " << x_rec.coeff(1,i) << "  ";  // vsds of Robot
        x_rec.col(i) = (x_rec.col(i) - rs) * 0.33333 + hds;  // *0.1333 is equal to / 3
        outfile << x_rec.coeff(0,i) << "  " << x_rec.coeff(1,i) << "  ";  // vsds of hd
        outfile << variance_rec(i,0)  << endl;   // the variance of the sampled points
      //  cout << variance_rec.rows() <<"  " << variance_rec.cols() << " sdfwer" << endl;
    }
    th_begin /= 3.0;  // map to haptic device
    x_cen = (x_rec.block(0,0,M,x_rec.cols()-1) + x_rec.block(0,1,M,x_rec.cols()-1))/2;
    // x_len_hd = x_len / 3;   // map to haptic device, it is a row vector, added later 
}

void MyDS::GetDesireTraj(Vec h0){
    Vec rs  = h_x.col(0); // starting pos of the robot traj.
    Vec a(2);
    for (size_t i =0; i < h_x.cols(); i++) {
        // map the traj.
        h_x.col(i) = (h_x.col(i) - rs) * 0.33333 + h0;  // *0.1333 is equal to / 3
    }
}