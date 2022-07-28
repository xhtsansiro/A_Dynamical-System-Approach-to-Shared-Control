#ifndef MOTIONGENERATION_H_
#define MOTIONGENERATION_H_

# include "utility.h"
# include "gp/gp_modulated_ds.h"
# include "gp/linear_velocity_fields.h"
#include "eigen3/Eigen/Dense"

using namespace special_math_functions;

class MyDS {
private:
    int K; // number of local DS
    int M; // number of data dimension
    int N; // number of data points

    int N_init;
   // Mat x_cen;  // make it public
    Vec x_len; // --> Robot/ Haptic device
    realtype pre_scale;
    Vec att;   // this is attractor, the target
   
  //  realtype th_begin;  //make it public
    realtype sigmascale;
    realtype dt;
    realtype threshold;
    realtype last_ratio;
    realtype traj_len;
    // for gp ds， shared_ptr here defined or put it in the function block
    Mat AA;  // the A matrix for linear field
    
    realtype velLimit;   // velocity limit of the linear dynamics
    realtype Br;  // the radius of the circle, which centered at the position attractor.
    realtype l;
    realtype sigma_f;
    realtype sigma_n;
    Mat training_pos_batch;  
    Mat training_vel_batch; 

public:
    MyDS(realtype sigmascale_, realtype dt_, realtype threshold_, realtype velLimit_, realtype Br_, realtype l_, realtype sigma_f_, realtype sigma_n_, Vec att_, Vec x0_, int M_, Mat AA_, Mat training_pos_batch_, Mat training_vel_batch_, int N_, realtype last_ratio_,  realtype omega_threshold_);
    realtype ave_variance;   // average of the variance along the traj, use it to determine threshold.
    Vec x0;
    Vec hd0;  // intial position of haptic device
    
    Mat x_rec;   // store the points for VSDS --> Robot/ Haptic device
    Mat x_cen;   // store the points for VSDS --> Robot/ Haptic device

    Vec variance;  // store the variance of the trajectory
    Vec variance_rec; // store the varaince of the sampled points
    realtype th_begin;
    realtype omega_threshold;  // omega_threshold 
    // use a pointer instead of direct the object
  //  GaussianProcessModulatedDS *gpmds_;
    GaussianProcessModulatedDS <realtype> *gpmds_;
 //   LinearVelocityField
 //   GaussianProcessModulatedDS <realtype> gpmds_;  // gp modulated ds object
    LinearVelocityField straightline_field;  // linear ds object, served for gpmds, 
   
    // store the trajectory
    Mat h_x;   // position of the haptic device
   // Mat h_v;   // velocity of the haptic device
   // Mat h_a;   // acceleration of the haptic device.

    Mat A;
    Mat B;
    Mat Stiffness;  // save the stiffness information of each spring.
    Vec Omega(Vec x);  //implemented
    Mat GetTempPoints(Vec x0, realtype dt, realtype threshold, bool first_gen);  // implemented
    void GetDesireTraj(Vec h0); // store the desired traj of haptic device
    // Mat GetDesiredMotion(Vec x), name it as VSDS
    Vec VSDS(Vec x);  // implemented
    void GetABMatrix();  //implemented
    realtype StartActivation(Vec x);  // implemented
    Mat GetStiffness(Vec x, realtype variance);   // implemented 
    Mat FindDampingBasis(Vec x);   // implemented
    Vec GlobalDS(Vec x);   // implemented
    void GetViaPointsReduced(Mat x_temp, Vec x0);  //implemented
    realtype PosCheck(Vec x);   // implemented, check if the point stills in tunnel region of VSDS
    void MotionRegenerate(Vec x);  // regenerate VSDS,
    Vec GetControlForce_vsds(Vec pos, Vec vel, Mat damping);  // VSDS controller, 
    Vec GetControlForce_free(); // free mode
    Vec GetControlForce_impe (Vec pos, Vec vel, int index); // open-loop impedance controller
    Vec GetControlForce_flow(Vec pos, Vec vel);  // closed-loop flow controller 
    
    realtype low_pass(realtype signal, realtype prev_filt, realtype cutt_off, realtype cycle_time); // low pass filter, 
    void Map_Robot_Hd(Vec rs, Vec hds);  // map the trajectory from robot to haptic device for VSDS construction.
    void StiffSave();  // save stiffness profile.
};

#endif // _GP_MODULATED_DS_H_
