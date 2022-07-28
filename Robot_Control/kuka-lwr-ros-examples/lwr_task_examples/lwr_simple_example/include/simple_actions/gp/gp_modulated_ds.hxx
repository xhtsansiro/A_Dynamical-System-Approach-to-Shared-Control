#include "gp_modulated_ds.h"

// define the constants
template <typename R>
const R GaussianProcessModulatedDS<R>::MIN_ANGLE = 0.001;

template <typename R>
Eigen::Matrix<R,2,1> ComputeReshapingParameters(const Eigen::Matrix<R,2,1>& act_vel, const Eigen::Matrix<R,2,1>& org_vel){
//Mat ComputeReshapingParameters(const Vec &act_vel, const Vec &org_vel){
  Eigen::Matrix<R,2,1> reshape;  // contains [theta; scaling_factor], it is a column vector
  // first, speed scaling with bias
  R kappa;
  kappa = act_vel.norm()/org_vel.norm() - 1;  // scaling factor
  // second, calculating the angle between 
  R theta; R sgn;
  theta = acos(act_vel.dot(org_vel)/ (act_vel.norm() * org_vel.norm()));  // in radian 
  // check the cross product of act_vel and org_vel,
  if (org_vel(0)*act_vel(1) - org_vel(1)*act_vel(0) >0){sgn = 1.0;}
  else {sgn = -1.0;}


  reshape(0) = sgn * theta;
  reshape(1) = kappa;

  return reshape;   // return the scaling factor, rotation angle and rotation vector
}

// give the angle, axis, and scaling factor, calculate the modulation matrix
template <typename R>  
typename GaussianProcessModulatedDS<R>::Mat GaussianProcessModulatedDS<R>::ModulationFunction(R theta, R speed_scaling){
  // auto angle = angle_axis.norm();  // the collected data is angle * rotate_vector, (rotate vector should be a unit one)
  Mat modulation_matrix;
  modulation_matrix << cos(theta), -sin(theta),
                       sin(theta), cos(theta);
  // We do not allow speed scaling to stop the motion.
  R speed_scaling_final = speed_scaling;

  if (fabs(speed_scaling_final +1) < 0.01){
    speed_scaling_final = 0;
  }

  else if (speed_scaling_final < -1.0){
    speed_scaling_final = -0.0 ;
  }
  else if (speed_scaling_final > 2.0){
    speed_scaling_final = 2;
  }
  else {;}

  speed_scaling_final += 1.0;

  // this code makes the tests fail
  // if(speed_scaling_final < 0.5){
  //   speed_scaling_final = 0.5;
  //   std::cout<<"thresholding! before: "<<speed_scaling+1.0<<" after: "<<speed_scaling_final<<std::endl;
  // }
  // // temporary hack to keep original velocity:
  // speed_scaling_final = 1.0;
  
  modulation_matrix *= speed_scaling_final;
  return modulation_matrix;
}


template <typename R>
typename GaussianProcessModulatedDS<R>::Mat GaussianProcessModulatedDS<R>::ModulationFunction(const Vec& in){
  Eigen::Matrix<R,3,1> reshape ;  // theta, k , and variance 
  reshape = gpr_->DoRegression(in);       // where is the pointer gpr defined ? 
  // Eigen::Matrix<R,3,1> aa_hat;
  R theta = reshape(0); 
  R k = reshape(1); 
  variance = reshape(2); 
  // for (size_t k=0; k<3; ++k)
  //  {
  //    aa_hat(k) = theta_hat(k);  
  //  }
  return ModulationFunction(theta,k);  // aa_hat is the angle axis, 
}


template <typename R>
void GaussianProcessModulatedDS<R>::AddData(const Vec& new_pos, const Vec& new_vel){
  auto reshaping_params = ComputeReshapingParameters(new_vel, this->original_dynamics_(new_pos));  // protected member of father class
  gpr_->AddTrainingData(new_pos, reshaping_params);
}

// batch add data stored in std::vectors 
//template <typename R>
//void GaussianProcessModulatedDS<R>::AddDataBatch(const std::vector<Vec>& new_pos, const std::vector<Vec>& new_vel){
  // create matrices and put all the data there
//  Eigen::Matrix<R,3,Eigen::Dynamic> inputs;
//  Eigen::Matrix<R,4,Eigen::Dynamic> reshaping_params;
//  reshaping_params.resize(4,new_vel.size());  // one vel corresponds to four params, 
//  inputs.resize(4,new_vel.size());
///  for (size_t k=0; k<new_pos.size(); ++k)
//    {
//      reshaping_params.col(k) = ComputeReshapingParameters(new_vel[k], this->original_dynamics_(new_pos[k]));
//      inputs.col(k) = new_pos[k];
//    } 
//  gpr_->AddTrainingDataBatch(inputs, reshaping_params);
//}

// batch add data along Eigen::Matrix columns
template <typename R>    // matrix change to 2-dimension, because it is in the plane 
void GaussianProcessModulatedDS<R>::AddDataBatch(const Eigen::Matrix<R,2,Eigen::Dynamic>& new_pos, const Eigen::Matrix<R,2,Eigen::Dynamic>& new_vel ){
  Eigen::Matrix<R,2,Eigen::Dynamic> reshaping_params;
  reshaping_params.resize(2,new_vel.cols());
  Vec v;  // additionally added
  for (size_t k=0; k<reshaping_params.cols(); ++k)
    {
      v = new_vel.col(k);   // give the column to the vector 
    //  cout << v.transpose() <<endl;
     // reshaping_params.col(k) = ComputeReshapingParameters(new_vel.col(k), this->original_dynamics_(new_pos.col(k)));
      reshaping_params.col(k) = ComputeReshapingParameters(v, this->original_dynamics_(new_pos.col(k)));
     // cout << reshaping_params.col(k).transpose() << endl;
     // cout << new_pos.col(k).transpose() << "\t" << this->original_dynamics_(new_pos.col(k)).transpose()<< endl;
    }
  gpr_->AddTrainingDataBatch(new_pos, reshaping_params);
}

