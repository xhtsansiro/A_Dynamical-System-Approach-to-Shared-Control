#ifndef _GP_MODULATED_DS_H_
#define _GP_MODULATED_DS_H_

#include "eigen3/Eigen/Dense"
#include "locally_modulated_ds.h"
#include "gaussian_process_regression.h"
#include <memory>
/** 
    \brief Implementation of GP-MDS. 
 */
template<typename realtype>
class GaussianProcessModulatedDS
  : public LocallyModulatedDS<Eigen::Matrix<realtype, 2, 1>,      // the template type of the object. must be intialize during use.
			      Eigen::Matrix<realtype, 2, 2>> {                      // original dimension is (3,1) and (3,3)

 private:
  static const realtype MIN_ANGLE;
  std::shared_ptr<GaussianProcessRegression<realtype> > gpr_;

 public:

  // Use Eigen for the locally-modulated DS types.
  typedef Eigen::Matrix<realtype, 2, 1> Vec;
  typedef Eigen::Matrix<realtype, 2, 2> Mat;
  realtype variance;  // store the variance

  // A Dynamical System is a function that maps position to velocity.
  typedef std::function<Vec(Vec)> DynamicalSystem;
  /** 
      \brief Constructor.

      @param original_dynamics a function handle for evaluating the original dynamics

  */

  GaussianProcessModulatedDS(DynamicalSystem original_dynamics)
    : LocallyModulatedDS<Vec, Mat>(original_dynamics),
    gpr_(new GaussianProcessRegression<realtype>(2, 2)) {   // input is 2 dimension, and output is also 2 dimension.
    variance = 1.0;    // intialize the variance.
    gpr_->SetHyperParams(3.2,1.0,0.02);  // l, sigma_f, sigma_n
  };

  virtual ~GaussianProcessModulatedDS() { };

  virtual Mat ModulationFunction(const Vec &in);
  Mat ModulationFunction(realtype theta,realtype speed_scaling);
    
  /** 
      \brief Add a single trianing pointa

  */
  // Mat ComputeReshapingParameters(const Vec &act_vel, const Vec &org_vel);  //declared here, added by HX

  void AddData(const Vec &new_pos, const Vec &new_vel);
  /** 

      \brief Batch add training data

  */

 // void AddDataBatch(const std::vector<Vec>& new_pos, const std::vector<Vec>& new_vel);

    /** 

      \brief Batch add training data

  */

  void AddDataBatch(const Eigen::Matrix<realtype,2,Eigen::Dynamic>& new_pos, const Eigen::Matrix<realtype,2,Eigen::Dynamic>& new_vel );

  /** 

      \brief Clear all training data

  */
  void ClearData(){
    gpr_->ClearTrainingData();
  };
  /** 

      \brief Get a pointer to GPR

  */

  std::shared_ptr<GaussianProcessRegression<realtype> > get_gpr(){return gpr_;};
    

};

#include "gp_modulated_ds.hxx"

#endif // _GP_MODULATED_DS_H_
