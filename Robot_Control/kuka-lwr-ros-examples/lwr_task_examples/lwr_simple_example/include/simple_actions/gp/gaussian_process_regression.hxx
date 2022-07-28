#ifndef GAUSSIAN_PROCESS_REGRESSION_HXX
#define GAUSSIAN_PROCESS_REGRESSION_HXX

//#include "gaussian_process_regression/gaussian_process_regression.h"

template<typename R>
GaussianProcessRegression<R>::GaussianProcessRegression(int inputDim,int outputDim)
{
  input_data_.resize(inputDim,0);   // why resize has the 0 dimension,
  output_data_.resize(outputDim,0);  // with 0, one has the number/outputDim automatically as cols???
  n_data_ = 0;
}


template<typename R>
void GaussianProcessRegression<R>::AddTrainingData(const VectorXr& newInput,const VectorXr& newOutput)
{
  n_data_++;
  if(n_data_>=input_data_.cols()){
    input_data_.conservativeResize(input_data_.rows(),n_data_);    // rows() 是数据的类型， cols()是数据的数量，每增添一个数据，相当于增添一个列。
    output_data_.conservativeResize(output_data_.rows(),n_data_);
  }
  input_data_.col(n_data_-1) = newInput;
  output_data_.col(n_data_-1) = newOutput;
  b_need_prepare_ = true;
}

// void show_dim(Eigen::MatrixXf a){
//   std::cout<<a.rows()<<" "<<a.cols()<<std::endl;
// }

template<typename R>
void GaussianProcessRegression<R>::AddTrainingDataBatch(const MatrixXr& newInput, const MatrixXr& newOutput)
{
  // sanity check of provided data
  assert(newInput.cols() == newOutput.cols());  // the number of inputs should match the number of outputs.
  // if this is the first data, just add it..
  if(n_data_ == 0){
    input_data_ = newInput;
    output_data_ = newOutput;
    n_data_ = input_data_.cols();
  }
  // if we already have data, first check dimensionaly match
  else{
    assert(input_data_.rows() == newInput.rows());
    assert(output_data_.rows() == newOutput.rows());
    size_t n_data_old = n_data_;
    n_data_ += newInput.cols();
    // resize the matrices
    if(n_data_ > input_data_.cols()){
      input_data_.conservativeResize(input_data_.rows(),n_data_);
      output_data_.conservativeResize(output_data_.rows(),n_data_);
    }
    // insert the new data using block operations
    input_data_.block(0,n_data_old,newInput.rows(),newInput.cols()) = newInput;   // starting from (0, n_data_old)  to insert the new data.
    output_data_.block(0,n_data_old,newOutput.rows(),newOutput.cols()) = newOutput;
  }
  // in any case after adding a batch of data we need to recompute decomposition (in lieu of matrix inversion)
  b_need_prepare_ = true;
}


template<typename R>
R GaussianProcessRegression<R>::SQEcovFuncD(VectorXr x1, VectorXr x2)
{
  dist = x1-x2;
  double d = dist.dot(dist);
  d = sigma_f_*exp(-1/l_scale_/2*d);  // sigma_f  * exp(-x_t * x /2/l)
  return d;
}

template<typename R>
typename GaussianProcessRegression<R>::VectorXr GaussianProcessRegression<R>::SQEcovFunc(MatrixXr x1, VectorXr x2){
  int nCol = x1.cols();
  VectorXr KXx(nCol);     // a column vector, number of rows can be changed
  for(int i=0;i<nCol;i++){
    KXx(i)=SQEcovFuncD(x1.col(i),x2);    // calculate KXx, the elements between each element in x1 and x2.
  }
  // cout << x2.transpose() <<endl;   // correct verified
  // cout << KXx <<endl;

  return KXx;   // it is a column vector 
}

template <typename R>
void GaussianProcessRegression<R>::PrepareRegression(bool force_prepare){
  if(!b_need_prepare_ & !force_prepare)
    return;

  KXX = SQEcovFunc(input_data_);
  KXX_ = KXX;
  // add measurement noise
  for(int i=0;i<KXX.cols();i++)
    KXX_(i,i) += sigma_n_*sigma_n_;  // white noise,

  b_need_prepare_ = false;
}
/*  
  alpha_.resize(output_data_.rows(),output_data_.cols());   // alpha defined in gaussian_process_regression.h 
  // pretty slow decomposition to compute
  //Eigen::FullPivLU<MatrixXr> decomposition(KXX_);
  // this is much much faster:
  Eigen::LDLT<MatrixXr> decomposition(KXX_);  // cholesky decomposition. create an object of this class. 
  for (size_t i=0; i < output_data_.rows(); ++i)
    {
      alpha_.row(i) = (decomposition.solve(output_data_.row(i).transpose())).transpose();    //what is this alpha_.row(i)
      // this output_data_ means the output of the existed collected points. 
      //  Ax = b, solve for x, x = A^-1 *b, then use this x to perform dot calculation with matrix K, 
      // in this sense, one get K * A^-1* b; 
      

    }
  b_need_prepare_ = false;
} */

// This is the right way to do it but this code should be refactored and tweaked so that the decompositon is not recomputed unless new training data has arrived. 
template <typename R>
typename GaussianProcessRegression<R>::VectorXr GaussianProcessRegression<R>::DoRegression(const VectorXr& inp,bool prepare){
  // if b_need_prepare_ is true, which means the data has been added, therefore KXX_ should be re-calculated.
 
  if(prepare || b_need_prepare_){
    PrepareRegression(); 
  }
  KXx = SQEcovFunc(input_data_,inp);  // a column vector
 // cout << KXX_. col(0) << endl;

 // cout << KXX_. col(50) << endl;

  Eigen::LDLT<MatrixXr> decomposition(KXX_);  // cholesky decomposition. create an object of this class. 
  alpha_.resize(output_data_.cols(), 1);   // alpha defined in gaussian_process_regression.h resize in (n,1) a col vector
  alpha_ = decomposition.solve(KXx);   //   (K_XX ^-1  * KXx)_t  = KxX * K_XX^-1   a col vector 
 //  Eigen::Matrix<R, output_data_.cols(), 1> alpha_n;
  // cout << alpha_ <<endl;
  VectorXr alpha_n = alpha_; 
 
  double sign;
  // alpha_n = truncate(alpha);  // implement the truncate function module in this class
  // Truncate the weights
  for (int i = 0; i < output_data_.cols(); i++){
    if (alpha_n(i) < 0){
      sign = -1;
    }
    else{
      sign = 1;
    }
    alpha_n(i) = sign * alpha_n(i);

    if (alpha_n(i) < 0.01){
      alpha_n(i) = 0;
    }
    else if (alpha_n(i) < 0.02){
      alpha_n(i) = sign * 0.5 * (1+ sin(M_PI*(alpha_n(i)- 0.01)/0.01 - M_PI/2)) * alpha_n(i);
    }
    else{
      alpha_n(i) = sign * alpha_n(i);
    }
  }
 // cout << alpha_<<endl;
  // cout <<"dsf"<<endl;
  //cout << alpha_n<<endl;

  // can return immediately if no training data has been added..
  VectorXr outp(output_data_.rows() + 1);  // the predicted velocity and variance 
  outp.setZero();
  if(n_data_==0)
    return outp;
  
   
  // KXx = SQEcovFunc(input_data_,inp);
  
  // PrepareRegression(prepare);
  //ok

  outp.setZero();
  // KXx = SQEcovFunc(input_data_,inp);
  for (size_t i=0; i < output_data_.rows(); ++i){
    outp(i) = alpha_n.dot(output_data_.row(i)); 
   
    // outp(i) = output_data_.row(i).dot(alpha_n);  
   //  outp(i) = KXx.dot(alpha_.row(i));   // this is KXx * inv(KXX + sigma_n * sigma_n) * theta ???
  }
  // outp(i) = sigma_f_ - (KXx.transpose()).dot(alpha_);   // this is the variance
  outp(output_data_.rows()) = sigma_f_ - (KXx.transpose()).dot(alpha_);  
  
  // cout << "parameters: " << outp.transpose()<<endl;
  return outp;
}


template<typename R>
void GaussianProcessRegression<R>::ClearTrainingData()
{
  input_data_.resize(input_data_.rows(),0);
  output_data_.resize(output_data_.rows(),0);
  b_need_prepare_ = true;
  n_data_ = 0;
}

template<typename R>
typename GaussianProcessRegression<R>::MatrixXr GaussianProcessRegression<R>::SQEcovFunc(MatrixXr x1){
  int nCol = x1.cols();
  MatrixXr retMat(nCol,nCol);
  for(int i=0;i<nCol;i++){
    for(int j=i;j<nCol;j++){
      retMat(i,j)=SQEcovFuncD(x1.col(i),x1.col(j));
      retMat(j,i)=retMat(i,j);
    }
  }
  return retMat;
}

template<typename R>
void GaussianProcessRegression<R>::Debug()
{
  std::cout<<"input data \n"<<input_data_<<std::endl;
  std::cout<<"output data \n"<<output_data_<<std::endl;
}

#endif /* GAUSSIAN_PROCESS_REGRESSION_HXX */
