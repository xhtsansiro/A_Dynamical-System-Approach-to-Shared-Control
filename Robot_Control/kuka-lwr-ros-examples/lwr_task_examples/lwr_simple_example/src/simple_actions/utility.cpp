#include "iostream"
#include "simple_actions/utility.h"

namespace special_math_functions {
	Mat Skew(const Vec &v) {
		if (v.size() == 1) {
			Mat out = Mat::Zero(2, 2);

			out << 0, -v(0),
				v(0), 0;
			return out;

		}
		else
		{
			Mat out = Mat::Zero(3, 3);
			out << 0, -v(2), v(1),
				v(2), 0, -v(0),
				-v(1), v(0), 0;
			return out;
		}


	}


    Eigen::Quaterniond quatMult(Eigen::Quaterniond q1, Eigen::Quaterniond q2) {
        Eigen::Quaterniond resultQ;
        resultQ.setIdentity();

        resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
        resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

        return resultQ;
    }


    Vec quatlog(Eigen::Quaterniond q1){

        if(q1.vec() != Vec::Zero(3)){

            return acos(q1.w()) * (q1.vec()/q1.vec().norm()  ) ;

        }

       return  Vec::Zero(3) ;

        }

    Mat PushBackColumn(Mat m, Vec x)
    {
        if (m.cols() == 0){
            m = x;
        }
        else{
            m.conservativeResize(Eigen::NoChange, m.cols()+1);
            m.col(m.cols()-1) = x;
        }
        return m;
    }

    Mat PushBackColumnMat(Mat m, Mat x)
    {
        if (m.cols() == 0){
            m = x;
        }
        else{
            int n = x.cols();
            m.conservativeResize(Eigen::NoChange, m.cols()+n);
            for (int i=0; i<n; i++)
            {
                m.col(m.cols()-n+i) = x.col(i);
            }
        }
        return m;
    }

    realtype eps()
    {
        return 0.00000000000000001;
    }

    realtype smooth_rise_threshold(realtype value, realtype lo, realtype hi, realtype v1, realtype v2)
    {
	if (value >= hi){
	    return v1+v2;
	}
	else if (value <= lo){
	    return v1-v2;        
	}
	else{
           realtype T = 2*(hi- lo);
           return v1 + v2*sin(2*pi*(value-lo)/T - pi*0.5);
        }
	
    }

    realtype smooth_fall_stiffness(realtype value, realtype lo, realtype hi, realtype v1, realtype v2)
    {
	if (value >= hi){
	    return v1-v2;
	}
	else if (value <= lo){
	    return v1+v2;        
	}
	else{
           realtype T = 2*(hi- lo);
           return v1 - v2*sin(2*pi*(value-lo)/T - pi*0.5);
        }
    }

}

namespace StiffnessProfiles {
	 

	Mat StiffnessProfile_MainTask(realtype t) {

		//temp 
		realtype k = 1*GeneralFunction::smooth_transition_rising(t, 4.0, 0, 600);

		return  Mat::Identity(3, 3)*k;


	}

	Mat StiffnessProfile_NullSpace(realtype t) {

		//	realtype k = 10* t;
        realtype k = 1 * GeneralFunction::smooth_transition_rising(t, 4.0, 0, 16);

		return  Mat::Identity(7, 7)*k;


	}

}

namespace GeneralFunction {

using namespace std;

	void Write_To_File(string file_name, vector<vector<float> > Input_Data) {
		ofstream myfile;
		myfile.open(file_name);

        if (!myfile)
        {
            cout << "No file found: " << file_name << endl;
            return;
        }

		for (int i = 0; i < Input_Data.size(); i++)
		{
			for (int j = 0; j < Input_Data[i].size(); j++)
			{

				if (j == Input_Data[i].size() - 1) {
					myfile << Input_Data[i][j] << endl;



				}
				else {

                    myfile << Input_Data[i][j] << " ";
				}


			}
		}
		myfile.close();
	}

	realtype smooth_transition_rising(realtype t, realtype t_max, realtype t_min, realtype scale){

        realtype alpha_min = 0.0;
        realtype alpha;

        if (t>t_max)
            alpha = 0;
        else if(t<t_min)
            alpha = 1;
        else
            alpha = (0.5*(1 + cos(pi / (t_max - t_min)*(t - t_min)))*(1 - alpha_min) + alpha_min);


         alpha = 1 - alpha;

        return  scale*alpha;

	}

    realtype smooth_transition_fall(realtype E, realtype E_max, realtype E_min){
        realtype alpha_min = 0.0;
        realtype alpha;

        if (E>E_max)
            alpha = 0;
        else if(E<E_min)
            alpha = 1;
        else
            alpha = (0.5*(1 + cos(pi / (E_max - E_min)*(E - E_min)))*(1 - alpha_min) + alpha_min);


        return  alpha;
	}
	
    void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
    {
        unsigned int numRows = matrix.rows();
        unsigned int numCols = matrix.cols()-1;

        if( colToRemove < numCols )
            matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

        matrix.conservativeResize(numRows,numCols);
    }

    realtype mod(realtype x, realtype y){
        return x - floor(x/y)*y ;
    }

    void  Vec2double(Vec a,double q[]) {

        int s = a.size();
        for (int i = 0; i < s; i++) {

            q[i] = a(i);
        }

    }

//    Vec double2Vec(double q[]) {
//        int s = (sizeof(q) / sizeof(*q));
//        Vec out = Vec::Zero(s);
//        for (int i = 0; i < s; i++) {

//            out[i] = q[i];
//        }
//        return out;

//    }

}

bool haptic_initial(double y, double z){
    if (drdOpen() < 0) {
        printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
        dhdSleep (2.0);
        return false;
    }

    if (!drdIsInitialized() && (drdAutoInit() < 0)) {
        printf ("error: initialization failed (%s)\n", dhdErrorGetLastStr ());
        dhdSleep (2.0);
        return false;
    }
    if (drdStart() < 0) {
        printf ("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr ());
        dhdSleep (2.0);
        return false;
    }

    drdMoveToPos( 0, y, 0) ;  // -0.08
    drdMoveToPos( 0, y, z);  // -0.08, -0.03
   // if(y == -0.08 && z == 0.03){
   //	 drdMoveToPos( -0.05, y, z); 
   //  }
 
    drdSleep (1.0);
    drdStop( );
    dhdOpen() ;
    dhdEnableForce (DHD_ON);
    dhdSetForce(0,0,0) ;
    drdSleep (3.0);
    return true;
	
}

// butterworth filter, first order zero phase lag
Mat butterworth(Mat input){
	// implemente first order butterworth filter, with cut off freq, [B,A] = butter(1, 3/(fs/2)), fs = 500Hz
     // input is a matrix () num_samples x 2
     
     Vec B(2); // Mat input = data;
     
     Mat Forward = Mat::Zero(input.rows(), input.cols());  // zero matrix, which stores result of forward processing 
     Mat Result = Mat::Zero(input.rows(), input.cols()) ;  // store the final result
     double A = -0.962994050950215;
     B << 0.018502974524892, 0.018502974524892;  // coefficients got from matlab
     double z0 = 0.981497025475108;  // initial conditions taken from matlab
     size_t nfilt = B.rows();
     size_t nfact = 3*(nfilt-1);     // max(1, 3(nfilt-1)) 
	

     // xt = 2*pos(1,:) - pos(nfact+1:-1:2,:)     
     Mat xt(nfact, 2);  
     for(size_t i =0; i< nfact; i++){
	    xt.row(i) = 2*input.row(0) - input.row(nfact-i);
     } 
     
     // [~,zo] = filter(b(:,ii),a(:,ii),xt,zi(:,ii)*xt(1,:)); % outer product 
     Vec z1 = z0 * (xt.row(0).transpose()); double x,y,z;
     Vec z2 = z1;
     for (size_t i = 0; i < input.cols(); i++) {         //input.cols should be 2
	    z = z1(i); 
        for (size_t j = 0; j < xt.rows(); j++){
            x = xt(j,i);
            y = B(0)* x + z;     // calculate output
            z = B(1)* x - A * y;   // calculate internal state
        }
        z2(i) = z;   // calculate the new state z
     }
     
     // this is forward processing
     // [yc2,zo] = filter(b(:,ii),a(:,ii),xc,zo); z2 are used as initial condition.
      Vec z3 = z2;
      for (size_t i = 0; i < input.cols(); i++) {         //input.cols should be 2
	    z = z2(i); 
        for (size_t j = 0; j < input.rows(); j++){
            x = input(j,i);
            y = B(0)* x + z;     // calculate output
            z = B(1)* x - A * y;   // calculate internal state
            Forward(j,i) = y;    // store result of forward processing.
        }
        z3(i) = z;   // calculate the new state z
     }
     
     // xt = bsxfun(@minus, 2*xc(end,:), xc(end-1:-1:end-nfact(1,1),:));
     //   yc3 = filter(b(:,ii),a(:,ii),xt,zo);  
     // calculate xt;
     // xt = 2*pos(end,:) - pos(end-1:-1:end-nfact,:)     
      // Mat xt2(nfact, 2);
     for(size_t i =0; i< nfact; i++){
	    xt.row(i) = 2*input.row(input.rows()-1) - input.row(input.rows()-2 -i);
     } 
     Mat yc3 = Mat::Zero(xt.rows(), xt.cols());

     for (size_t i = 0; i < input.cols(); i++) {         //input.cols should be 2
	    z = z3(i); 
        for (size_t j = 0; j < xt.rows(); j++){
            x = xt(j,i);
            y = B(0)* x + z;     // calculate output
            z = B(1)* x - A * y;   // calculate internal state
            yc3(j,i) =y;     // calculate yc3
        }
      //  z2(i) = z;   // calculate the new state z
     }

     // [~,zo] = filter(b(:,ii),a(:,ii),yc3(end:-1:1,:),zi(:,ii)*yc3(end,:));
     Vec z4 = z0 * (yc3.row(yc3.rows()-1).transpose());
     Vec z5 = z4;
     for (size_t i = 0; i < input.cols(); i++) {         //input.cols should be 2
	    z = z4(i); 
        for (int j = yc3.rows()-1; j >= 0 ; j--){   // j has to be a signed integer
            x = yc3(j,i);
            y = B(0)* x + z;     // calculate output
            z = B(1)* x - A * y;   // calculate internal state
         //   Forward(j,i) = y;    // store result of forward processing.
        }
        z5(i) = z;   // calculate the new state z
     }

     // next step, filter it again, yc5 = filter(b(:,ii),a(:,ii),yc2(end:-1:1,:), zo);
      for (size_t i = 0; i < input.cols(); i++) {         //input.cols should be 2
	    z = z5(i); 
        for (int j = Forward.rows()-1; j >=0 ; j--){  // j has to be a signed integer
            x = Forward(j,i);
            y = B(0)* x + z;     // calculate output
            z = B(1)* x - A * y;   // calculate internal state
            Result(j,i) = y;    // store result of forward processing.
        }
       // z5(i) = z;   // calculate the new state z
     }

     return Result.transpose();
}
     
Vec segement(Mat vel, Mat pos){
    // check the starting index and ending index of the traj. 
    // vel is the velocity matrix, has dimension samples x 2 (for 2D plane)
    Vec index(2);  // 0: starting index, 1: end index
    int c = 0;
    Vec target(2); target << 0.4, 0.1; // this i target
    cout << "vel.rows()" << vel.rows() << endl;
    bool flag_start = false;
    for (size_t i = 0; i < vel.rows(); i++){
        // when to start, when the velocity is greater than 1e-3.
        if ((vel.row(i)).norm() > 1e-3 && flag_start == false){
            index(0) = i;
            flag_start = true;
        }
        // when to stop, when the velocity is less than 1e-3
        if ((vel.row(i)).norm() < 1e-3 && flag_start == true && (pos.row(i) - target.transpose()).norm() < 0.02){
            index(1) = i;
            c = 1;
            break;
        }
    }
    if (c == 0){
        index(1) = vel.rows()-1;
    }

    return index;
}

Mat spline(Mat data, int N){  // downsampling the data, the same as spline in matlab.
    // the input data has the dimension 1 x samples, N is the number of new samples
    
    // check the total length of the traj, figure out how many points needed to sample from this trajectory.
   // realtype length = (data.block(0,0,2,data.cols()-1) - data.block(0,1,2,data.cols()-1)).colwise().norm().sum();
  //  realtype master_l = 0.794728217992968;
   // int N = (int) ceil(200 * length / master_l);
    double t0, t1, t2, t3, t4, t5, t6;
    t0 = drdGetTime();
    
    Mat result = Mat::Zero(1,N);  // store the result
    // calculate the old timestamps and the new timestamps
    vector<realtype> time_old; 
    vector<realtype> time_new;
    for (size_t i = 0; i < data.cols(); i++){
        time_old.push_back(i*0.002);
    }
    
    realtype t_start = 0; 
    realtype dt = time_old.back()/(N-1);
    
    for (size_t i = 0; i < N; i++){
        time_new.push_back(t_start + dt*i);
    }
    t1 = drdGetTime(); cout << "t1-t0 is:" << t1-t0 << endl;
    // based on time_old, data.row(i), time_new, perform spline. This is the conversion from https://blog.csdn.net/jpc20144055069/article/details/103204377
    int n = data.cols(); // how many samples, int n = xy.size();

    Mat a = Mat::Zero(n-1, 1);
    Mat b = Mat::Zero(n-1, 1);
    Mat d = Mat::Zero(n-1, 1);
    Mat dx = Mat::Zero(n-1, 1);
    Mat dy = Mat::Zero(n-1, 1);

    for (size_t i = 0; i< n-1; i++){
        a(i,0) = data(0,i);         //data.col(i);
        dx(i,0) = time_old[i+1] - time_old[i];
        dy(i,0) = data(0, i+1) - data(0, i);
    }
    t2 = drdGetTime(); cout << "t2-t1 is:" << t2- t1 << endl;

    Mat A = Mat::Zero(n,n);
    Mat B = Mat::Zero(n,1);
    A(0,0) = 1; A(n-1,n-1) = 1;

    for(size_t i = 1; i<= n -2 ; i++){
        A(i,i-1) = dx(i-1, 0);
        A(i,i) = 2 * (dx(i-1,0)+dx(i,0));
        A(i,i+1) = dx(i,0);
        B(i,0) = 3 *(dy(i,0)/dx(i,0) - dy(i-1,0)/dx(i-1,0));
    }
    t3 = drdGetTime(); cout << "t3-t2 is:" << t3-t2 << endl;
    
    // suppose A positiv semidefinit, use LDLT to calculate inverse, 
    Eigen::LDLT<Mat> decomp(A);
    Mat c = decomp.solve(B);   // c = inv(A) * B
    
    t4 = drdGetTime(); cout << "t4-t3 is:"<< t4-t3 << endl; 

    for(size_t i = 0; i <= n-2; i++){
    	d(i,0) = (c(i+1, 0) -c(i,0)) / (3*dx(i,0));
	    b(i,0) = dy(i,0)/dx(i,0) - dx(i,0)*(2*c(i,0)+c(i+1,0))/3;
    }
     
    t5 = drdGetTime(); cout << "t5-t4 is:"<< t5-t4 << endl;

    int k;
    for (int i = 0; i < N; i++){
	    for (int j = 0; j <= n - 2; j++){
	        if (time_new[i] >= time_old[j] && time_new[i] < time_old[j + 1]){
		        k = j;
		        break;
	        }
	        else if (time_new[i] >= time_old[n-1]){   // the last time stamp
		        k = n-2;
	        }
	    }

	    realtype predV = a(k, 0) + b(k, 0)*(time_new[i] - time_old[k]) + c(k, 0)*(time_new[i] - time_old[k])*(time_new[i] - time_old[k]) + d(k, 0)*(time_new[i] - time_old[k])*(time_new[i] - time_old[k])*(time_new[i] - time_old[k]);
	    //yy.push_back(middleV);
        result(0,i) = predV;
    }
    
    t6 = drdGetTime(); cout << "t6-t5 is:"<< t6-t5 << endl;

    return result;
}


Mat spline_tk (Mat data, int N){  // downsampling the data, use the spline function module found online.
    
    Mat result = Mat::Zero(1,N);  // placeholders for storing result.
    // generate new time stamps for downsampling.
    vector<realtype> time_old; 
    vector<realtype> time_new;
    for (size_t i = 0; i < data.cols(); i++){
        time_old.push_back(i*0.002);
    }
    
    realtype t_start = 0; 
    realtype dt = time_old.back()/(N-1);
    
    for (int i = 0; i < N; i++){
        time_new.push_back(t_start + dt*i);
    }

    // convert the data from eigen vector to a 
    vector<realtype> output;
    // vector<realtype> output (&data[0], data.data()+data.cols()*data.rows());
    for (size_t i = 0; i < data.cols(); i++){
        output.push_back(data(0,i));
    }

    // construct spline object
    tk::spline s(time_old, output);

    // reasonalize the output at fixed certain point
    for (int i = 0; i < N; i++){
        result(0,i) = s(time_new[i]); 
    }
    return result;
}

int inertia_matrix_hd (realtype j[DHD_MAX_DOF], realtype inertia[6][6]) {
	int ss_1;
        ss_1 = dhdGetJointAngles(j);
        if (ss_1 == 0){
            cout << "successfully retrieve joint angles." <<endl;
        }
        else{
            cout << "cannot successfully retrieve joint angles." << endl;
	    return 0;
        }
        cout << "Comes here 2" << endl;

        int ss_2;
        ss_2 = dhdJointAnglesToInertiaMatrix(j, inertia);
        if (ss_2 == 0){
            cout << "successfully retrieve inertia matrix." <<endl;
        }
        else {
            cout << "cannot successfully retrieve inertia matrix." << endl;
            return 0;
	}

	return 1;
}







