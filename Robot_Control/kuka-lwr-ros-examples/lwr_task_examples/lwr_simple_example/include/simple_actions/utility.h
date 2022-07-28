#pragma once
#include "eigen3/Eigen/Dense"
#include "vector"
#include <fstream>
#include "iostream"
#include "dhdc.h"
#include "drdc.h"
#include "spline.h"
using namespace std;
using namespace Eigen ;
#ifndef PASSIVE_DS_TYPEDEFS_H
#define PASSIVE_DS_TYPEDEFS_H


	//#ifdef USE_DOUBLE_PRECISION
	typedef double realtype;
	/*#else
	typedef float realtype;
	#endif*/
	typedef Eigen::Matrix<realtype, Eigen::Dynamic, Eigen::Dynamic> Mat;
	typedef Eigen::Matrix<realtype, Eigen::Dynamic, 1> Vec;

#endif
         const realtype pi=         3.141592653589793238462643383279502884 /* pi */;

         template <typename T> int sgn(T val) {
             return (T(0) < val) - (val < T(0));
         }

namespace special_math_functions
{

    Mat Skew(const Vec &v);
    Quaterniond quatMult(Eigen::Quaterniond q1, Eigen::Quaterniond q2) ;
    Vec quatlog(Eigen::Quaterniond q1) ;
    Mat PushBackColumn(Mat m, Vec x);
    Mat PushBackColumnMat(Mat m, Mat x);
    realtype eps();
    realtype smooth_rise_threshold(realtype value, realtype lo, realtype hi, realtype v1, realtype v2);
    realtype smooth_fall_stiffness(realtype value, realtype lo, realtype hi, realtype v1, realtype v2);
}
namespace StiffnessProfiles
{
	Mat StiffnessProfile_MainTask(realtype t);
	Mat StiffnessProfile_NullSpace(realtype t);
}
namespace GeneralFunction {
	void Write_To_File(string file_name, vector<vector<float> > Input_Data);
	realtype smooth_transition_rising(realtype t, realtype t_max, realtype t_min, realtype scale);
	realtype smooth_transition_fall(realtype E, realtype E_max, realtype E_min);
    void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove) ;
    realtype mod(realtype x, realtype y) ;
    void  Vec2double(Vec a,double q[]) ;
    Vec double2Vec(double q[]) ;
    template <size_t size_x, size_t size_y>

    Mat double2Mat(double(&T)[size_x][size_y])
    {
        int rows = size_x;
        int cols = size_y;
        Mat out = Mat::Zero(rows, cols);
        for (int i = 0; i<rows; i++)
        {
            for (int j = 0; j<cols; j++)
            {
                out(i, j) = T[i][j];
            }

        }
        return out;
    }
}

bool haptic_initial(double y, double z); // haptic device initialization

Mat butterworth(Mat input); // butterworth filter 

Vec segement(Mat vel, Mat pos);   // give back where the trajectory begins and ends.

Mat spline(Mat data, int N);  // downsampling the data, the same as spline in matlab.

Mat spline_tk (Mat data, int N);  // downsampling the data, use the spline function module found online.

int inertia_matrix_hd (realtype j[DHD_MAX_DOF], realtype inertia[6][6]);  //retrieve inertia matrix of haptic device

