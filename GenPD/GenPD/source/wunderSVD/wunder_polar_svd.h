#ifndef WUNDER_POLAR_SVD
#define WUNDER_POLAR_SVD

// Computes the closest rotation to input matrix A using specialized 3x3 SVD singular value decomposition (WunderSVD3x3)
// Inputs:
//   A  3 by 3 matrix to be decomposed
// Outputs:
//   R  3 by 3 closest element in SO(3) (closeness in terms of Frobenius metric)
//
//	This means that det(R) = 1. Technically it's not polar decomposition which guarantees positive semidefinite
//   stretch factor (at the cost of having det(R) = -1).
//
template<typename Mat, typename Vec>
inline void wunder_polar_svd(const Mat& A, Mat& R, Vec& S);

// Implementation
#include "WunderSVD3x3.h"
//#ifndef __APPLE__
//#  include "WunderSVD3x3_SSE.h"
//#  include "WunderSVD3x3_AVX.h"
//#endif

template<typename Mat, typename Vec>
inline void wunder_polar_svd(const Mat& A, Mat& R, Vec& S)
{
	// should be caught at compile time, but just to be 150% sure:
	assert(A.rows() == 3 && A.cols() == 3);

	Eigen::Matrix<typename Mat::Scalar, 3, 3> U, Vt;	
	wunderSVD3x3(A, U, S, Vt);
	R = U * Vt.transpose();
}

template<typename Mat>
inline void wunder_polar_svd(const Mat& A, Mat& R)
{
	// should be caught at compile time, but just to be 150% sure:
	assert(A.rows() == 3 && A.cols() == 3);

	Eigen::Matrix<typename Mat::Scalar, 3, 3> U, Vt;
	Eigen::Matrix<typename Mat::Scalar, 3, 1> S;
	wunderSVD3x3(A, U, S, Vt);
	R = U * Vt.transpose();
}

//#ifndef __APPLE__
//
//template<typename T>
//void wunder_polar_svd_SSE(const Eigen::Matrix<T, 3*4, 3>& A, Eigen::Matrix<T, 3*4, 3> &R)
//{
//	// should be caught at compile time, but just to be 150% sure:
//	assert(A.rows() == 3*4 && A.cols() == 3);
//
//	Eigen::Matrix<T, 3*4, 3> U, Vt;
//	Eigen::Matrix<T, 3*4, 1> S;	
//	wunderSVD3x3_SSE(A, U, S, Vt);
//
//	for (int k=0; k<4; k++)
//	{
//		R.block(3*k, 0, 3, 3) = U.block(3*k, 0, 3, 3) * Vt.block(3*k, 0, 3, 3).transpose();
//	}
//
//	//// test:
//	//for (int k=0; k<4; k++)
//	//{
//	//	Eigen::Matrix3f Apart = A.block(3*k, 0, 3, 3);
//	//	Eigen::Matrix3f Rpart;
//	//	wunder_polar_svd(Apart, Rpart);
//
//	//	Eigen::Matrix3f Rpart_SSE = R.block(3*k, 0, 3, 3);
//	//	Eigen::Matrix3f diff = Rpart - Rpart_SSE;
//	//	float diffNorm = diff.norm();
//
//	//	int hu = 1;
//	//}
//	//// eof test
//}
//
//template<typename T>
//void wunder_polar_svd_AVX(const Eigen::Matrix<T, 3*8, 3>& A, Eigen::Matrix<T, 3*8, 3> &R)
//{
//	// should be caught at compile time, but just to be 150% sure:
//	assert(A.rows() == 3*8 && A.cols() == 3);
//
//	Eigen::Matrix<T, 3*8, 3> U, Vt;
//	Eigen::Matrix<T, 3*8, 1> S;	
//	wunderSVD3x3_AVX(A, U, S, Vt);
//
//	for (int k=0; k<8; k++)
//	{
//		R.block(3*k, 0, 3, 3) = U.block(3*k, 0, 3, 3) * Vt.block(3*k, 0, 3, 3).transpose();
//	}
//
//	// test:
//	for (int k=0; k<8; k++)
//	{
//		Eigen::Matrix3f Apart = A.block(3*k, 0, 3, 3);
//		Eigen::Matrix3f Rpart;
//		wunder_polar_svd(Apart, Rpart);
//
//		Eigen::Matrix3f Rpart_SSE = R.block(3*k, 0, 3, 3);
//		Eigen::Matrix3f diff = Rpart - Rpart_SSE;
//		float diffNorm = diff.norm();
//		if (abs(diffNorm) > 0.001) 
//		{
//			printf("Huh: diffNorm = %15f (k = %i)\n", diffNorm, k);
//		}
//
//		int hu = 1;
//	}
//	// eof test
//}
//#endif


#endif
