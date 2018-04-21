#ifndef _PROCRUSTES_H_
#define _PROCRUSTES_H_

#include "Eigen/Dense"
#include "wunder_polar_svd.h"

// from input n x 3 matrices 'X' and 'Y' (of 3-vectors, each in one row) computes optimal fitting rotation 'R':
// order: X-vectors are the ones being rotated onto Y:
void vectorProcrustes(Eigen::Matrix<float, 3, 3> &R, Eigen::Matrix<float, 3, 1> &S, const Eigen::Matrix<float, Eigen::Dynamic, 3> &X, const Eigen::Matrix<float, Eigen::Dynamic, 3> &Y);

// from input n x 3 matrices 'P' and 'Q' (of 3-points, each in one row) computes optimal fitting rotation 'R' and translation 't'
// order: P-points are the ones being rotated onto Q:
void pointProcrustes(Eigen::Matrix<float, 3, 3> &R, Eigen::Matrix<float, 3, 1> &t, const Eigen::Matrix<float, Eigen::Dynamic, 3> &P, const Eigen::Matrix<float, Eigen::Dynamic, 3> &Q);
void pointProcrustes(Eigen::Matrix<float, 3, 3> &R, Eigen::Matrix<float, 3, 1> &t, Eigen::Matrix<float, 3, 1> &S, const Eigen::Matrix<float, Eigen::Dynamic, 3> &P, const Eigen::Matrix<float, Eigen::Dynamic, 3> &Q);

void averageMatrix(Eigen::Matrix<float, 3, 1> &avg, const Eigen::Matrix<float, Eigen::Dynamic, 3> &P);
void subtractAverage(Eigen::Matrix<float, Eigen::Dynamic, 3> &P, const Eigen::Matrix<float, 3, 1> &avg);

void testProcrustes();
void testProcrustesVec();

// assumes 'A' is 3 x 3r matrix, returns best fitting rotations for each 3x3 block in 'R' (again 3 x 3r):
void SVDify(Eigen::Matrix<float,3,Eigen::Dynamic> &R, const Eigen::Matrix<float,3,Eigen::Dynamic> &A);

#endif
