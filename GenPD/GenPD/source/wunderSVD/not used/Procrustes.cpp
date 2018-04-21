#include "Procrustes.h"

void SVDify(Eigen::Matrix<float,3,Eigen::Dynamic>& R, const Eigen::Matrix<float,3,Eigen::Dynamic>& A)
{
  assert(A.rows() == 3);
  const int numMatrices = A.cols() / 3;
  assert(numMatrices * 3 == A.cols());
  R.resize(3, 3*numMatrices);

  for (int i=0; i<numMatrices; i++)
  {
    // convert block to Eigen matrix M and also transpose while doing so:
    Eigen::Matrix<float, 3, 3> Ma, Mr;
    for (int a=0; a<3; a++)
    {
      for (int b=0; b<3; b++)
      {
        Ma(a, b) = (float)A(a,3*i + b);
      }
    }

    // SVDify:
    Eigen::Matrix<float, 3, 1> S;
    wunder_polar_svd(Ma, Mr, S);

    // write back into R:
    for (int a=0; a<3; a++)
    {
      for (int b=0; b<3; b++)
      {
        R(a,3*i + b) = Mr(a, b);
      }
    }
  }
}

void vectorProcrustes(Eigen::Matrix<float, 3, 3> &R, Eigen::Matrix<float, 3, 1> &S, const Eigen::Matrix<float, Eigen::Dynamic, 3> &X, const Eigen::Matrix<float, Eigen::Dynamic, 3> &Y)
{
  Eigen::Matrix<float, 3, 3> YtX = Y.transpose() * X;
  wunder_polar_svd(YtX, R, S);
}

void pointProcrustes(Eigen::Matrix<float, 3, 3> &R, Eigen::Matrix<float, 3, 1> &t, const Eigen::Matrix<float, Eigen::Dynamic, 3> &P, const Eigen::Matrix<float, Eigen::Dynamic, 3> &Q)
{
  Eigen::Matrix<float, 3, 1> Pavg, Qavg;
  averageMatrix(Pavg, P);
  averageMatrix(Qavg, Q);	

  Eigen::Matrix<float, Eigen::Dynamic, 3> X, Y;
  X = P;
  Y = Q;
  subtractAverage(X, Pavg);
  subtractAverage(Y, Qavg);
  
  Eigen::Matrix<float, 3, 3> YtX = Y.transpose() * X;
  wunder_polar_svd(YtX, R);

  t = Qavg - R*Pavg;
}

void pointProcrustes(Eigen::Matrix<float, 3, 3> &R, Eigen::Matrix<float, 3, 1> &t, Eigen::Matrix<float, 3, 1> &S, const Eigen::Matrix<float, Eigen::Dynamic, 3> &P, const Eigen::Matrix<float, Eigen::Dynamic, 3> &Q)
{
  Eigen::Matrix<float, 3, 1> Pavg, Qavg;
  averageMatrix(Pavg, P);
  averageMatrix(Qavg, Q);	

  Eigen::Matrix<float, Eigen::Dynamic, 3> X, Y;
  X = P;
  Y = Q;
  subtractAverage(X, Pavg);
  subtractAverage(Y, Qavg);
  vectorProcrustes(R, S, X, Y);

  t = Qavg - R*Pavg;
}

void averageMatrix(Eigen::Matrix<float, 3, 1> &avg, const Eigen::Matrix<float, Eigen::Dynamic, 3> &P)
{	
  const int cnt = (int)P.rows();
  for (int c=0; c<3; c++) avg(c,0) = float(0.0);

  for (int i=0; i<cnt; i++)
  {
    for (int c=0; c<3; c++) avg(c,0) += P(i, c);
  }

  avg /= float(cnt);	
}

void subtractAverage(Eigen::Matrix<float, Eigen::Dynamic, 3> &P, const Eigen::Matrix<float, 3, 1> &avg)
{
  const int cnt = (int)P.rows();
  for (int i=0; i<cnt; i++)
  {
    for (int c=0; c<3; c++) P(i, c) -= avg(c,0);
  }
}

void subtractAverage(Eigen::Matrix<float, 3, 3> &P, const Eigen::Matrix<float, 3, 1> &avg)
{
  const int cnt = (int)P.rows();
  for (int i=0; i<cnt; i++)
  {
    for (int c=0; c<3; c++) P(i, c) -= avg(c,0);
  }
}