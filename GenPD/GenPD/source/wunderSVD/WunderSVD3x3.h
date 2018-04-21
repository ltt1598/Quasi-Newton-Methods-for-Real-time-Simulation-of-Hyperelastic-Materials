#include <Eigen/Dense>

// Super fast 3x3 SVD according to http://pages.cs.wisc.edu/~sifakis/project_pages/svd.html
// The resulting decomposition is A = U * diag(S[0], S[1], S[2]) * V'
// BEWARE: this SVD algorithm guarantees that det(U) = det(V) = 1, but this 
// comes at the cost that 'sigma3' can be negative
// for computing polar decomposition it's great because all we need to do is U*V'
// and the result will automatically have positive determinant

template<typename T>
void wunderSVD3x3(const Eigen::Matrix<T, 3, 3>& A, Eigen::Matrix<T, 3, 3> &U, Eigen::Matrix<T, 3, 1> &S, Eigen::Matrix<T, 3, 3>&V);
