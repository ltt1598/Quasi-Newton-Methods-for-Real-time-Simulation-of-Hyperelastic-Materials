// ---------------------------------------------------------------------------------//
// Copyright (c) 2015, Regents of the University of Pennsylvania                    //
// All rights reserved.                                                             //
//                                                                                  //
// Redistribution and use in source and binary forms, with or without               //
// modification, are permitted provided that the following conditions are met:      //
//     * Redistributions of source code must retain the above copyright             //
//       notice, this list of conditions and the following disclaimer.              //
//     * Redistributions in binary form must reproduce the above copyright          //
//       notice, this list of conditions and the following disclaimer in the        //
//       documentation and/or other materials provided with the distribution.       //
//     * Neither the name of the <organization> nor the                             //
//       names of its contributors may be used to endorse or promote products       //
//       derived from this software without specific prior written permission.      //
//                                                                                  //
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND  //
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED    //
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           //
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY               //
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES       //
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;     //
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND      //
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       //
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS    //
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                     //
//                                                                                  //
// Contact Tiantian Liu (ltt1598@gmail.com) if you have any questions.              //
//----------------------------------------------------------------------------------//


#include "constraint.h"
#include <unsupported/Eigen/KroneckerProduct>

#ifdef ENABLE_MATLAB_DEBUGGING
#include "matlab_debugger.h"
extern MatlabDebugger *g_debugger;
#endif

//----------SpringConstraint Class----------//
SpringConstraint::SpringConstraint(ConstraintType type, unsigned int p1, unsigned int p2, ScalarType length) :
Constraint(type),
m_p1(p1),
m_p2(p2),
m_rest_length(length)
{
}

SpringConstraint::SpringConstraint(unsigned int p1, unsigned int p2, ScalarType length) :
Constraint(CONSTRAINT_TYPE_SPRING),
m_p1(p1),
m_p2(p2),
m_rest_length(length)
{
}

SpringConstraint::SpringConstraint(ScalarType stiffness, unsigned int p1, unsigned int p2, ScalarType length) :
Constraint(CONSTRAINT_TYPE_SPRING, stiffness),
m_p1(p1),
m_p2(p2),
m_rest_length(length)
{
}

SpringConstraint::SpringConstraint(const SpringConstraint& other) :
Constraint(other),
m_p1(other.m_p1),
m_p2(other.m_p2),
m_rest_length(other.m_rest_length)
{
}

SpringConstraint::~SpringConstraint()
{
}

// 0.5*k*(current_length - rest_length)^2
ScalarType SpringConstraint::EvaluateEnergy(const VectorX& x)
{
	EigenVector3 x_ij = x.block_vector(m_p1) - x.block_vector(m_p2);
	ScalarType length_difference_ij = x_ij.norm() - m_rest_length;
	ScalarType e_ij = 0.5*(m_stiffness)*length_difference_ij*length_difference_ij;

	m_energy = e_ij;

	return e_ij;
}

ScalarType  SpringConstraint::GetEnergy()
{
	return m_energy;
}


// sping gradient: k*(current_length-rest_length)*current_direction;
void SpringConstraint::EvaluateGradient(const VectorX& x, VectorX& gradient)
{
	EigenVector3 x_ij = x.block_vector(m_p1) - x.block_vector(m_p2);
	EigenVector3 g_ij = (m_stiffness)*(x_ij.norm() - m_rest_length)*x_ij.normalized();
	gradient.block_vector(m_p1) += g_ij;
	gradient.block_vector(m_p2) -= g_ij;
}

void SpringConstraint::EvaluateGradient(const VectorX& x)
{
	EigenVector3 x_ij = x.block_vector(m_p1) - x.block_vector(m_p2);
	EigenVector3 g_ij = (m_stiffness)*(x_ij.norm() - m_rest_length)*x_ij.normalized();

	m_g1 = g_ij;
	m_g2 = -g_ij;
}

void SpringConstraint::GetGradient(VectorX& gradient)
{
	gradient.block_vector(m_p1) += m_g1;
	gradient.block_vector(m_p2) += m_g2;
}

ScalarType SpringConstraint::EvaluateEnergyAndGradient(const VectorX& x, VectorX& gradient)
{
	EvaluateEnergyAndGradient(x);
	gradient.block_vector(m_p1) += m_g1;
	gradient.block_vector(m_p2) += m_g2;

	return m_energy;
}
ScalarType SpringConstraint::EvaluateEnergyAndGradient(const VectorX& x)
{
	// energy
	EigenVector3 x_ij = x.block_vector(m_p1) - x.block_vector(m_p2);
	ScalarType x_ij_norm = x_ij.norm();
	EigenVector3 x_ij_normalized = x_ij / x_ij_norm;
	ScalarType length_difference_ij = x_ij_norm - m_rest_length;
	m_energy = 0.5*(m_stiffness)*length_difference_ij*length_difference_ij;
	//gradient
	EigenVector3 g_ij = (m_stiffness)*length_difference_ij*x_ij_normalized;
	m_g1 = g_ij;
	m_g2 = -g_ij;

	return m_energy;
}
ScalarType SpringConstraint::GetEnergyAndGradient(VectorX& gradient)
{
	gradient.block_vector(m_p1) += m_g1;
	gradient.block_vector(m_p2) += m_g2;
	return m_energy;
}

void SpringConstraint::EvaluateHessian(const VectorX& x, bool definiteness_fix)
{
	EigenVector3 x_ij = x.block_vector(m_p1) - x.block_vector(m_p2);
	ScalarType l_ij = x_ij.norm();
	ScalarType l0 = m_rest_length;
	ScalarType ks = m_stiffness;

	m_H = ks * (EigenMatrix3::Identity() - l0 / l_ij*(EigenMatrix3::Identity() - (x_ij*x_ij.transpose()) / (l_ij*l_ij)));
	//EigenMatrix3 k = ks * (1-l0/l_ij) * EigenMatrix3::Identity();

	if (definiteness_fix)
	{
		// definiteness fix
		Eigen::EigenSolver<EigenMatrix3> evd;
		evd.compute(m_H);
		EigenMatrix3 Q = evd.eigenvectors().real();
		EigenVector3 LAMBDA = evd.eigenvalues().real();
		//assert(LAMBDA(0) > 0);
		//ScalarType smallest_lambda = LAMBDA(0) * 1e-10;
		ScalarType smallest_lambda = 1e-6;
		for (unsigned int i = 0; i != LAMBDA.size(); i++)
		{
			//assert(LAMBDA(0) > LAMBDA(i));
			if (LAMBDA(i) < smallest_lambda)
			{
				LAMBDA(i) = smallest_lambda;
			}
		}
		m_H = Q * LAMBDA.asDiagonal() * Q.transpose();
	}
}

void SpringConstraint::EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets, bool definiteness_fix)
{
	EvaluateHessian(x, definiteness_fix);

	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			ScalarType val = m_H(row, col);
			//Update the global hessian matrix
			hessian_triplets.push_back(SparseMatrixTriplet(3 * m_p1 + row, 3 * m_p1 + col, val));
			hessian_triplets.push_back(SparseMatrixTriplet(3 * m_p1 + row, 3 * m_p2 + col, -val));
			hessian_triplets.push_back(SparseMatrixTriplet(3 * m_p2 + row, 3 * m_p1 + col, -val));
			hessian_triplets.push_back(SparseMatrixTriplet(3 * m_p2 + row, 3 * m_p2 + col, val));
		}
	}
}

void SpringConstraint::ApplyHessian(const VectorX& x, VectorX& b)
{
	EigenVector3 p1 = x.block_vector(m_p1);
	EigenVector3 p2 = x.block_vector(m_p2);
	EigenVector3 d = m_H * p1 - m_H * p2;
	b.block_vector(m_p1) += d;
	b.block_vector(m_p2) -= d;
}

void SpringConstraint::EvaluateWeightedLaplacian(std::vector<SparseMatrixTriplet>& laplacian_triplets)
{
	ScalarType ks = m_stiffness;
	// block 1 1
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p1 + 0, 3 * m_p1 + 0, ks));
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p1 + 1, 3 * m_p1 + 1, ks));
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p1 + 2, 3 * m_p1 + 2, ks));
	// block 1 2
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p1 + 0, 3 * m_p2 + 0, -ks));
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p1 + 1, 3 * m_p2 + 1, -ks));
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p1 + 2, 3 * m_p2 + 2, -ks));
	// block 2 1
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p2 + 0, 3 * m_p1 + 0, -ks));
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p2 + 1, 3 * m_p1 + 1, -ks));
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p2 + 2, 3 * m_p1 + 2, -ks));
	// block 2 2
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p2 + 0, 3 * m_p2 + 0, ks));
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p2 + 1, 3 * m_p2 + 1, ks));
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p2 + 2, 3 * m_p2 + 2, ks));
}

void SpringConstraint::EvaluateWeightedDiagonal(std::vector<SparseMatrixTriplet>& diagonal_triplets)
{
	ScalarType ks = m_stiffness;
	// block 1 1
	diagonal_triplets.push_back(SparseMatrixTriplet(3 * m_p1 + 0, 3 * m_p1 + 0, ks));
	diagonal_triplets.push_back(SparseMatrixTriplet(3 * m_p1 + 1, 3 * m_p1 + 1, ks));
	diagonal_triplets.push_back(SparseMatrixTriplet(3 * m_p1 + 2, 3 * m_p1 + 2, ks));
	// block 2 2
	diagonal_triplets.push_back(SparseMatrixTriplet(3 * m_p2 + 0, 3 * m_p2 + 0, ks));
	diagonal_triplets.push_back(SparseMatrixTriplet(3 * m_p2 + 1, 3 * m_p2 + 1, ks));
	diagonal_triplets.push_back(SparseMatrixTriplet(3 * m_p2 + 2, 3 * m_p2 + 2, ks));
}

void SpringConstraint::EvaluateWeightedLaplacian1D(std::vector<SparseMatrixTriplet>& laplacian_1d_triplets)
{
	ScalarType ks = m_stiffness;
	// block 1 1
	laplacian_1d_triplets.push_back(SparseMatrixTriplet(m_p1 + 0, m_p1 + 0, ks));
	// block 1 2
	laplacian_1d_triplets.push_back(SparseMatrixTriplet(m_p1 + 0, m_p2 + 0, -ks));
	// block 2 1
	laplacian_1d_triplets.push_back(SparseMatrixTriplet(m_p2 + 0, m_p1 + 0, -ks));
	// block 2 2
	laplacian_1d_triplets.push_back(SparseMatrixTriplet(m_p2 + 0, m_p2 + 0, ks));
}
