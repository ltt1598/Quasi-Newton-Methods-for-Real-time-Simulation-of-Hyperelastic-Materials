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

#ifdef ENABLE_MATLAB_DEBUGGING
#include "matlab_debugger.h"
extern MatlabDebugger *g_debugger;
#endif

//----------CollisionSpringConstraint Class----------//
CollisionSpringConstraint::CollisionSpringConstraint(unsigned int p0, const EigenVector3& fixedpoint, const EigenVector3& normal) :
Constraint(CONSTRAINT_TYPE_COLLISION),
m_p0(p0),
m_fixed_point(fixedpoint),
m_normal(normal)
{

}

CollisionSpringConstraint::CollisionSpringConstraint(ScalarType stiffness, unsigned int p0, const EigenVector3& fixedpoint, const EigenVector3& normal) :
Constraint(CONSTRAINT_TYPE_COLLISION, stiffness),
m_p0(p0),
m_fixed_point(fixedpoint),
m_normal(normal)
{

}

CollisionSpringConstraint::CollisionSpringConstraint(const CollisionSpringConstraint& other) :
Constraint(other),
m_p0(other.m_p0),
m_fixed_point(other.m_fixed_point),
m_normal(other.m_normal)
{

}

CollisionSpringConstraint::~CollisionSpringConstraint()
{

}

bool CollisionSpringConstraint::IsActive(const VectorX& x)
{
	EigenVector3 x0 = x.block_vector(m_p0);

	if ((x0 - m_fixed_point).dot(m_normal) > 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

// 0.5*k*(current_length)^2
ScalarType CollisionSpringConstraint::EvaluateEnergy(const VectorX& x)
{
	if (IsActive(x))
	{
		ScalarType e_i = 0.5*(m_stiffness)*(x.block_vector(m_p0) - m_fixed_point).squaredNorm();
		m_energy = e_i;

		return e_i;
	}
	else
	{
		m_energy = 0;

		return 0;
	}
}

ScalarType CollisionSpringConstraint::GetEnergy()
{
	return m_energy;
}

// attachment spring gradient: k*(current_length)*current_direction
void CollisionSpringConstraint::EvaluateGradient(const VectorX& x, VectorX& gradient)
{
	if (IsActive(x))
	{
		EigenVector3 g_i = (m_stiffness)*(x.block_vector(m_p0) - m_fixed_point);
		gradient.block_vector(m_p0) += g_i;
	}
}

void CollisionSpringConstraint::EvaluateGradient(const VectorX& x)
{
	if (IsActive(x))
	{
		m_g = (m_stiffness)*(x.block_vector(m_p0) - m_fixed_point);
	}
	else
	{
		m_g.setZero();
	}
}

void CollisionSpringConstraint::GetGradient(VectorX& gradient)
{
	gradient.block_vector(m_p0) += m_g;
}

ScalarType CollisionSpringConstraint::EvaluateEnergyAndGradient(const VectorX& x, VectorX& gradient)
{
	EvaluateEnergyAndGradient(x);
	gradient.block_vector(m_p0) += m_g;

	return m_energy;
}
ScalarType CollisionSpringConstraint::EvaluateEnergyAndGradient(const VectorX& x)
{
	if (IsActive(x))
	{
		// energy
		m_energy = 0.5*(m_stiffness)*(x.block_vector(m_p0) - m_fixed_point).squaredNorm();
		// gradient
		m_g = (m_stiffness)*(x.block_vector(m_p0) - m_fixed_point);
	}
	else
	{
		m_energy = 0;
		m_g.setZero();
	}

	return m_energy;
}
ScalarType CollisionSpringConstraint::GetEnergyAndGradient(VectorX& gradient)
{
	gradient.block_vector(m_p0) += m_g;
	return m_energy;
}

void CollisionSpringConstraint::EvaluateHessian(const VectorX& x, bool definiteness_fix /* = false */ /* = 1 */)
{
	if (IsActive(x))
	{
		ScalarType ks = m_stiffness;

		EigenVector3 H_d;
		for (unsigned int i = 0; i != 3; i++)
		{
			H_d(i) = ks;
		}
		m_H = H_d.asDiagonal();
	}
	else
	{
		m_H.setZero();
	}
}

void CollisionSpringConstraint::EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets, bool definiteness_fix)
{
	EvaluateHessian(x, definiteness_fix);
	for (unsigned int i = 0; i != 3; i++)
	{
		hessian_triplets.push_back(SparseMatrixTriplet(3 * m_p0 + i, 3 * m_p0 + i, m_H(i, i)));
	}
}

void CollisionSpringConstraint::ApplyHessian(const VectorX& x, VectorX& b)
{
	b.block_vector(m_p0) += m_H * x.block_vector(m_p0);
}

void CollisionSpringConstraint::EvaluateWeightedLaplacian(std::vector<SparseMatrixTriplet>& laplacian_triplets)
{
	ScalarType ks = m_stiffness;
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p0, 3 * m_p0, ks));
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p0 + 1, 3 * m_p0 + 1, ks));
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p0 + 2, 3 * m_p0 + 2, ks));
}

void CollisionSpringConstraint::EvaluateWeightedLaplacian1D(std::vector<SparseMatrixTriplet>& laplacian_1d_triplets)
{
	ScalarType ks = m_stiffness;
	laplacian_1d_triplets.push_back(SparseMatrixTriplet(m_p0, m_p0, ks));
}
