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

//----------AttachmentConstraint Class----------//
AttachmentConstraint::AttachmentConstraint(unsigned int p0, const EigenVector3& fixedpoint) :
Constraint(CONSTRAINT_TYPE_ATTACHMENT),
m_p0(p0),
m_fixd_point(fixedpoint)
{

}

AttachmentConstraint::AttachmentConstraint(ScalarType stiffness, unsigned int p0, const EigenVector3& fixedpoint) :
Constraint(CONSTRAINT_TYPE_ATTACHMENT, stiffness),
m_p0(p0),
m_fixd_point(fixedpoint)
{

}

AttachmentConstraint::AttachmentConstraint(const AttachmentConstraint& other) :
Constraint(other),
m_p0(other.m_p0),
m_fixd_point(other.m_fixd_point)
{

}

AttachmentConstraint::~AttachmentConstraint()
{

}

// 0.5*k*(current_length)^2
ScalarType AttachmentConstraint::EvaluateEnergy(const VectorX& x)
{
	ScalarType e_i = 0.5*(m_stiffness)*(x.block_vector(m_p0) - m_fixd_point).squaredNorm();
	m_energy = e_i;

	return e_i;
}

ScalarType AttachmentConstraint::GetEnergy()
{
	return m_energy;
}

// attachment spring gradient: k*(current_length)*current_direction
void AttachmentConstraint::EvaluateGradient(const VectorX& x, VectorX& gradient)
{
	EigenVector3 g_i = (m_stiffness)*(x.block_vector(m_p0) - m_fixd_point);
	gradient.block_vector(m_p0) += g_i;
}

void AttachmentConstraint::EvaluateGradient(const VectorX& x)
{
	m_g = (m_stiffness)*(x.block_vector(m_p0) - m_fixd_point);
}

void AttachmentConstraint::GetGradient(VectorX& gradient)
{
	gradient.block_vector(m_p0) += m_g;
}

ScalarType AttachmentConstraint::EvaluateEnergyAndGradient(const VectorX& x, VectorX& gradient)
{
	EvaluateEnergyAndGradient(x);
	gradient.block_vector(m_p0) += m_g;

	return m_energy;
}
ScalarType AttachmentConstraint::EvaluateEnergyAndGradient(const VectorX& x)
{
	// energy
	m_energy = 0.5*(m_stiffness)*(x.block_vector(m_p0) - m_fixd_point).squaredNorm();
	// gradient
	m_g = (m_stiffness)*(x.block_vector(m_p0) - m_fixd_point);

	return m_energy;
}
ScalarType AttachmentConstraint::GetEnergyAndGradient(VectorX& gradient)
{
	gradient.block_vector(m_p0) += m_g;
	return m_energy;
}

void AttachmentConstraint::EvaluateHessian(const VectorX& x, bool definiteness_fix /* = false */ /* = 1 */)
{
	ScalarType ks = m_stiffness;

	EigenVector3 H_d;
	for (unsigned int i = 0; i != 3; i++)
	{
		H_d(i) = ks;
	}
	m_H = H_d.asDiagonal();
}

void AttachmentConstraint::EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets, bool definiteness_fix)
{
	EvaluateHessian(x, definiteness_fix);
	for (unsigned int i = 0; i != 3; i++)
	{
		hessian_triplets.push_back(SparseMatrixTriplet(3 * m_p0 + i, 3 * m_p0 + i, m_H(i, i)));
	}
}

void AttachmentConstraint::ApplyHessian(const VectorX& x, VectorX& b)
{
	b.block_vector(m_p0) += m_H * x.block_vector(m_p0);
}

void AttachmentConstraint::EvaluateWeightedLaplacian(std::vector<SparseMatrixTriplet>& laplacian_triplets)
{
	ScalarType ks = m_stiffness;
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p0, 3 * m_p0, ks));
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p0 + 1, 3 * m_p0 + 1, ks));
	laplacian_triplets.push_back(SparseMatrixTriplet(3 * m_p0 + 2, 3 * m_p0 + 2, ks));
}

void AttachmentConstraint::EvaluateWeightedDiagonal(std::vector<SparseMatrixTriplet>& diagonal_triplets)
{
	ScalarType ks = m_stiffness;
	diagonal_triplets.push_back(SparseMatrixTriplet(3 * m_p0, 3 * m_p0, ks));
	diagonal_triplets.push_back(SparseMatrixTriplet(3 * m_p0 + 1, 3 * m_p0 + 1, ks));
	diagonal_triplets.push_back(SparseMatrixTriplet(3 * m_p0 + 2, 3 * m_p0 + 2, ks));
}

void AttachmentConstraint::EvaluateWeightedLaplacian1D(std::vector<SparseMatrixTriplet>& laplacian_1d_triplets)
{
	ScalarType ks = m_stiffness;
	laplacian_1d_triplets.push_back(SparseMatrixTriplet(m_p0, m_p0, ks));
}

// IO
void AttachmentConstraint::WriteToFileOBJ(std::ofstream& outfile, int& existing_vertices)
{
	// assume outfile is correctly open.

	std::vector<glm::vec3>& vertices = m_attachment_constraint_body.GetPositions();
	std::vector<unsigned short>& triangles = m_attachment_constraint_body.GetTriangulation();
	glm::vec3 move_to = Eigen2GLM(m_fixd_point);
	glm::vec3 v;

	for (unsigned int i = 0; i < vertices.size(); i++)
	{
		v = vertices[i] + move_to;
		// save positions
		outfile << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
	}
	outfile << std::endl;
	for (unsigned int f = 0; f < triangles.size(); f += 3)
	{
		outfile << "f " << triangles[f + 0] + 1 + existing_vertices << " " << triangles[f + 1] + 1 + existing_vertices << " " << triangles[f + 2] + 1 + existing_vertices << std::endl;
	}
	outfile << std::endl;

	existing_vertices += vertices.size();
}

void AttachmentConstraint::WriteToFileOBJHead(std::ofstream& outfile)
{
	EigenVector3& v = m_fixd_point;
	outfile << "// " << m_p0 << " " << v[0] << " " << v[1] << " " << v[2] << std::endl;
}