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

#ifndef _CONSTRAINT_H_
#define _CONSTRAINT_H_

#include <vector>
#include <iostream>
#include <fstream>

#include "global_headers.h"
#include "math_headers.h"
#include "tensor_math.h"
#include "opengl_headers.h"
#include "primitive.h"

typedef enum
{
	MATERIAL_TYPE_COROT,
	MATERIAL_TYPE_StVK,
	MATERIAL_TYPE_NEOHOOKEAN_EXTEND_LOG,
	MATERIAL_TYPE_TOTAL_NUM
} MaterialType;

typedef enum 
{
	CONSTRAINT_TYPE_ATTACHMENT,
	CONSTRAINT_TYPE_SPRING,
	CONSTRAINT_TYPE_SPRING_BENDING,
	CONSTRAINT_TYPE_COLLISION,
	CONSTRAINT_TYPE_TET,
	CONSTRAINT_TYPE_NULL,
	CONSTRAINT_TYPE_TOTAL_NUM
} ConstraintType;

class Constraint
{
public:
	Constraint();
	Constraint(ConstraintType type);
	Constraint(ConstraintType type, ScalarType stiffness);
	Constraint(const Constraint& other);
	virtual ~Constraint();

	virtual void GetMaterialProperty(ScalarType& stiffness) { stiffness = m_stiffness; }
	virtual void GetMaterialProperty(MaterialType& type, ScalarType& mu, ScalarType& lambda, ScalarType& kappa) { std::cout << "Warning: reach <Constraint::GetMaterialProperty> base class virtual function." << std::endl; }
	virtual void SetMaterialProperty(ScalarType stiffness) { m_stiffness = stiffness; }
	virtual void SetMaterialProperty(MaterialType type, ScalarType mu, ScalarType lambda, ScalarType kappa, ScalarType laplacian_coeff) { std::cout << "Warning: reach <Constraint::SetMaterialProperty> base class virtual function." << std::endl; }
	virtual ScalarType ComputeLaplacianWeight() { std::cout << "Warning: reach <Constraint::ComputeLaplacianWeight> base class virtual function." << std::endl; return 0; }

	virtual bool VertexIncluded(unsigned int vi) { return false; }

	virtual ScalarType  EvaluateEnergy(const VectorX& x) { std::cout << "Warning: reach <Constraint::EvaluatePotentialEnergy> base class virtual function." << std::endl; return 0; }
	virtual ScalarType  GetEnergy() { std::cout << "Warning: reach <Constraint::GetPotentialEnergy> base class virtual function." << std::endl; return 0; }
	virtual void  EvaluateGradient(const VectorX& x, VectorX& gradient) { std::cout << "Warning: reach <Constraint::EvaluateGradient> base class virtual function." << std::endl; }
	virtual void  EvaluateGradient(const VectorX& x) { std::cout << "Warning: reach <Constraint::EvaluateGradient> base class virtual function." << std::endl; }
	virtual void  GetGradient(VectorX& gradient) { std::cout << "Warning: reach <Constraint::GetGradient> base class virtual function." << std::endl; }
	virtual ScalarType  EvaluateEnergyAndGradient(const VectorX& x, VectorX& gradient) { std::cout << "Warning: reach <Constraint::EvaluateEnergyAndGradient> base class virtual function." << std::endl; return 0; }
	virtual ScalarType  EvaluateEnergyAndGradient(const VectorX& x) { std::cout << "Warning: reach <Constraint::EvaluateEnergyAndGradient> base class virtual function." << std::endl; return 0; }
	virtual ScalarType  GetEnergyAndGradient(VectorX& gradient) { std::cout << "Warning: reach <Constraint::GetEnergyAndGradient> base class virtual function." << std::endl; return 0; }
	virtual void  EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets, bool definiteness_fix = false) { std::cout << "Warning: reach <Constraint::EvaluateHessian> base class virtual function." << std::endl; }
	virtual void  EvaluateHessian(const VectorX& x, bool definiteness_fix = false) { std::cout << "Warning: reach <Constraint::EvaluateHessian> base class virtual function." << std::endl; }
	virtual void  ApplyHessian(const VectorX& x, VectorX& b) {}; // b = H*x, applying hessian for this block only;
	virtual void  EvaluateFiniteDifferentHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets) { std::cout << "Warning: reach <Constraint::EvaluateFiniteDifferentHessian> base class virtual function." << std::endl; }
	virtual void  EvaluateWeightedLaplacian(std::vector<SparseMatrixTriplet>& laplacian_triplets) {std::cout << "Warning: reach <Constraint::EvaluateWeightedLaplacian> base class virtual function." << std::endl;}
	virtual void  EvaluateWeightedDiagonal(std::vector<SparseMatrixTriplet>& diagonal_triplets) {std::cout << "Warning: reach <Constraint::EvaluateWeightedDiagonal> base class virtual function." << std::endl;}
	virtual void  EvaluateWeightedLaplacian1D(std::vector<SparseMatrixTriplet>& laplacian_1d_triplets) { std::cout << "Warning: reach <Constraint::EvaluateWeightedLaplacian1D> base class virtual function." << std::endl; }

	// accesser
	virtual ScalarType GetVolume(const VectorX& x) { return 0; }
	virtual void GetRotation(const VectorX& x, std::vector<EigenQuaternion>& rotations) {}

	// inline
	const ConstraintType& Type() { return m_constraint_type; }

protected:
	ConstraintType m_constraint_type;
	ScalarType m_stiffness;

	// saved energy
	ScalarType m_energy;

// for visualization and selection
public:
	virtual void WriteToFileOBJ(std::ofstream& outfile, int& existing_vertices) { /*do nothing*/ }
	virtual void WriteToFileOBJHead(std::ofstream& outfile) { /*do nothing*/ }
	virtual void WriteToFileOBJTet(std::ofstream& outfile) { /*do nothing*/ }
	virtual void DrawTet(const VectorX& x, const VBO& vbos) { /*do nothing*/ }
	//virtual ScalarType RayConstraintIntersection() {return false;}
};

class AttachmentConstraint : public Constraint
{
public:
	AttachmentConstraint(unsigned int p0, const EigenVector3& fixedpoint);
	AttachmentConstraint(ScalarType stiffness, unsigned int p0, const EigenVector3& fixedpoint);
	AttachmentConstraint(const AttachmentConstraint& other);
	virtual ~AttachmentConstraint();

	virtual ScalarType  EvaluateEnergy(const VectorX& x);
	virtual ScalarType  GetEnergy();
	virtual void  EvaluateGradient(const VectorX& x, VectorX& gradient);
	virtual void  EvaluateGradient(const VectorX& x);
	virtual void  GetGradient(VectorX& gradient);
	virtual ScalarType  EvaluateEnergyAndGradient(const VectorX& x, VectorX& gradient);
	virtual ScalarType  EvaluateEnergyAndGradient(const VectorX& x);
	virtual ScalarType  GetEnergyAndGradient(VectorX& gradient);
	virtual void  EvaluateHessian(const VectorX& x, bool definiteness_fix = false);
	virtual void  EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets, bool definiteness_fix = false);
	virtual void  ApplyHessian(const VectorX& x, VectorX& b); // b = H*x, applying hessian for this block only;
	virtual void  EvaluateWeightedLaplacian(std::vector<SparseMatrixTriplet>& laplacian_triplets);
	virtual void  EvaluateWeightedDiagonal(std::vector<SparseMatrixTriplet>& diagonal_triplets);
	virtual void  EvaluateWeightedLaplacian1D(std::vector<SparseMatrixTriplet>& laplacian_1d_triplets);

protected:
	unsigned int m_p0;
	EigenVector3 m_g;
	EigenVector3 m_fixd_point;
	EigenMatrix3 m_H;

public:
	// for visualization and selection
	virtual void WriteToFileOBJ(std::ofstream& outfile, int& existing_vertices);
	virtual void WriteToFileOBJHead(std::ofstream& outfile);
	inline EigenVector3 GetFixedPoint() {return m_fixd_point;}
	inline void SetFixedPoint(const EigenVector3& target) {m_fixd_point = target;}
	inline unsigned int GetConstrainedVertexIndex() {return m_p0;}

private:
	Cube m_attachment_constraint_body;
};

class CollisionSpringConstraint : public Constraint
{
public:
	CollisionSpringConstraint(unsigned int p0, const EigenVector3& fixedpoint, const EigenVector3& normal);
	CollisionSpringConstraint(ScalarType stiffness, unsigned int p0, const EigenVector3& fixedpoint, const EigenVector3& normal);
	CollisionSpringConstraint(const CollisionSpringConstraint& other);
	virtual ~CollisionSpringConstraint();

	bool IsActive(const VectorX& x);
	virtual ScalarType  EvaluateEnergy(const VectorX& x);
	virtual ScalarType  GetEnergy();
	virtual void  EvaluateGradient(const VectorX& x, VectorX& gradient);
	virtual void  EvaluateGradient(const VectorX& x);
	virtual void  GetGradient(VectorX& gradient);
	virtual ScalarType  EvaluateEnergyAndGradient(const VectorX& x, VectorX& gradient);
	virtual ScalarType  EvaluateEnergyAndGradient(const VectorX& x);
	virtual ScalarType  GetEnergyAndGradient(VectorX& gradient);
	virtual void  EvaluateHessian(const VectorX& x, bool definiteness_fix = false);
	virtual void  EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets, bool definiteness_fix = false);
	virtual void  ApplyHessian(const VectorX& x, VectorX& b); // b = H*x, applying hessian for this block only;
	virtual void  EvaluateWeightedLaplacian(std::vector<SparseMatrixTriplet>& laplacian_triplets);
	virtual void  EvaluateWeightedLaplacian1D(std::vector<SparseMatrixTriplet>& laplacian_1d_triplets);

protected:
	unsigned int m_p0;
	EigenVector3 m_g;
	EigenVector3 m_fixed_point;
	EigenVector3 m_normal;
	EigenMatrix3 m_H;
};

class SpringConstraint : public Constraint
{
public:
	SpringConstraint(ConstraintType type, unsigned int p1, unsigned int p2, ScalarType length);
	SpringConstraint(unsigned int p1, unsigned int p2, ScalarType length);
	SpringConstraint(ScalarType stiffness, unsigned int p1, unsigned int p2, ScalarType length);
	SpringConstraint(const SpringConstraint& other);
	virtual ~SpringConstraint();

	virtual bool VertexIncluded(unsigned int vi) { return (m_p1 == vi || m_p2 == vi); }

	virtual ScalarType  EvaluateEnergy(const VectorX& x);
	virtual ScalarType  GetEnergy();
	virtual void  EvaluateGradient(const VectorX& x, VectorX& gradient);
	virtual void  EvaluateGradient(const VectorX& x);
	virtual void  GetGradient(VectorX& gradient);
	virtual ScalarType  EvaluateEnergyAndGradient(const VectorX& x, VectorX& gradient);
	virtual ScalarType  EvaluateEnergyAndGradient(const VectorX& x);
	virtual ScalarType  GetEnergyAndGradient(VectorX& gradient);
	virtual void  EvaluateHessian(const VectorX& x, bool definiteness_fix = false);
	virtual void  EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets, bool definiteness_fix = false);
	virtual void  ApplyHessian(const VectorX& x, VectorX& b); // b = H*x, applying hessian for this block only;
	virtual void  EvaluateWeightedLaplacian(std::vector<SparseMatrixTriplet>& laplacian_triplets);
	virtual void  EvaluateWeightedDiagonal(std::vector<SparseMatrixTriplet>& diagonal_triplets);
	virtual void  EvaluateWeightedLaplacian1D(std::vector<SparseMatrixTriplet>& laplacian_1d_triplets);

protected:
	unsigned int m_p1, m_p2;
	EigenVector3 m_g1, m_g2;
	EigenMatrix3 m_H;
	// rest length
	ScalarType m_rest_length;
};

class TetConstraint : public Constraint
{
public:
	TetConstraint(unsigned int p1, unsigned int p2, unsigned int p3, unsigned int p4, VectorX& x);
	TetConstraint(const TetConstraint& other);
	virtual ~TetConstraint();

	virtual bool VertexIncluded(unsigned int vi) { for (unsigned int i = 0; i != 4; i++) { return vi == m_p[i]; } return false; }

	virtual void GetMaterialProperty(MaterialType& type, ScalarType& mu, ScalarType& lambda, ScalarType& kappa);
	virtual void SetMaterialProperty(MaterialType type, ScalarType mu, ScalarType lambda, ScalarType kappa, ScalarType laplacian_coeff);
	virtual ScalarType ComputeLaplacianWeight();

	virtual ScalarType  EvaluateEnergy(const VectorX& x);
	virtual ScalarType  GetEnergy();
	virtual void  EvaluateGradient(const VectorX& x, VectorX& gradient);
	virtual void  EvaluateGradient(const VectorX& x);
	virtual void  GetGradient(VectorX& gradient);
	virtual ScalarType  EvaluateEnergyAndGradient(const VectorX& x, VectorX& gradient);
	virtual ScalarType  EvaluateEnergyAndGradient(const VectorX& x);
	virtual ScalarType  GetEnergyAndGradient(VectorX& gradient);
	virtual void  EvaluateHessian(const VectorX& x, bool definiteness_fix = false);
	virtual void  EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets, bool definiteness_fix = false);
	virtual void  ApplyHessian(const VectorX& x, VectorX& b); // b = H*x, applying hessian for this block only;
	virtual void  EvaluateFiniteDifferentHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets);
	virtual void  EvaluateWeightedLaplacian(std::vector<SparseMatrixTriplet>& laplacian_triplets);
	virtual void  EvaluateWeightedLaplacian1D(std::vector<SparseMatrixTriplet>& laplacian_1d_triplets);

	// accesser
	virtual ScalarType GetVolume(const VectorX& x);
	virtual void GetRotation(const VectorX& x, std::vector<EigenQuaternion>& rotations);

	// set mass matrix
	ScalarType SetMassMatrix(std::vector<SparseMatrixTriplet>& m, std::vector<SparseMatrixTriplet>& m_1d);

	// accesser
	//inline const ScalarType& Volume() { return m_W; }

public:
	// for ghost force detection and visualization
	void KeyHessianAndRotation(const VectorX& x, bool definiteness_fix = false);
	void KeyHessianAndRotation(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets, bool definiteness_fix = false);
	void KeyRotation(const VectorX& x);
	void GetRotationChange(const VectorX& x, std::vector<EigenQuaternion>& rotations);
	void ComputeGhostForce(const VectorX& x, const std::vector<EigenQuaternion>& rotations, bool stiffness_warped_matrix = true, bool outdated = true);
	virtual void WriteToFileOBJTet(std::ofstream& outfile, const VectorX& x);
	virtual void DrawTet(const VectorX& x, const VBO& vbos);

private:
	void getMatrixDs(EigenMatrix3& Dd, const VectorX& x);
	void getDeformationGradient(EigenMatrix3& F, const VectorX& x);
	void getStressTensor(EigenMatrix3& P, const EigenMatrix3& F, EigenMatrix3& R);
	ScalarType getStressTensorAndEnergyDensity(EigenMatrix3& P, const EigenMatrix3& F, EigenMatrix3& R);
	void singularValueDecomp(EigenMatrix3& U, EigenVector3& SIGMA, EigenMatrix3& V, const EigenMatrix3& A, bool signed_svd = true);
	void extractRotation(EigenMatrix3& R, const EigenMatrix3& A, bool signed_svd = true);
	void evaluateFDGradient(const VectorX& x, VectorX& gradient);

	void calculateDPDF(Matrix3333& dPdF, const EigenMatrix3& F);

protected:
	// material properties
	MaterialType m_material_type;
	ScalarType m_mu;
	ScalarType m_lambda;
	ScalarType m_kappa;
	ScalarType m_laplacian_coeff;

	unsigned int m_p[4]; // indices of four vertices
	EigenVector3 m_g[4];
	EigenMatrix12 m_H;
	//EigenMatrix3 m_H_blocks[4][4];
	EigenMatrix3 m_Dm; // [x1-x4|x2-x4|x3-x4]
	EigenMatrix3 m_Dm_inv; // inverse of m_Dm
	Eigen::Matrix<ScalarType, 3, 4> m_G; // Q = m_Dr^(-T) * IND;
	ScalarType m_W; // 1/6 det(Dr);

	// for neohookean inverted part
	const ScalarType m_neohookean_clamp_value = 0.1;

	// for ghost force detection and visualization
	EigenMatrix12 m_previous_hessian;
	EigenMatrix3 m_previous_rotation;
	EigenVector3 m_ghost_force;
};

#endif