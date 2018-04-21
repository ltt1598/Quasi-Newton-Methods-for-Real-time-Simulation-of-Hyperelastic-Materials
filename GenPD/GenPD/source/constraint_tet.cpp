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
#include "wunderSVD/svd3x3.h"
//#include "SVD"

#ifdef ENABLE_MATLAB_DEBUGGING
#include "matlab_debugger.h"
extern MatlabDebugger *g_debugger;
#endif

#define LOGJ_QUADRATIC_EXTENSION // comment this line to get linear extention
#define NEOHOOKEAN_PURE_I1_TERM

void vectorize9x1(const EigenMatrix3& src, VectorX& dst)
{
	dst.resize(9);

	dst.block<3, 1>(0, 0) = src.block<3, 1>(0, 0);
	dst.block<3, 1>(3, 0) = src.block<3, 1>(0, 1);
	dst.block<3, 1>(6, 0) = src.block<3, 1>(0, 2);
}

void reshape3x3(const VectorX& src, EigenMatrix3& dst)
{
	dst.block<3, 1>(0, 0) = src.block<3, 1>(0, 0);
	dst.block<3, 1>(0, 1) = src.block<3, 1>(3, 0);
	dst.block<3, 1>(0, 2) = src.block<3, 1>(6, 0);
}

void ThreeVector3ToMatrix3(EigenMatrix3& m, EigenVector3& v1, EigenVector3& v2, EigenVector3& v3)
{
	m.block<3, 1>(0, 0) = v1;
	m.block<3, 1>(0, 1) = v2;
	m.block<3, 1>(0, 2) = v3;
}

// neohookean functions
ScalarType neohookean_f(const ScalarType x, const ScalarType mu)
{
	return 0.5*mu*(x*x - 1);
}
ScalarType neohookean_df(const ScalarType x, const ScalarType mu)
{
	return mu*x;
}
ScalarType neohookean_d2f(const ScalarType mu)
{
	return mu;
}
ScalarType neohookean_h(const ScalarType x, const ScalarType mu, const ScalarType lambda)
{
	return -mu*log(x) + 0.5*lambda*(pow(log(x), 2));
}
ScalarType neohookean_dh(const ScalarType x, const ScalarType mu, const ScalarType lambda)
{
	return -mu / x + lambda*log(x) / x;
}
ScalarType neohookean_d2h(const ScalarType x, const ScalarType mu, const ScalarType lambda)
{
	return (mu + lambda * (1 - log(x))) / (x*x);
}

ScalarType neohookean_h_modified(const ScalarType x, const ScalarType x0, const ScalarType mu, const ScalarType lambda)
{
	assert(x0 > 0);
	if (x > x0)
	{
		return neohookean_h(x, mu, lambda);
	}
	else
	{
		ScalarType hx0 = neohookean_h(x0, mu, lambda);
		ScalarType dhx0 = neohookean_dh(x0, mu, lambda);
		ScalarType d2hx0 = neohookean_d2h(x0, mu, lambda);

		return hx0 + dhx0*(x - x0) + 0.5*d2hx0*std::pow((x - x0), 2);
	}
}
ScalarType neohookean_dh_modified(const ScalarType x, const ScalarType x0, const ScalarType mu, const ScalarType lambda)
{
	assert(x0 > 0);
	if (x > x0)
	{
		return neohookean_dh(x, mu, lambda);
	}
	else
	{
		ScalarType dhx0 = neohookean_dh(x0, mu, lambda);
		ScalarType d2hx0 = neohookean_d2h(x0, mu, lambda);

		return dhx0 + d2hx0*(x - x0);
	}
}
ScalarType neohookean_d2h_modified(const ScalarType x, const ScalarType x0, const ScalarType mu, const ScalarType lambda)
{
	assert(x0 > 0);
	if (x > x0)
	{
		return neohookean_d2h(x, mu, lambda);
	}
	else
	{
		ScalarType d2hx0 = neohookean_d2h(x0, mu, lambda);

		return d2hx0;
	}
}

//----------TetConstraint Class----------//
TetConstraint::TetConstraint(unsigned int p1, unsigned int p2, unsigned int p3, unsigned int p4, VectorX& x) :
Constraint(CONSTRAINT_TYPE_TET)
{
	m_p[0] = p1;
	m_p[1] = p2;
	m_p[2] = p3;
	m_p[3] = p4;

	EigenVector3 v1 = x.block_vector(p1) - x.block_vector(p4);
	EigenVector3 v2 = x.block_vector(p2) - x.block_vector(p4);
	EigenVector3 v3 = x.block_vector(p3) - x.block_vector(p4);

	ThreeVector3ToMatrix3(m_Dm, v1, v2, v3);

	m_W = m_Dm.determinant();

	m_W = 1.0 / 6.0 * std::abs(m_W);

	m_Dm_inv = m_Dm.inverse();

	Eigen::Matrix<ScalarType, 3, 4> IND;
	IND.block<3, 3>(0, 0) = EigenMatrix3::Identity();
	IND.block<3, 1>(0, 3) = EigenVector3(-1, -1, -1);

	m_G = m_Dm_inv.transpose() * IND;
}

TetConstraint::TetConstraint(const TetConstraint& other) :
Constraint(other)
{
	m_p[0] = other.m_p[0];
	m_p[1] = other.m_p[1];
	m_p[2] = other.m_p[2];
	m_p[3] = other.m_p[3];

	m_Dm = other.m_Dm;
	m_Dm_inv = other.m_Dm_inv;

	m_W = other.m_W;
	m_G = other.m_G;

	m_material_type = other.m_material_type;
	m_mu = other.m_mu;
	m_lambda = other.m_lambda;
	m_kappa = other.m_kappa;
	m_laplacian_coeff = other.m_laplacian_coeff;
}

TetConstraint::~TetConstraint()
{
}

void TetConstraint::getMatrixDs(EigenMatrix3& Dd, const VectorX& x)
{
	EigenVector3 v1 = x.block_vector(m_p[0]) - x.block_vector(m_p[3]);
	EigenVector3 v2 = x.block_vector(m_p[1]) - x.block_vector(m_p[3]);
	EigenVector3 v3 = x.block_vector(m_p[2]) - x.block_vector(m_p[3]);

	ThreeVector3ToMatrix3(Dd, v1, v2, v3);
}

void TetConstraint::GetMaterialProperty(MaterialType& type, ScalarType& mu, ScalarType& lambda, ScalarType& kappa)
{
	type = m_material_type;
	mu = m_mu;
	lambda = m_lambda;
	kappa = m_kappa;
}
void TetConstraint::SetMaterialProperty(MaterialType type, ScalarType mu, ScalarType lambda, ScalarType kappa, ScalarType laplacian_coeff)
{
	m_material_type = type;
	m_mu = mu;
	m_lambda = lambda;
	m_kappa = kappa;
	m_laplacian_coeff = laplacian_coeff;
}

ScalarType TetConstraint::ComputeLaplacianWeight()
{
	// read section 4.1 in our paper 
	switch (m_material_type)
	{
	case MATERIAL_TYPE_COROT:
		// 2mu (x-1) + lambda (x-1)
		m_laplacian_coeff = 2 * m_mu + m_lambda;
		break;
	case MATERIAL_TYPE_StVK:
		// mu * (x^2  - 1) + 0.5lambda * (x^3 - x)
		// 10% window
		m_laplacian_coeff = 2 * m_mu + 1.0033 * m_lambda;
		//// 20% window
		//m_laplacian_coeff = 2 * m_mu + 1.0126 * m_lambda;
		break;
	case MATERIAL_TYPE_NEOHOOKEAN_EXTEND_LOG:
		// mu * x - mu / x + lambda * log(x) / x
		// 10% window
		m_laplacian_coeff = 2.0066 * m_mu + 1.0122 * m_lambda;
		//// 20% window
		//m_laplacian_coeff = 2.0260 * m_mu + 1.0480 * m_lambda;
		break;
	default:
		break;
	}
	return m_laplacian_coeff;
}

// 0.5*mu*||F-R||^2
ScalarType TetConstraint::EvaluateEnergy(const VectorX& x)
{
	EigenMatrix3 F;
	getDeformationGradient(F, x);

	ScalarType e_this = 0;
	switch (m_material_type)
	{
	case MATERIAL_TYPE_COROT:
	{
		EigenMatrix3 U;
		EigenMatrix3 V;
		EigenVector3 SIGMA;
		singularValueDecomp(U, SIGMA, V, F);

		EigenMatrix3 R = U*V.transpose();

		e_this = m_mu*(F - R).squaredNorm() + 0.5*m_lambda*std::pow((R.transpose()*F).trace() - 3, 2);
	}
	break;
	case MATERIAL_TYPE_StVK:
	{
		EigenMatrix3 I = EigenMatrix3::Identity();
		EigenMatrix3 E = 0.5*(F.transpose()*F - I);
		e_this = m_mu*E.squaredNorm() + 0.5*m_lambda*std::pow(E.trace(), 2);
		ScalarType J = F.determinant();
		if (J < 1)
		{
			e_this += m_kappa / 12 * std::pow((1 - J) / 6, 3);
		}
	}
	break;
	case MATERIAL_TYPE_NEOHOOKEAN_EXTEND_LOG:
	{
		EigenMatrix3 FtF = F.transpose() * F;
		ScalarType I1 = FtF.trace();
		ScalarType J = F.determinant();
		e_this = 0.5*m_mu*(I1 - 3);
		ScalarType logJ;
		const ScalarType& J0 = m_neohookean_clamp_value;
		if (J > J0)
		{
			logJ = std::log(J);
			e_this += -m_mu*logJ + 0.5*m_lambda*logJ*logJ;
		}
		else
		{
#ifdef LOGJ_QUADRATIC_EXTENSION
			ScalarType fJ = log(J0) + (J - J0) / J0 - 0.5*std::pow((J / J0 - 1), 2);
#else
			ScalarType fJ = log(J0) + (J - J0) / J0;
#endif
			e_this += -m_mu*fJ + 0.5*m_lambda*fJ*fJ;
		}
	}
	break;
	}

	e_this *= m_W;

	m_energy = e_this;

	return e_this;
}

ScalarType  TetConstraint::GetEnergy()
{
	return m_energy;
}

void TetConstraint::EvaluateGradient(const VectorX& x, VectorX& gradient)
{
	//VectorX gradient1 = gradient;
	//evaluateFDGradient(x, gradient1);

	//gradient = gradient1;
	//return;

	EigenMatrix3 F;
	getDeformationGradient(F, x);

	EigenMatrix3 P;
	EigenMatrix3 R;
	getStressTensor(P, F, R);

	EigenMatrix3 H = m_W * P * m_Dm_inv.transpose();

	EigenVector3 g[4];
	g[0] = H.block<3, 1>(0, 0);
	g[1] = H.block<3, 1>(0, 1);
	g[2] = H.block<3, 1>(0, 2);
	g[3] = -g[0] - g[1] - g[2];

	for (unsigned int i = 0; i < 4; i++)
	{
		gradient.block_vector(m_p[i]) += g[i];
	}
}

void TetConstraint::EvaluateGradient(const VectorX& x)
{
	EigenMatrix3 F;
	getDeformationGradient(F, x);

	EigenMatrix3 P;
	EigenMatrix3 R;
	getStressTensor(P, F, R);

	EigenMatrix3 H = m_W * P * m_Dm_inv.transpose();

	m_g[0] = H.block<3, 1>(0, 0);
	m_g[1] = H.block<3, 1>(0, 1);
	m_g[2] = H.block<3, 1>(0, 2);
	m_g[3] = -m_g[0] - m_g[1] - m_g[2];
}

void TetConstraint::GetGradient(VectorX& gradient)
{
	for (unsigned int i = 0; i < 4; i++)
	{
		gradient.block_vector(m_p[i]) += m_g[i];
	}
}
ScalarType TetConstraint::EvaluateEnergyAndGradient(const VectorX& x, VectorX& gradient)
{
	EvaluateEnergyAndGradient(x);

	for (unsigned int i = 0; i < 4; i++)
	{
		gradient.block_vector(m_p[i]) += m_g[i];
	}
	return m_energy;
}
ScalarType TetConstraint::EvaluateEnergyAndGradient(const VectorX& x)
{
	EigenMatrix3 F;
	getDeformationGradient(F, x);

	EigenMatrix3 P;
	EigenMatrix3 R;
	ScalarType e_this = getStressTensorAndEnergyDensity(P, F, R);
	m_energy = e_this * m_W;

	EigenMatrix3 H = m_W * P * m_Dm_inv.transpose();

	m_g[0] = H.block<3, 1>(0, 0);
	m_g[1] = H.block<3, 1>(0, 1);
	m_g[2] = H.block<3, 1>(0, 2);
	m_g[3] = -m_g[0] - m_g[1] - m_g[2];

	return m_energy;
}
ScalarType TetConstraint::GetEnergyAndGradient(VectorX& gradient)
{
	for (unsigned int i = 0; i < 4; i++)
	{
		gradient.block_vector(m_p[i]) += m_g[i];
	}
	return m_energy;
}

void TetConstraint::EvaluateHessian(const VectorX& x, bool definiteness_fix)
{
	EigenMatrix3 F;
	getDeformationGradient(F, x);

	Matrix3333 dPdF;

	calculateDPDF(dPdF, F);

	Matrix3333 dH;
	dH = m_W * dPdF * m_Dm_inv.transpose();

	//EigenMatrix3 H_blocks[4][4];
	EigenMatrix3 H_one_block;

	// i == 0 to 2 case 
	for (unsigned int i = 0; i < 3; i++)
	{
		// j == 0 to 2 case
		for (unsigned int j = 0; j < 3; j++)
		{
			EigenVector3 v0 = dH(0, j)*m_Dm_inv.transpose().block<3, 1>(0, i);
			EigenVector3 v1 = dH(1, j)*m_Dm_inv.transpose().block<3, 1>(0, i);
			EigenVector3 v2 = dH(2, j)*m_Dm_inv.transpose().block<3, 1>(0, i);

			ThreeVector3ToMatrix3(H_one_block, v0, v1, v2);
			m_H.block<3, 3>(i * 3, j * 3) = H_one_block;
		}
		// j == 3 case
		//H_blocks[i][3] = -H_blocks[i][0] - H_blocks[i][1] - H_blocks[i][2];
		m_H.block<3, 3>(i * 3, 9) = -m_H.block<3, 3>(i * 3, 0) - m_H.block<3, 3>(i * 3, 3) - m_H.block<3, 3>(i * 3, 6);
	}
	// i == 3 case
	for (unsigned int j = 0; j < 4; j++)
	{
		//H_blocks[3][j] = -H_blocks[0][j]-H_blocks[1][j]-H_blocks[2][j];
		m_H.block<3, 3>(9, j * 3) = -m_H.block<3, 3>(0, j * 3) - m_H.block<3, 3>(3, j * 3) - m_H.block<3, 3>(6, j * 3);
	}

	if (definiteness_fix)
	{
		// definiteness fix
		Eigen::EigenSolver<EigenMatrix12> evd;
		evd.compute(m_H);
		EigenMatrix12 Q = evd.eigenvectors().real();
		VectorX LAMBDA = evd.eigenvalues().real();
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

		//// debug
		//evd.compute(m_H);
		//Q = evd.eigenvectors().real();
		//LAMBDA = evd.eigenvalues().real();	
	}
}

void TetConstraint::EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets, bool definiteness_fix)
{
	//// finite difference test
	//std::vector<SparseMatrixTriplet> hessian_triplets_copy = hessian_triplets;
	//EvaluateFiniteDifferentHessian(x, hessian_triplets_copy);

	EvaluateHessian(x, definiteness_fix);

	// set to triplets
	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 0; j < 4; j++)
		{
			//EigenMatrix3 block = H_blocks.block<3, 3>(i * 3, j * 3);
			for (unsigned int row = 0; row < 3; row++)
			{
				for (unsigned int col = 0; col < 3; col++)
				{
					hessian_triplets.push_back(SparseMatrixTriplet(m_p[i] * 3 + row, m_p[j] * 3 + col, m_H(i * 3 + row, j * 3 + col)));
				}
			}
		}
	}
}

void TetConstraint::ApplyHessian(const VectorX& x, VectorX& b)
{
	EigenVector12 x_blocks;
	for (unsigned int i = 0; i != 4; i++)
	{
		x_blocks.block_vector(i) = x.block_vector(m_p[i]);
	}
	EigenVector12 b_blocks = m_H * x_blocks;
	for (unsigned int i = 0; i != 4; i++)
	{
		b.block_vector(m_p[i]) += b_blocks.block_vector(i);
	}
}

void TetConstraint::EvaluateFiniteDifferentHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets)
{
	unsigned int size = x.size();
	VectorX gradient(size);
	VectorX gradient_prime(size);
	gradient.setZero();
	EvaluateGradient(x, gradient);

	EigenVector3 dt[3];
	dt[0] = EigenVector3(1e-6, 0, 0);
	dt[1] = EigenVector3(0, 1e-6, 0);
	dt[2] = EigenVector3(0, 0, 1e-6);
	VectorX x_plus_t;

	EigenMatrix3 H_blocks[4][4];
	// finite difference p0, x
	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 0; j < 4; j++)
		{
			EigenVector3 temp[3];
			for (unsigned int dim = 0; dim < 3; dim++)
			{
				x_plus_t = x;
				x_plus_t.block_vector(m_p[j]) += dt[dim];
				gradient_prime.setZero();
				EvaluateGradient(x_plus_t, gradient_prime);
				temp[dim] = (gradient_prime.block_vector(m_p[i]) - gradient.block_vector(m_p[i]))*1e6;
			}
			ThreeVector3ToMatrix3(H_blocks[i][j], temp[0], temp[1], temp[2]);
		}
	}

	// set to triplets
	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 0; j < 4; j++)
		{
			EigenMatrix3& block = H_blocks[i][j];
			for (unsigned int row = 0; row < 3; row++)
			{
				for (unsigned int col = 0; col < 3; col++)
				{
					hessian_triplets.push_back(SparseMatrixTriplet(m_p[i] * 3 + row, m_p[j] * 3 + col, block(row, col)));
				}
			}
		}
	}
}

void TetConstraint::EvaluateWeightedLaplacian(std::vector<SparseMatrixTriplet>& laplacian_triplets)
{
	ScalarType ks = m_laplacian_coeff* m_W;

	Matrix3333 Identity_tensor;
	Identity_tensor.SetIdentity();

	Matrix3333 dH;
	dH = ks * Identity_tensor * m_Dm_inv.transpose();

	EigenMatrix3 H_blocks[4][4];

	// i == 0 to 2 case 
	for (unsigned int i = 0; i < 3; i++)
	{
		// j == 0 to 2 case
		for (unsigned int j = 0; j < 3; j++)
		{
			EigenVector3 v0 = dH(0, j)*m_Dm_inv.transpose().block<3, 1>(0, i);
			EigenVector3 v1 = dH(1, j)*m_Dm_inv.transpose().block<3, 1>(0, i);
			EigenVector3 v2 = dH(2, j)*m_Dm_inv.transpose().block<3, 1>(0, i);

			ThreeVector3ToMatrix3(H_blocks[i][j], v0, v1, v2);
		}
		// j == 3 case
		H_blocks[i][3] = -H_blocks[i][0] - H_blocks[i][1] - H_blocks[i][2];
	}
	// i == 3 case
	for (unsigned int j = 0; j < 4; j++)
	{
		H_blocks[3][j] = -H_blocks[0][j] - H_blocks[1][j] - H_blocks[2][j];
	}

	// set to triplets
	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 0; j < 4; j++)
		{
			EigenMatrix3& block = H_blocks[i][j];
			for (unsigned int row = 0; row < 3; row++)
			{
				for (unsigned int col = 0; col < 3; col++)
				{
					laplacian_triplets.push_back(SparseMatrixTriplet(m_p[i] * 3 + row, m_p[j] * 3 + col, block(row, col)));
				}
			}
		}
	}
}

void TetConstraint::EvaluateWeightedLaplacian1D(std::vector<SparseMatrixTriplet>& laplacian_1d_triplets)
{
	ScalarType ks = m_laplacian_coeff* m_W;

	EigenMatrix4 L = ks * m_G.transpose() * m_G;

	// set to triplets
	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 0; j < 4; j++)
		{
			laplacian_1d_triplets.push_back(SparseMatrixTriplet(m_p[i], m_p[j], L(i, j)));
		}
	}
}

ScalarType TetConstraint::GetVolume(const VectorX& x)
{
	EigenMatrix3 F;
	getDeformationGradient(F, x);

	return (F.determinant() / 6.0);
}

void TetConstraint::GetRotation(const VectorX& x, std::vector<EigenQuaternion>& rotations)
{
	EigenMatrix3 F;
	getDeformationGradient(F, x);

	EigenMatrix3 U;
	EigenMatrix3 V;
	EigenVector3 SIGMA;
	singularValueDecomp(U, SIGMA, V, F);

	EigenMatrix3 R = U*V.transpose();

	EigenQuaternion q(R);

	for (unsigned int i = 0; i != 4; i++)
	{
		rotations[m_p[i]].vec() += q.vec();
		rotations[m_p[i]].w() += q.w();
	}
}

void TetConstraint::calculateDPDF(Matrix3333& dPdF, const EigenMatrix3& F)
{
	Matrix3333 dFdF;
	dFdF.SetIdentity();
	switch (m_material_type)
	{
	case MATERIAL_TYPE_COROT:
	{
		EigenMatrix3 U;
		EigenMatrix3 V;
		EigenVector3 SIGMA;
		singularValueDecomp(U, SIGMA, V, F);

		EigenMatrix3 R = U*V.transpose();

		Matrix3333 dRdF;

		// to compute dRdF using derivative of SVD
		for (unsigned int i = 0; i < 3; i++)
		{
			for (unsigned int j = 0; j < 3; j++)
			{
				EigenMatrix3 deltaF = dFdF(i, j);

				EigenMatrix3 A = U.transpose() * deltaF * V;

				// special case for uniform scaling including restpose, restpose is a common pose
				if (abs(SIGMA(0) - SIGMA(1)) < LARGER_EPSILON && abs(SIGMA(0) - SIGMA(2)) < LARGER_EPSILON)
				{
					ScalarType alpha = SIGMA(0); // SIGMA = alpha * Identity
					if (alpha < LARGER_EPSILON) // alpha should be greater than zero because it comes from the ordered SVD
					{
						alpha = LARGER_EPSILON;
					}

					EigenMatrix3 off_diag_A;
					off_diag_A.setZero();
					for (unsigned int row = 0; row != 3; row++)
					{
						for (unsigned int col = 0; col != 3; col++)
						{
							// assign off diagonal values of U^T * delta*F * V
							if (row == col)
								continue;
							else
							{
								off_diag_A(row, col) = A(row, col) / alpha;
							}
						}
					}
					dRdF(i, j) = U*off_diag_A*V.transpose();
				}
				// otherwise TODO: should also discuss the case where 2 sigular values are the same, but since it's very rare, we are gonna just treat it using regularization
				else
				{
					// there are 9 unkown variables, u10, u20, u21, v10, v20, v21, sig00, sig11, sig22
					EigenVector2 unknown_side, known_side;
					EigenMatrix2 known_matrix;
					EigenMatrix3 U_tilde, V_tilde;
					U_tilde.setZero(); V_tilde.setZero();
					EigenMatrix2 reg;
					reg.setZero();
					reg(0, 0) = reg(1, 1) = LARGER_EPSILON;
					for (unsigned int row = 0; row < 3; row++)
					{
						for (unsigned int col = 0; col < row; col++)
						{
							known_side = EigenVector2(A(col, row), A(row, col));
							known_matrix.block<2, 1>(0, 0) = EigenVector2(-SIGMA[row], SIGMA[col]);
							known_matrix.block<2, 1>(0, 1) = EigenVector2(-SIGMA[col], SIGMA[row]);

							if (std::abs(SIGMA[row] - SIGMA[col]) < LARGER_EPSILON) //regularization
							{
								//throw std::exception("Ill-conditioned hessian matrix, using FD hessian.");
								known_matrix += reg;
							}
							else
							{
								assert(std::abs(known_matrix.determinant()) > 1e-6);
							}

							unknown_side = known_matrix.inverse() * known_side;
							EigenVector2 test_vector = known_matrix*unknown_side;
							U_tilde(row, col) = unknown_side[0];
							U_tilde(col, row) = -U_tilde(row, col);
							V_tilde(row, col) = unknown_side[1];
							V_tilde(col, row) = -V_tilde(row, col);
						}
					}
					EigenMatrix3 deltaU = U*U_tilde;
					EigenMatrix3 deltaV = V_tilde*V.transpose();

					dRdF(i, j) = deltaU*V.transpose() + U*deltaV;
				}
			}
		}

		dPdF = 2 * m_mu* (dFdF - dRdF); // first mu term

		Matrix3333 dlambda_term_dR;
		Matrix3333 dlambda_term_dRF;
		Matrix3333 R_kron_R;
		directProduct(R_kron_R, R, R);
		Matrix3333 F_kron_R;
		directProduct(F_kron_R, F, R);
		dlambda_term_dR = F_kron_R + ((R.transpose()*F).trace() - 3) * dFdF/*dFdF = Identity*/;
		dlambda_term_dRF = dlambda_term_dR.Contract(dRdF);

		dPdF = dPdF + (R_kron_R + dlambda_term_dRF) * m_lambda;
	}
	break;
	case MATERIAL_TYPE_StVK:
	{
		EigenMatrix3 E = 0.5 * (F.transpose()*F - EigenMatrix3::Identity());
		EigenMatrix3 deltaE;
		EigenMatrix3 deltaF;
		ScalarType J = F.determinant();
		EigenMatrix3 Finv = F.inverse();
		EigenMatrix3 FinvT = Finv.transpose();

		for (unsigned int i = 0; i < 3; i++)
		{
			for (unsigned int j = 0; j < 3; j++)
			{
				deltaF = dFdF(i, j);
				deltaE = 0.5 * (deltaF.transpose() * F + F.transpose() * deltaF);

				dPdF(i, j) = deltaF * (2 * m_mu * E + m_lambda * E.trace() * EigenMatrix3::Identity()) + F * (2 * m_mu * deltaE + m_lambda*deltaE.trace()*EigenMatrix3::Identity());
				if (J < 1)
				{
					ScalarType one_minus_J_over_six = (1 - J) / 6.0;
					ScalarType one_minus_J_over_six_square = one_minus_J_over_six*one_minus_J_over_six;

					dPdF(i, j) += -m_kappa / 24 * ((-one_minus_J_over_six*J / 3 + one_minus_J_over_six_square)*J*(Finv*deltaF).trace()*FinvT - one_minus_J_over_six_square*J*FinvT*deltaF.transpose()*FinvT);
				}
			}
		}
	}
	break;
	case MATERIAL_TYPE_NEOHOOKEAN_EXTEND_LOG:
	{
		EigenMatrix3 deltaF;
		ScalarType J = F.determinant();
		EigenMatrix3 Finv = F.inverse();  // assert J != 0;
		EigenMatrix3 FinvT = Finv.transpose();
		ScalarType logJ;

		for (unsigned int i = 0; i < 3; i++)
		{
			for (unsigned int j = 0; j < 3; j++)
			{
				deltaF = dFdF(i, j);

				dPdF(i, j) = m_mu * deltaF;

				const ScalarType& J0 = m_neohookean_clamp_value;
				if (J > J0)
				{
					logJ = std::log(J);

					dPdF(i, j) += (m_mu - m_lambda*logJ)*FinvT*deltaF.transpose()*FinvT + m_lambda*(Finv*deltaF).trace() * FinvT;
				}
				else
				{
#ifdef LOGJ_QUADRATIC_EXTENSION
					// quadratic
					ScalarType fJ = log(J0) + (J - J0) / J0 - 0.5*std::pow((J / J0 - 1), 2);
					ScalarType dfJdJ = 1.0 / J0 - (J - J0) / (J0*J0);
					ScalarType d2fJdJ2 = -1.0 / (J0*J0);
#else
					ScalarType fJ = log(J0) + (J - J0) / J0;
					ScalarType dfJdJ = 1.0 / J0;
					ScalarType d2fJdJ2 = 0;
#endif
					EigenMatrix3 FinvTdFTFinvT = FinvT*deltaF.transpose()*FinvT;
					EigenMatrix3 FinvdFtraceFinvT = (Finv*deltaF).trace() * FinvT;

					dPdF(i, j) += -m_mu * (d2fJdJ2*J + dfJdJ) * J * FinvdFtraceFinvT;
					dPdF(i, j) += m_mu * (dfJdJ*J) * FinvTdFTFinvT;
					dPdF(i, j) += m_lambda * (dfJdJ*dfJdJ*J + fJ*(d2fJdJ2*J + dfJdJ)) * J * FinvdFtraceFinvT;
					dPdF(i, j) += -m_lambda * (fJ*dfJdJ*J) * FinvTdFTFinvT;
				}
			}
		}
	}
	break;
	}
}

void TetConstraint::getDeformationGradient(EigenMatrix3& F, const VectorX& x)
{
	EigenMatrix3 Ds;
	getMatrixDs(Ds, x);
	F = Ds * m_Dm_inv;
}

void TetConstraint::getStressTensor(EigenMatrix3& P, const EigenMatrix3& F, EigenMatrix3& R)
{
	switch (m_material_type)
	{
	case MATERIAL_TYPE_COROT:
	{
		EigenMatrix3 U;
		EigenMatrix3 V;
		EigenVector3 SIGMA;
		singularValueDecomp(U, SIGMA, V, F);

		R = U*V.transpose();

		P = 2 * m_mu * (F - R) + m_lambda * ((R.transpose()*F).trace() - 3) * R;
	}
	break;
	case MATERIAL_TYPE_StVK:
	{
		EigenMatrix3 I = EigenMatrix3::Identity();
		EigenMatrix3 E = 0.5*(F.transpose()*F - I);
		P = F * (2 * m_mu*E + m_lambda*E.trace() * I);
		ScalarType J = F.determinant();
		if (J < 1)
		{
			P += -m_kappa / 24 * std::pow((1 - J) / 6, 2) * J * F.inverse().transpose();
		}
	}
	break;
	case MATERIAL_TYPE_NEOHOOKEAN_EXTEND_LOG:
	{
		ScalarType J = F.determinant();

		P = m_mu*F;
		ScalarType logJ;
		const ScalarType& J0 = m_neohookean_clamp_value;
		if (J > J0)
		{
			logJ = std::log(J);
			EigenMatrix3 FinvT = F.inverse().transpose(); // F is invertible because J > 0
			P += m_mu*(-FinvT) + m_lambda*logJ*FinvT;
		}
		else
		{
#ifdef LOGJ_QUADRATIC_EXTENSION
			ScalarType fJ = log(J0) + (J - J0) / J0 - 0.5*std::pow((J / J0 - 1), 2);
			ScalarType dfJdJ = 1.0 / J0 - (J - J0) / (J0*J0);
#else
			ScalarType fJ = log(J0) + (J - J0) / J0;
			ScalarType dfJdJ = 1.0 / J0;
#endif
			//ScalarType fJ = log(J);
			//ScalarType dfJdJ = 1 / J;
			EigenMatrix3 FinvT = F.inverse().transpose(); // TODO: here F is nolonger guaranteed to be invertible....
			P += -m_mu * dfJdJ * J * FinvT + m_lambda* fJ * dfJdJ * J * FinvT;
		}
	}
	break;
	default:
		break;
	}
}

ScalarType TetConstraint::getStressTensorAndEnergyDensity(EigenMatrix3& P, const EigenMatrix3& F, EigenMatrix3& R)
{
	ScalarType e_this = 0;
	switch (m_material_type)
	{
	case MATERIAL_TYPE_COROT:
	{
		EigenMatrix3 U;
		EigenMatrix3 V;
		EigenVector3 SIGMA;
		singularValueDecomp(U, SIGMA, V, F);

		R = U*V.transpose();

		P = 2 * m_mu * (F - R) + m_lambda * ((R.transpose()*F).trace() - 3) * R;

		e_this = m_mu*(F - R).squaredNorm() + 0.5*m_lambda*std::pow((R.transpose()*F).trace() - 3, 2);
	}
	break;
	case MATERIAL_TYPE_StVK:
	{
		EigenMatrix3 I = EigenMatrix3::Identity();
		EigenMatrix3 E = 0.5*(F.transpose()*F - I);
		P = F * (2 * m_mu*E + m_lambda*E.trace() * I);
		e_this = m_mu*E.squaredNorm() + 0.5*m_lambda*std::pow(E.trace(), 2);
		ScalarType J = F.determinant();
		if (J < 1)
		{
			P += -m_kappa / 24 * std::pow((1 - J) / 6, 2) * J * F.inverse().transpose();
			e_this += m_kappa / 12 * std::pow((1 - J) / 6, 3);
		}
	}
	break;
	case MATERIAL_TYPE_NEOHOOKEAN_EXTEND_LOG:
	{
		EigenMatrix3 FtF = F.transpose() * F;
		ScalarType I1 = FtF.trace();
		ScalarType J = F.determinant();

		P = m_mu*F;
		e_this = 0.5*m_mu*(I1 - 3);
		ScalarType logJ;
		const ScalarType& J0 = m_neohookean_clamp_value;
		if (J > J0)
		{
			logJ = std::log(J);
			EigenMatrix3 FinvT = F.inverse().transpose(); // F is invertible because J > 0
			P += m_mu*(-FinvT) + m_lambda*logJ*FinvT;
			e_this += -m_mu*logJ + 0.5*m_lambda*logJ*logJ;
		}
		else
		{
#ifdef LOGJ_QUADRATIC_EXTENSION
			ScalarType fJ = log(J0) + (J - J0) / J0 - 0.5*std::pow((J / J0 - 1), 2);
			ScalarType dfJdJ = 1.0 / J0 - (J - J0) / (J0*J0);
#else
			ScalarType fJ = log(J0) + (J - J0) / J0;
			ScalarType dfJdJ = 1.0 / J0;
#endif
			//ScalarType fJ = log(J);
			//ScalarType dfJdJ = 1 / J;
			EigenMatrix3 FinvT = F.inverse().transpose(); // TODO: here F is nolonger guaranteed to be invertible....
			P += -m_mu * dfJdJ * J * FinvT + m_lambda* fJ * dfJdJ * J * FinvT;
			e_this += -m_mu*fJ + 0.5*m_lambda*fJ*fJ;
		}
	}
	break;
	default:
		break;
	}

	return e_this;
}

void TetConstraint::singularValueDecomp(EigenMatrix3& U, EigenVector3& SIGMA, EigenMatrix3& V, const EigenMatrix3& A, bool signed_svd)
{
#include "Eigen/SVD"
	//Eigen Jacobi SVD
	Eigen::JacobiSVD<EigenMatrix3> svd;
	svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

	U = svd.matrixU();
	V = svd.matrixV();
	SIGMA = svd.singularValues();

	if (signed_svd)
	{
		ScalarType detU = U.determinant();
		ScalarType detV = V.determinant();
		if (detU < 0)
		{
			U.block<3, 1>(0, 2) *= -1;
			SIGMA[2] *= -1;
		}
		if (detV < 0)
		{
			V.block<3, 1>(0, 2) *= -1;
			SIGMA[2] *= -1;
		}
	}

	//	// Eftychios' Wunder SVD, only support single floating points
	//	// Will cause some problem that the FD hessian does not match the hessian if you use it. (For example, in corot energy)
	//#ifdef HIGH_PRECISION
	//	Eigen::Matrix3f Af = A.cast<float>();
	//	Eigen::Matrix3f Uf;
	//	Eigen::Vector3f Sf;
	//	Eigen::Matrix3f Vf;
	//
	//	igl::svd3x3(Af, Uf, Sf, Vf);
	//
	//	U = Uf.cast<ScalarType>();
	//	SIGMA = Sf.cast<ScalarType>();
	//	V = Vf.cast<ScalarType>();
	//#else
	//	igl::svd3x3(A, U, SIGMA, V);
	//#endif
}

void TetConstraint::extractRotation(EigenMatrix3& R, const EigenMatrix3& A, bool signed_svd)
{
	EigenMatrix3 U;
	EigenMatrix3 V;
	EigenVector3 SIGMA;
	singularValueDecomp(U, SIGMA, V, A, signed_svd);

	R = U*V.transpose();
}

void TetConstraint::evaluateFDGradient(const VectorX& x, VectorX& gradient)
{
	ScalarType energy = 0;
	ScalarType energy_prime;

	energy = EvaluateEnergy(x);

	EigenVector3 dt[3];
	dt[0] = EigenVector3(1e-5, 0, 0);
	dt[1] = EigenVector3(0, 1e-5, 0);
	dt[2] = EigenVector3(0, 0, 1e-5);
	VectorX x_plus_t;

	// finite difference p0, x
	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int dim = 0; dim < 3; dim++)
		{
			x_plus_t = x;
			x_plus_t.block_vector(m_p[i]) += dt[dim];
			energy_prime = EvaluateEnergy(x_plus_t);
			gradient.block_vector(m_p[i])[dim] += (energy_prime - energy)*1e5;
		}
	}
}

ScalarType TetConstraint::SetMassMatrix(std::vector<SparseMatrixTriplet>& m, std::vector<SparseMatrixTriplet>& m_1d)
{
	ScalarType W_inv = 1.0 / m_W;
	for (unsigned i = 0; i != 4; i++)
	{
		// mass
		m.push_back(SparseMatrixTriplet(3 * m_p[i], 3 * m_p[i], 0.25*m_W));
		m.push_back(SparseMatrixTriplet(3 * m_p[i] + 1, 3 * m_p[i] + 1, 0.25*m_W));
		m.push_back(SparseMatrixTriplet(3 * m_p[i] + 2, 3 * m_p[i] + 2, 0.25*m_W));

		// mass_1d
		m_1d.push_back(SparseMatrixTriplet(m_p[i], m_p[i], 0.25*m_W));
	}

	return m_W;
}

void TetConstraint::KeyHessianAndRotation(const VectorX& x, bool definiteness_fix)
{
	EvaluateHessian(x, definiteness_fix);
	m_previous_hessian = m_H;

	EigenMatrix3 F;
	getDeformationGradient(F, x);

	EigenMatrix3 U;
	EigenMatrix3 V;
	EigenVector3 SIGMA;
	singularValueDecomp(U, SIGMA, V, F);

	m_previous_rotation = U*V.transpose();
}

void TetConstraint::KeyHessianAndRotation(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets, bool definiteness_fix)
{
	EvaluateHessian(x, definiteness_fix);
	m_previous_hessian = m_H;

	EigenMatrix3 F;
	getDeformationGradient(F, x);

	EigenMatrix3 U;
	EigenMatrix3 V;
	EigenVector3 SIGMA;
	singularValueDecomp(U, SIGMA, V, F);

	m_previous_rotation = U*V.transpose();

	// set to triplets
	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 0; j < 4; j++)
		{
			//EigenMatrix3 block = H_blocks.block<3, 3>(i * 3, j * 3);
			for (unsigned int row = 0; row < 3; row++)
			{
				for (unsigned int col = 0; col < 3; col++)
				{
					hessian_triplets.push_back(SparseMatrixTriplet(m_p[i] * 3 + row, m_p[j] * 3 + col, m_H(i * 3 + row, j * 3 + col)));
				}
			}
		}
	}
}

void TetConstraint::KeyRotation(const VectorX& x)
{
	EigenMatrix3 F;
	getDeformationGradient(F, x);

	EigenMatrix3 U;
	EigenMatrix3 V;
	EigenVector3 SIGMA;
	singularValueDecomp(U, SIGMA, V, F);

	m_previous_rotation = U*V.transpose();
}

void TetConstraint::GetRotationChange(const VectorX& x, std::vector<EigenQuaternion>& rotations)
{
	EigenMatrix3 F;
	getDeformationGradient(F, x);

	EigenMatrix3 U;
	EigenMatrix3 V;
	EigenVector3 SIGMA;
	singularValueDecomp(U, SIGMA, V, F);

	EigenMatrix3 R_change = U*V.transpose() * m_previous_rotation.transpose();

	EigenQuaternion q_change(R_change);

	for (unsigned int i = 0; i != 4; i++)
	{
		rotations[m_p[i]].vec() += q_change.vec();
		rotations[m_p[i]].w() += q_change.w();
	}
}
void TetConstraint::ComputeGhostForce(const VectorX& x, const std::vector<EigenQuaternion>& rotations, bool stiffness_warped_matrix, bool outdated)
{
	if (!outdated)
	{
		m_ghost_force.setZero();
	}
	else
	{
		// compute force (or gradient)
		EvaluateGradient(x);
		EigenVector12 g;

		// compute per-nodal rotation
		EigenMatrix12 nodal_R_change;
		nodal_R_change.setIdentity();

		// stiffness_warp case
		if (stiffness_warped_matrix)
		{
			for (unsigned int i = 0; i != 4; i++)
			{
				// normalized the quaternion, AKA QLERP
				EigenQuaternion q = rotations[i].normalized();
				// get the rotation matrix from quaternion
				EigenMatrix3 r(q);

				nodal_R_change.block<3, 3>(i * 3, i * 3) = r;
				g.block<3, 1>(i * 3, 0) = m_g[i];
			}
		}

		EigenMatrix12 I;
		I.setIdentity();

		EigenMatrix12 H_tilde = nodal_R_change * (m_H + I) * nodal_R_change.transpose();

		EigenVector12 total_force = H_tilde.inverse() * g;
		m_ghost_force = total_force.block_vector(0) + total_force.block_vector(1) + total_force.block_vector(2) + total_force.block_vector(3);
	}
}

void TetConstraint::WriteToFileOBJTet(std::ofstream& outfile, const VectorX& x)
{
	if (m_ghost_force.norm() > 0.4)
	{
		EigenMatrix3 Dd;
		getMatrixDs(Dd, x);
		if (Dd.determinant() < 0)
		{
			outfile << "f " << m_p[3] + 1 << " " << m_p[2] + 1 << " " << m_p[0] + 1 << std::endl;
			outfile << "f " << m_p[3] + 1 << " " << m_p[1] + 1 << " " << m_p[2] + 1 << std::endl;
			outfile << "f " << m_p[3] + 1 << " " << m_p[0] + 1 << " " << m_p[1] + 1 << std::endl;
			outfile << "f " << m_p[0] + 1 << " " << m_p[2] + 1 << " " << m_p[1] + 1 << std::endl;
		}
		else
		{
			outfile << "f " << m_p[3] + 1 << " " << m_p[0] + 1 << " " << m_p[2] + 1 << std::endl;
			outfile << "f " << m_p[3] + 1 << " " << m_p[2] + 1 << " " << m_p[1] + 1 << std::endl;
			outfile << "f " << m_p[3] + 1 << " " << m_p[1] + 1 << " " << m_p[0] + 1 << std::endl;
			outfile << "f " << m_p[0] + 1 << " " << m_p[1] + 1 << " " << m_p[2] + 1 << std::endl;
		}
	}
}

void TetConstraint::DrawTet(const VectorX& x, const VBO& vbos)
{
	if (m_ghost_force.norm() > 0.4)
	{
		std::vector<glm::vec3> positions; positions.resize(4);
		std::vector<glm::vec3> colors; colors.resize(4);
		std::vector<glm::vec3> normals; normals.resize(4);
		std::vector<unsigned short> indices; indices.resize(12);

		for (unsigned int i = 0; i != 4; i++)
		{
			positions[i] = Eigen2GLM(x.block_vector(m_p[i]));
			colors[i] = glm::vec3(2, 0, 0);
			normals[i] = glm::vec3(1, 0, 0);
		}
		EigenMatrix3 Dd;
		getMatrixDs(Dd, x);
		if (Dd.determinant() > 0)
		{
			indices[0] = 3;
			indices[1] = 2;
			indices[2] = 0;
			indices[3] = 3;
			indices[4] = 1;
			indices[5] = 2;
			indices[6] = 3;
			indices[7] = 0;
			indices[8] = 1;
			indices[9] = 0;
			indices[10] = 2;
			indices[11] = 1;
		}
		else
		{
			indices[0] = 3;
			indices[1] = 0;
			indices[2] = 2;
			indices[3] = 3;
			indices[4] = 2;
			indices[5] = 1;
			indices[6] = 3;
			indices[7] = 1;
			indices[8] = 0;
			indices[9] = 0;
			indices[10] = 1;
			indices[11] = 2;
		}

		// position
		glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
		glBufferData(GL_ARRAY_BUFFER, 3 * positions.size() * sizeof(float), &positions[0], GL_STREAM_DRAW);

		// color
		glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
		glBufferData(GL_ARRAY_BUFFER, 3 * colors.size() * sizeof(float), &colors[0], GL_STREAM_DRAW);

		// normal
		glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
		glBufferData(GL_ARRAY_BUFFER, 3 * normals.size() * sizeof(float), &normals[0], GL_STREAM_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned short), &indices[0], GL_STATIC_DRAW);

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glEnableVertexAttribArray(2);

		glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glm::mat4 transformation(1.0f);
		//transformation = glm::translate(transformation, m_pos);

		glUniformMatrix4fv(vbos.m_uniform_transformation, 1, false, &transformation[0][0]);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
		glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_SHORT, 0);//GL_UNSIGNED_INT

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glDisableVertexAttribArray(2);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}
}