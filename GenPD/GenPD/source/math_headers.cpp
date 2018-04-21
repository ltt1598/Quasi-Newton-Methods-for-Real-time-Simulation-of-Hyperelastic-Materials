// ---------------------------------------------------------------------------------//
// Copyright (c) 2013, Regents of the University of Pennsylvania                    //
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

#include "math_headers.h"
#include <vector>

glm::vec3 Eigen2GLM(const EigenVector3& eigen_vector)
{
	return glm::vec3(eigen_vector[0], eigen_vector[1], eigen_vector[2]);
}
EigenVector3 GLM2Eigen(const glm::vec3& glm_vector)
{
	return EigenVector3(glm_vector[0], glm_vector[1], glm_vector[2]);
}
void Eigen2GLM(const VectorX& eigen_vector, std::vector<glm::vec3>& glm_vector)
{
	int size = glm_vector.size();
	assert(eigen_vector.size() == glm_vector.size() * 3);
#ifdef HIGH_PRECISION
	for (unsigned int i = 0; i < size; ++i)
	{
		glm_vector[i] = glm::vec3(eigen_vector[3 * i + 0], eigen_vector[3 * i + 1], eigen_vector[3 * i + 2]);
	}
#else
	memcpy(glm_vector.data(), eigen_vector.data(), sizeof(float)*size * 3);
#endif
}
void GLM2Eigen(const std::vector<glm::vec3>& glm_vector, VectorX& eigen_vector)
{
	int size = glm_vector.size();
	assert(eigen_vector.size() == glm_vector.size() * 3);
#ifdef HIGH_PRECISION
	for (unsigned int i = 0; i < size; ++i)
	{
		eigen_vector[3 * i + 0] = glm_vector[i].x;
		eigen_vector[3 * i + 1] = glm_vector[i].y;
		eigen_vector[3 * i + 2] = glm_vector[i].z;
	}
#else
	memcpy(eigen_vector.data(), glm_vector.data(), sizeof(float)*size * 3);
#endif
}

void EigenMakeSparseIdentityMatrix(unsigned int rows, unsigned int cols, SparseMatrix& I)
{
	assert(rows == cols);
	std::vector<SparseMatrixTriplet> triplets;
	for (unsigned int i = 0; i != rows; i++)
	{
		triplets.push_back(SparseMatrixTriplet(i, i, 1));
	}
	I.resize(rows, cols);
	I.setFromTriplets(triplets.begin(), triplets.end());
}

void EigenSparseMatrixToTriplets(const SparseMatrix& A, std::vector<SparseMatrixTriplet>& A_triplets)
{
	A_triplets.clear();
	A_triplets.reserve(A.nonZeros());
	for (unsigned int col = 0; col != A.outerSize(); ++col)
	{
		for (SparseMatrix::InnerIterator it(A, col); it; ++it)
		{
			A_triplets.push_back(SparseMatrixTriplet(it.row(), col, it.value()));
		}
	}
}

void EigenExtractDiagonalOffDiagonal(const SparseMatrix& A, VectorX& D, SparseMatrix& OD)
{
	unsigned rows = A.rows();
	unsigned cols = A.cols();
	assert(rows == cols);

	std::vector<SparseMatrixTriplet> A_triplets; A_triplets.clear();
	//std::vector<SparseMatrixTriplet> M_triplets; M_triplets.clear();
	std::vector<SparseMatrixTriplet> N_triplets; N_triplets.clear();

	D.resize(rows);
	OD.resize(rows, cols);

	D.setZero();

	EigenSparseMatrixToTriplets(A, A_triplets);

	ScalarType ind, val;
	for (std::vector<SparseMatrixTriplet>::iterator it = A_triplets.begin(); it != A_triplets.end(); it++)
	{
		if ((ind = it->col()) == it->row()) // digonal
		{
			val = it->value();
			D(ind) += val;
		}
		else
		{
			N_triplets.push_back((*it));
		}
	}

	OD.setFromTriplets(N_triplets.begin(), N_triplets.end());
}
void EigenExtractTriangular(const SparseMatrix& A, SparseMatrix& DL, SparseMatrix& U)
{
	unsigned rows = A.rows();
	unsigned cols = A.cols();
	assert(rows == cols);

	std::vector<SparseMatrixTriplet> A_triplets; A_triplets.clear();
	std::vector<SparseMatrixTriplet> M_triplets; M_triplets.clear();
	std::vector<SparseMatrixTriplet> N_triplets; N_triplets.clear();

	DL.resize(rows, cols);
	U.resize(rows, cols);

	EigenSparseMatrixToTriplets(A, A_triplets);

	ScalarType ind, val;
	for (std::vector<SparseMatrixTriplet>::iterator it = A_triplets.begin(); it != A_triplets.end(); it++)
	{
		if ((ind = it->col()) <= it->row()) // digonal
		{
			M_triplets.push_back((*it));
		}
		else
		{
			N_triplets.push_back((*it));
		}
	}

	DL.setFromTriplets(M_triplets.begin(), M_triplets.end());
	U.setFromTriplets(N_triplets.begin(), N_triplets.end());
}

// for subspace usage
void EigenExtractCols(Matrix& dst, const Matrix& src, const std::vector<unsigned int>& indices)
{
	unsigned int size = indices.size();
	assert(size > 0);

	dst.resize(src.rows(), size);

	for (unsigned int i = 0; i != size; ++i)
	{
		dst.col(i) = src.col(indices[i]);
	}
}

void EigenVectorExtractElements(VectorX& dst, const VectorX& src, const std::vector<unsigned int>& indices)
{
	unsigned int size = indices.size();
	assert(size > 0);

	dst.resize(size);

	for (unsigned int i = 0; i != size; ++i)
	{
		dst[i] = src[indices[i]];
	}
}

void EigenExtractCols(Matrix& dst, const Matrix& src, unsigned int d, bool from_front_end)
{
	std::vector<unsigned int> indices; indices.clear();
	for (unsigned int i = 0; i != d; ++i)
	{
		if (from_front_end)
		{
			indices.push_back(i);
		}
		else
		{
			indices.push_back(src.rows() - i - 1);
		}
	}

	EigenExtractCols(dst, src, indices);
}
void EigenVectorExtractElements(VectorX& dst, const VectorX& src, unsigned int d, bool from_front_end)
{
	std::vector<unsigned int> indices; indices.clear();
	for (unsigned int i = 0; i != d; ++i)
	{
		if (from_front_end)
		{
			indices.push_back(i);
		}
		else
		{
			indices.push_back(src.size() - i - 1);
		}
	}

	EigenVectorExtractElements(dst, src, indices);
}


// for matlab usage
#ifndef HIGH_PRECISION
void EigenSparseMatrixToTriplets(const SparseMatrix& A, std::vector<Eigen::Triplet<double, int>>& A_triplets)
{
	A_triplets.clear();
	A_triplets.reserve(A.nonZeros());
	for (unsigned int col = 0; col != A.outerSize(); ++col)
	{
		for (SparseMatrix::InnerIterator it(A, col); it; ++it)
		{
			A_triplets.push_back(Eigen::Triplet<double, int>(it.row(), col, (double)(it.value())));
		}
	}
}
void EigenVectorSingleToDouble(const Eigen::VectorXf& src, Eigen::VectorXd& dst)
{
	dst.resize(src.size());
	for (unsigned int i = 0; i != src.size(); i++)
	{
		dst(i) = (double)(src(i));
	}
}
#endif //HIGH_PRECISION


QueueLBFGS::QueueLBFGS(unsigned int vector_size, unsigned int queue_size)
{
	m_data_s = new ScalarType[vector_size*queue_size];
	m_data_y = new ScalarType[vector_size*queue_size];

	m_is_empty = true;
	m_is_full = false;

	m_vector_size = vector_size;
	m_capacity = queue_size;
	m_head_pointer = 0;
	m_tail_pointer = 0;
}

QueueLBFGS::~QueueLBFGS()
{
	delete[] m_data_s;
	delete[] m_data_y;
}

int QueueLBFGS::size()
{
	if (m_is_empty)
	{
		return 0;
	}
	else if (m_is_full)
	{
		return m_capacity;
	}
	else if (m_tail_pointer >= m_head_pointer)
	{
		return m_tail_pointer - m_head_pointer;
	}
	else
	{
		return m_tail_pointer + m_capacity - m_head_pointer;
	}
	return 0;
}

void QueueLBFGS::enqueue(const VectorX & sk, const VectorX & yk)
{
	if (!m_is_full && m_capacity!=0)
	{
		ScalarType* p_s_start = m_data_s + m_tail_pointer*m_vector_size;
		ScalarType* p_y_start = m_data_y + m_tail_pointer*m_vector_size;

		memcpy(p_s_start, sk.data(), sk.size()*sizeof(ScalarType));
		memcpy(p_y_start, yk.data(), yk.size()*sizeof(ScalarType));

		m_tail_pointer = (m_tail_pointer + 1) % m_capacity;
		m_is_empty = false;
		checkFull();
	}
}

void QueueLBFGS::dequeue()
{
	if (!m_is_empty && m_capacity != 0)
	{
		m_head_pointer = (m_head_pointer + 1) % m_capacity;
		m_is_full = false;
		checkEmpty();
	}
}

void QueueLBFGS::visitSandY(ScalarType ** s, ScalarType ** y, int i)
{
	int visit_pointer = (i + m_head_pointer) % m_capacity;

	(*s) = m_data_s + visit_pointer*m_vector_size;
	(*y) = m_data_y + visit_pointer*m_vector_size;
}