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


#include "tensor_math.h"

// matrix3333
Matrix3333::Matrix3333()
{

}

Matrix3333::Matrix3333(const Matrix3333& other)
{
	for (unsigned int row = 0; row != 3; ++row)
	{
		for (unsigned int col = 0; col != 3; ++col)
		{
			mat[row][col] = other.mat[row][col];
		}
	}
}

void Matrix3333::SetZero()
{
	for (unsigned int row = 0; row != 3; ++row)
	{
		for (unsigned int col = 0; col != 3; ++col)
		{
			mat[row][col] = EigenMatrix3::Zero();
		}
	}
}

void Matrix3333::SetIdentity()
{
	for (unsigned int row = 0; row != 3; ++row)
	{
		for (unsigned int col = 0; col != 3; ++col)
		{
			mat[row][col] = EigenMatrix3::Zero();
			mat[row][col](row, col) = 1.0;
		}
	}
}

EigenMatrix3& Matrix3333::operator() (int row, int col)
{
	assert(row >= 0 && row < 3 && col >= 0 && col < 3);
	return mat[row][col];
}

Matrix3333 Matrix3333::operator+ (const Matrix3333& plus)
{
	Matrix3333 res;
	for (unsigned int row = 0; row != 3; ++row)
	{
		for (unsigned int col = 0; col != 3; ++col)
		{
			res.mat[row][col] = mat[row][col] + plus.mat[row][col];
		}
	}
	return res;
}

Matrix3333 Matrix3333::operator- (const Matrix3333& minus)
{
	Matrix3333 res;
	for (unsigned int row = 0; row != 3; ++row)
	{
		for (unsigned int col = 0; col != 3; ++col)
		{
			res.mat[row][col] = mat[row][col] - minus.mat[row][col];
		}
	}
	return res;
}

Matrix3333 Matrix3333::operator* (const EigenMatrix3& multi)
{
	Matrix3333 res;
	for (unsigned int i = 0; i < 3; ++i)
	{
		for (unsigned int j = 0; j < 3; ++j)
		{
			res.mat[i][j].setZero();
			for (unsigned int k = 0; k < 3; k++)
			{
				res.mat[i][j] += mat[i][k] * multi(k, j);
			}
		}
	}
	return res;
}

Matrix3333 operator* (const EigenMatrix3& multi1, Matrix3333& multi2)
{
	Matrix3333 res;
	for (unsigned int i = 0; i < 3; ++i)
	{
		for (unsigned int j = 0; j < 3; ++j)
		{
			res(i, j).setZero();
			for (unsigned int k = 0; k < 3; k++)
			{
				res(i, j) += multi1(i, k) * multi2(k, j);
			}
		}
	}
	return res;
}

Matrix3333 Matrix3333::operator* (ScalarType multi)
{
	Matrix3333 res;
	for (unsigned int i = 0; i < 3; ++i)
	{
		for (unsigned int j = 0; j < 3; ++j)
		{
			res.mat[i][j] = mat[i][j] * multi;
		}
	}
	return res;
}

Matrix3333 operator* (ScalarType multi1, Matrix3333& multi2)
{
	Matrix3333 res;
	for (unsigned int i = 0; i < 3; ++i)
	{
		for (unsigned int j = 0; j < 3; ++j)
		{
			res(i, j) = multi1 * multi2(i, j);
		}
	}
	return res;
}

Matrix3333 Matrix3333::transpose()
{
	Matrix3333 res;
	for (unsigned int i = 0; i < 3; ++i)
	{
		for (unsigned int j = 0; j < 3; ++j)
		{
			res(i, j) = mat[j][i];
		}
	}
	return res;
}
EigenMatrix3 Matrix3333::Contract(const EigenMatrix3& multi)
{
	EigenMatrix3 res;
	res.setZero();
	for (unsigned int i = 0; i < 3; ++i)
	{
		for (unsigned int j = 0; j < 3; ++j)
		{
			res += mat[i][j] * multi(i, j);
		}
	}
	return res;
}

Matrix3333 Matrix3333::Contract(Matrix3333& multi)
{
	Matrix3333 res;
	for (unsigned int i = 0; i < 3; ++i)
	{
		for (unsigned int j = 0; j < 3; ++j)
		{
			res(i, j) = this->Contract(multi(i, j));
		}
	}
	return res;
}

// matrix2222
Matrix2222::Matrix2222()
{

}

Matrix2222::Matrix2222(const Matrix2222& other)
{
	for (unsigned int row = 0; row != 2; ++row)
	{
		for (unsigned int col = 0; col != 2; ++col)
		{
			mat[row][col] = other.mat[row][col];
		}
	}
}

void Matrix2222::SetZero()
{
	for (unsigned int row = 0; row != 2; ++row)
	{
		for (unsigned int col = 0; col != 2; ++col)
		{
			mat[row][col] = EigenMatrix2::Zero();
		}
	}
}

void Matrix2222::SetIdentity()
{
	for (unsigned int row = 0; row != 2; ++row)
	{
		for (unsigned int col = 0; col != 2; ++col)
		{
			mat[row][col] = EigenMatrix2::Zero();
			mat[row][col](row, col) = 1.0;
		}
	}
}

EigenMatrix2& Matrix2222::operator() (int row, int col)
{
	assert(row >= 0 && row < 2 && col >= 0 && col < 2);
	return mat[row][col];
}

Matrix2222 Matrix2222::operator+ (const Matrix2222& plus)
{
	Matrix2222 res;
	for (unsigned int row = 0; row != 2; ++row)
	{
		for (unsigned int col = 0; col != 2; ++col)
		{
			res.mat[row][col] = mat[row][col] + plus.mat[row][col];
		}
	}
	return res;
}

Matrix2222 Matrix2222::operator- (const Matrix2222& minus)
{
	Matrix2222 res;
	for (unsigned int row = 0; row != 2; ++row)
	{
		for (unsigned int col = 0; col != 2; ++col)
		{
			res.mat[row][col] = mat[row][col] - minus.mat[row][col];
		}
	}
	return res;
}

Matrix2222 Matrix2222::operator* (const EigenMatrix2& multi)
{
	Matrix2222 res;
	for (unsigned int i = 0; i < 2; ++i)
	{
		for (unsigned int j = 0; j < 2; ++j)
		{
			res.mat[i][j].setZero();
			for (unsigned int k = 0; k < 2; k++)
			{
				res.mat[i][j] += mat[i][k] * multi(k, j);
			}
		}
	}
	return res;
}
Matrix2222 operator* (const EigenMatrix2& multi1, Matrix2222& multi2)
{
	Matrix2222 res;
	for (unsigned int i = 0; i < 2; ++i)
	{
		for (unsigned int j = 0; j < 2; ++j)
		{
			res(i, j).setZero();
			for (unsigned int k = 0; k < 2; k++)
			{
				res(i, j) = multi1(i, k) * multi2(k, j);
			}
		}
	}
	return res;
}

Matrix2222 Matrix2222::operator* (ScalarType multi)
{
	Matrix2222 res;
	for (unsigned int i = 0; i < 2; ++i)
	{
		for (unsigned int j = 0; j < 2; ++j)
		{
			res.mat[i][j] = mat[i][j] * multi;
		}
	}
	return res;
}

Matrix2222 operator* (ScalarType multi1, Matrix2222& multi2)
{
	Matrix2222 res;
	for (unsigned int i = 0; i < 2; ++i)
	{
		for (unsigned int j = 0; j < 2; ++j)
		{
			res(i, j) = multi1 * multi2(i, j);
		}
	}
	return res;
}

Matrix2222 Matrix2222::transpose()
{
	Matrix2222 res;
	for (unsigned int i = 0; i < 2; ++i)
	{
		for (unsigned int j = 0; j < 2; ++j)
		{
			res(i, j) = mat[j][i];
		}
	}
	return res;
}
EigenMatrix2 Matrix2222::Contract(const EigenMatrix2& multi)
{
	EigenMatrix2 res;
	res.setZero();
	for (unsigned int i = 0; i < 2; ++i)
	{
		for (unsigned int j = 0; j < 2; ++j)
		{
			res += mat[i][j] * multi(i, j);
		}
	}
	return res;
}

Matrix2222 Matrix2222::Contract(Matrix2222& multi)
{
	Matrix2222 res;
	for (unsigned int i = 0; i < 2; ++i)
	{
		for (unsigned int j = 0; j < 2; ++j)
		{
			res(i, j) = this->Contract(multi(i, j));
		}
	}
	return res;
}


void directProduct(Matrix3333& dst, const EigenMatrix3& src1, const EigenMatrix3& src2)
{
	for (unsigned int i = 0; i < 3; ++i)
	{
		for (unsigned int j = 0; j < 3; ++j)
		{
			dst(i, j) = src1(i, j) * src2;
		}
	}
}
void directProduct(Matrix2222& dst, const EigenMatrix2& src1, const EigenMatrix2& src2)
{
	for (unsigned int i = 0; i < 2; ++i)
	{
		for (unsigned int j = 0; j < 2; ++j)
		{
			dst(i, j) = src1(i, j) * src2;
		}
	}
}
