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


#ifndef _TENSOR_MATH_H_
#define _TENSOR_MATH_H_

#include "global_headers.h"
#include "math_headers.h"

class Matrix3333 // 3x3 matrix: each element is a 3x3 matrix
{
public:
	Matrix3333();
	Matrix3333(const Matrix3333& other);
	~Matrix3333() {}

	void SetZero(); // [0 0 0; 0 0 0; 0 0 0]; 0 = 3x3 zeros
	void SetIdentity(); //[I 0 0; 0 I 0; 0 0 I]; 0 = 3x3 zeros, I = 3x3 identity

	// operators
	EigenMatrix3& operator() (int row, int col);
	Matrix3333 operator+ (const Matrix3333& plus);
	Matrix3333 operator- (const Matrix3333& minus);
	Matrix3333 operator* (const EigenMatrix3& multi);
	friend Matrix3333 operator* (const EigenMatrix3& multi1, Matrix3333& multi2);
	Matrix3333 operator* (ScalarType multi);
	friend Matrix3333 operator* (ScalarType multi1, Matrix3333& multi2);
	Matrix3333 transpose();
	EigenMatrix3 Contract(const EigenMatrix3& multi); // this operator is commutative
	Matrix3333 Contract(Matrix3333& multi);

protected:

	EigenMatrix3 mat[3][3];
};

class Matrix2222 // 2x2 matrix: each element is a 2x2 matrix
{
public:
	Matrix2222();
	Matrix2222(const Matrix2222& other);
	~Matrix2222() {}

	void SetZero(); // [0 0; 0 0]; 0 = 2x2 zeros
	void SetIdentity(); //[I 0; 0 I;]; 0 = 2x2 zeros, I = 2x2 identity

	// operators and basic functions
	EigenMatrix2& operator() (int row, int col);
	Matrix2222 operator+ (const Matrix2222& plus);
	Matrix2222 operator- (const Matrix2222& minus);
	Matrix2222 operator* (const EigenMatrix2& multi);
	friend Matrix2222 operator* (const EigenMatrix2& multi1, Matrix2222& multi2);
	Matrix2222 operator* (ScalarType multi);
	friend Matrix2222 operator* (ScalarType multi1, Matrix2222& multi2);
	Matrix2222 transpose();
	EigenMatrix2 Contract(const EigenMatrix2& multi); // this operator is commutative
	Matrix2222 Contract(Matrix2222& multi);

protected:

	EigenMatrix2 mat[2][2];
};

// dst = src1 \kron src2
void directProduct(Matrix3333& dst, const EigenMatrix3& src1, const EigenMatrix3& src2);
void directProduct(Matrix2222& dst, const EigenMatrix2& src1, const EigenMatrix2& src2);


#endif