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


#ifndef _HANDLE_H_
#define _HANDLE_H_

#include <vector>
#include "math_headers.h"
#include "opengl_headers.h"
#include "constraint.h"

class Handle
{
public:
	Handle() {init();}
	Handle(const std::vector<unsigned int>& indices, const VectorX& vertices, const glm::vec3& color, const int id) : 
		m_indices(indices), 
		m_original_x(vertices), 
		m_color(color),
		m_handle_id(id)
	{
		init();
	}
    Handle(const Handle& other) : 
        m_indices(other.m_indices), 
        m_original_x(other.m_original_x),
		m_original_centor_of_mass(other.m_original_centor_of_mass),
		m_x(other.m_x),
		m_translation(other.m_translation),
		m_temp_translation(other.m_temp_translation),
		m_R(other.m_R),
		m_temp_R(other.m_temp_R),
		//m_rotation_primal_quat_3d(other.m_rotation_primal_quat_3d),
		//m_temp_rotation_primal_quat_3d(other.m_temp_rotation_primal_quat_3d),
		m_color(other.m_color),
		m_handle_id(other.m_handle_id),
		attachment_constraints(other.attachment_constraints)
    {
		Update();
    }
	~Handle() {cleanup();}

	void Reset();
	void Update();

	bool Select(std::vector<glm::vec3> ray);
	bool Select(std::vector<glm::vec3> ray, float& distance);
	ScalarType BoundingBoxSize();
	glm::vec3 GetCoM();
	glm::vec3 GetLocalCoM();
	void MoveTemporary(const glm::vec3& trans);
	void MoveFinalize();
	void RotateToValue(ScalarType theta);
	void RotateTemporary(const glm::vec3& axis, const float& theta);
	void RotateFinalize();
	void ChangeTranslationOnXAxis(ScalarType t);
	void ChangeTranslation(const EigenVector3& dir, const float t);
	void ChangeRotation(const EigenVector3& dir, const float t);
	void RotateExtra(ScalarType theta);

	void Draw(const VBO& vbos, bool selected = false);

	ScalarType HandleHandleDifference(const Handle& other);

	// data accesser
	inline std::vector<unsigned int>& Indices() {return m_indices;}
	inline glm::vec3& Color() {return m_color;}
	inline EigenMatrix4& GetTransformationMatrix() {return m_T; }
	//inline EigenQuaternion& Rotation3d() {return m_rotation_primal_quat_3d;}
	inline EigenVector3& Translation() {return m_translation;}
	inline EigenVector3& CoM() { return m_original_centor_of_mass; }
	inline EigenMatrix3& Rotation() {return m_R; }
	inline unsigned int& ID() {return m_handle_id;}
	inline const VectorX& Vertices() { return m_x; }
	inline EigenVector3 operator[] (int i) { return m_x.block<3, 1>(i * 3, 0); }

public:
	// for fast accessing the related attachment constraints
	std::vector<AttachmentConstraint*> attachment_constraints;

protected:

	// key components
	std::vector<unsigned int> m_indices;
	VectorX m_original_x;
	EigenVector3 m_original_centor_of_mass;
	VectorX m_x;
	// translation
	EigenVector3 m_temp_translation;
	EigenVector3 m_translation;
	// 3d rotation
	EigenMatrix3 m_R;
	EigenMatrix3 m_temp_R;
	EigenAngleAxis m_temp_rotation;
	EigenAngleAxis m_rotation;
	//EigenQuaternion m_temp_rotation_primal_quat_3d;
	//EigenQuaternion m_rotation_primal_quat_3d;
	// transformation matrix 3x3 in 2d case and 4x4 in 3d case
	EigenMatrix4 m_T;

	// for visualization and selection and identity
	unsigned int m_handle_id;
	glm::vec3 m_color;
	glm::vec3 m_bounding_box_min;
	glm::vec3 m_bounding_box_max;

protected:
	void init();
	void cleanup();
	bool insidebb(const glm::vec3& p);
	bool raybbintersection(const glm::vec3& p, const glm::vec3& normal, float& dist);

	void updateTransformationMatrix();
};

#endif