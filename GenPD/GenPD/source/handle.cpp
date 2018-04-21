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


#include "handle.h"
#include "primitive.h"

//public

void Handle::Reset()
{
	//m_rotation_primal_quat_3d.setIdentity();
	m_translation.setZero();
	m_R.setIdentity();
	m_temp_R.setIdentity();

	Update();
}

bool Handle::Select(std::vector<glm::vec3> ray)
{
	//if (m_mode == MODE_2D)
	//{
	//	glm::vec3 p = ray[0]; p.z = 0.0f;
	//	if (insidebb(p))
	//	{
	//		return true;
	//	}
	//	else
	//	{
	//		return false;
	//	}
	//}
	//else
	//{
		// TODO: 3d mode
	//}
	//insidebb()

	return false;
}

bool Handle::Select(std::vector<glm::vec3> ray, float& distance)
{
	// distance: distance from BB to camera. negative if there's no intersection
	if (raybbintersection(ray[0], ray[1], distance))
	{
		return true;
	}
	else
	{
		return false;
	}
}

ScalarType Handle::BoundingBoxSize()
{
	glm::vec3 diff = m_bounding_box_max-m_bounding_box_min;

	return (diff.x*diff.y + diff.x*diff.z + diff.y*diff.z);
}

glm::vec3 Handle::GetCoM()
{
	VectorX t = m_original_centor_of_mass + m_translation;
	//if (m_mode == MODE_2D)
	//{
	//	return glm::vec3(t.x(), t.y(), 0.0f);
	//}
	//else
	//{
		return glm::vec3(t.x(), t.y(), t.z());
	//}
}

glm::vec3 Handle::GetLocalCoM()
{
	return glm::vec3(m_original_centor_of_mass.x(), m_original_centor_of_mass.y(), m_original_centor_of_mass.z());
}

void Handle::MoveTemporary(const glm::vec3& trans)
{
	m_temp_translation = GLM2Eigen(trans);

	Update();
}

void Handle::MoveFinalize()
{
	m_translation = m_temp_translation;
	m_temp_translation.setZero();
	Update();
}

void Handle::RotateToValue(ScalarType theta)
{
	//if (m_mode == MODE_2D)
	//{
	//	m_rotation_angle_2d = theta;
	//}

	Update();
}

void Handle::RotateTemporary(const glm::vec3& axis, const float& theta)
{
	//if (m_mode == MODE_2D)
	//{
	//	m_temp_rotation_angle_2d = theta;
	//}
	//else
	//{
	//	// TODO: 3d mode
	//}

	m_temp_rotation = EigenAngleAxis(theta, GLM2Eigen(axis));
	m_temp_R = m_temp_rotation;

	Update();
}

void Handle::RotateFinalize()
{
	//if (m_mode == MODE_2D)
	//{
	//	m_rotation_angle_2d = m_temp_rotation_angle_2d;
	//	m_temp_rotation_angle_2d = 0.0;
	//}
	//else
	//{

	//	// TODO: 3d mode
	//}

	m_R = m_temp_R * m_R;
	m_temp_R.setIdentity();

	Update();
}

void Handle::ChangeTranslationOnXAxis(ScalarType t)
{
	m_translation(0) += t;

	Update();
}

void Handle::ChangeTranslation(const EigenVector3& dir, const float t)
{
	EigenVector3 dir_n = dir.normalized();// in case that dir is not unit vector
	
	m_translation += dir_n * t;

	Update();
}

void Handle::ChangeRotation(const EigenVector3& dir, const float t)
{
	EigenVector3 dir_n = dir.normalized();// in case that dir is not unit vector
	ScalarType tr = t * 3.1415926 / 180.0; // convert t into radius
	EigenAngleAxis angle_axis(tr, dir_n);

	m_R = angle_axis * m_R;

	Update();
}

void Handle::RotateExtra(ScalarType theta)
{
	//if (m_mode == MODE_2D)
	//{
	//	m_rotation_angle_2d += theta;
	//}
	//else
	//{

	//	// TODO: 3d mode
	//}
	Update();
}

void Handle::Draw(const VBO& vbos, bool selected)
{
	Cube cube(DEFAULT_SELECTION_RADIUS, DEFAULT_SELECTION_RADIUS, DEFAULT_SELECTION_RADIUS);
	if (selected)
	{
		cube.change_color(glm::vec3(1.2f, 1.2f, 0.0f)); // highlight using yellow
	}
	else
	{
		cube.change_color(m_color);
	}
	glm::vec3 m_x_i;
	for (unsigned int i = 0; i * 3 < m_original_x.size(); ++i)
	{
		// for 2d
		//m_x_i.x = m_x(2*i);
		//m_x_i.y = m_x(2*i+1);
		//m_x_i.z = 0;
		//cube.move_to(m_x_i);
		//cube.Draw(vbos);

		// for 3d
		m_x_i = Eigen2GLM(m_x.block<3, 1>(3 * i, 0));
		cube.move_to(m_x_i);
		cube.Draw(vbos);
	}
}

ScalarType Handle::HandleHandleDifference(const Handle& other)
{
	VectorX t1 = this->m_translation;
	VectorX t2 = other.m_translation;

	//if (m_mode == MODE_2D)
	//{
	//	ScalarType theta1 = this->m_rotation_angle_2d;
	//	ScalarType theta2 = other.m_rotation_angle_2d;

	//	ScalarType d1 = (t1-t2).norm();
	//	ScalarType d2 = std::abs(std::tan(0.5*(theta1-theta2)));

	//	return d1 + d2;
	//}
	//else
	//{
	//	// TODO: 3d mode

	//	return 0;
	//}
	return 0;
}

// protected:
void Handle::init()
{
	// vertices
	m_x = m_original_x;

	// transformation
	//m_rotation_primal_quat_3d.setIdentity();
	//m_temp_rotation_primal_quat_3d.setIdentity();
	m_translation.setZero();
	m_temp_translation.setZero();
	m_T.setIdentity();
	m_R.setIdentity();
	m_temp_R.setIdentity();

	// CoM
	m_original_centor_of_mass.setZero();
	int size = m_original_x.size()/3;
	if (size != 0)
	{
		for (unsigned int i = 0; i != size; ++i)
		{
			for (unsigned int j = 0; j != 3; ++j)
			{
				m_original_centor_of_mass(j) += m_original_x(3*i+j);
			}
		}
		m_original_centor_of_mass = m_original_centor_of_mass / (ScalarType)(size);
	}

	// update transformation matrix
	Update();
}

void Handle::cleanup()
{
	m_indices.clear();
}

bool Handle::insidebb(const glm::vec3& p)
{
	if (p.x < m_bounding_box_max.x && p.y < m_bounding_box_max.y && p.z < m_bounding_box_max.z &&
		p.x > m_bounding_box_min.x && p.y > m_bounding_box_min.y && p.z > m_bounding_box_min.z)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Handle::raybbintersection(const glm::vec3& p, const glm::vec3& normal, float& dist)
{
	float tmin = -1e10, tmax = 1e10;

	for (unsigned int i = 0; i != 3; i++)
	{
		if (normal[i] != 0.0) {
			float tx1 = (m_bounding_box_min[i] - p[i]) / normal[i];
			float tx2 = (m_bounding_box_max[i] - p[i]) / normal[i];

			tmin = std::max(tmin, std::min(tx1, tx2));
			tmax = std::min(tmax, std::max(tx1, tx2));
		}
	}

	dist = tmin;

	return tmax >= tmin;
}

void Handle::updateTransformationMatrix()
{
	//if (m_mode == MODE_2D)
	//{
	//	float theta;
	//	if (abs(m_temp_rotation_angle_2d) > EPSILON)
	//	{
	//		theta = m_temp_rotation_angle_2d;
	//	}
	//	else
	//	{
	//		theta = m_rotation_angle_2d;
	//	}
	//	VectorX translation = m_translation;
	//	if (m_temp_translation.squaredNorm() > EPSILON)
	//	{
	//		translation = m_temp_translation;
	//	}

	//	EigenMatrix3 R;
	//	R.setIdentity();
	//	R(0, 0) = cos(theta); R(0, 1) = -sin(theta); R(1, 0) = sin(theta); R(1, 1) = cos(theta);
	//	EigenMatrix3 T_to_origin, T_to_center, T_translate;
	//	T_to_origin.setIdentity(); T_to_center.setIdentity(); T_translate.setIdentity();
	//	T_to_origin.block<2, 1>(0, 2) = -m_original_centor_of_mass;
	//	T_to_center.block<2, 1>(0, 2) = m_original_centor_of_mass;
	//	T_translate.block<2, 1>(0, 2) = translation;
	//	//T_to_origin(0, 2) = -m_center_of_mass.x;
	//	//T_to_origin(1, 2) = -m_center_of_mass.y;
	//	//T_to_center(0, 2) = m_center_of_mass.x;
	//	//T_to_center(1, 2) = m_center_of_mass.y;
	//	//T_translate(0, 2) = m_translation_2d.x();
	//	//T_translate(1, 2) = m_translation_2d.y();

	//	m_T = T_translate * T_to_center * R * T_to_origin;
	//}
	//else
	//{
	//	// TODO: 3d case update transformation matrix;
	//}
	
	EigenVector3 translation = m_translation;
	if (m_temp_translation.squaredNorm() > EPSILON)
	{
		translation = m_temp_translation;
	}

	EigenMatrix4 R;
	R.setIdentity();
	R.block<3, 3>(0, 0) = m_temp_R*m_R;
	//R(0, 0) = cos(theta); R(0, 1) = -sin(theta); R(1, 0) = sin(theta); R(1, 1) = cos(theta);
	EigenMatrix4 T_to_origin, T_to_center, T_translate;
	T_to_origin.setIdentity(); T_to_center.setIdentity(); T_translate.setIdentity();
	T_to_origin.block<3, 1>(0, 3) = -m_original_centor_of_mass;
	T_to_center.block<3, 1>(0, 3) = m_original_centor_of_mass;
	T_translate.block<3, 1>(0, 3) = translation;

	m_T = T_translate * T_to_center * R * T_to_origin;
}

void Handle::Update()
{
	updateTransformationMatrix();

	// update position
	for (unsigned int i = 0; i * 3 < m_original_x.size(); ++i)
	{
		EigenVector4 v(m_original_x(3 * i), m_original_x(3 * i + 1), m_original_x(3 * i + 2), 1.0);
		v = m_T * v;

		m_x.block<3, 1>(3 * i, 0) = v.block<3, 1>(0, 0);
	}

	// update bounding box
	m_bounding_box_min = glm::vec3(1e10, 1e10, 1e10);
	m_bounding_box_max = -m_bounding_box_min;
	glm::vec3 m_x_i;
	for (unsigned int i = 0; i * 3 < m_original_x.size(); ++i)
	{
		m_x_i = Eigen2GLM(m_x.block<3, 1>(3 * i, 0));
		m_bounding_box_min = glm::min(m_bounding_box_min, m_x_i);
		m_bounding_box_max = glm::max(m_bounding_box_max, m_x_i);
	}
	// selection slackness
	m_bounding_box_min -= glm::vec3(DEFAULT_SELECTION_RADIUS, DEFAULT_SELECTION_RADIUS, DEFAULT_SELECTION_RADIUS);
	m_bounding_box_max += glm::vec3(DEFAULT_SELECTION_RADIUS, DEFAULT_SELECTION_RADIUS, DEFAULT_SELECTION_RADIUS);
}
