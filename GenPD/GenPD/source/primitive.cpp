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

#include <fstream>

#include "primitive.h"

#define COLLISION_EPSILON 1e-3

bool lineTriangleIntersection(EigenVector3& A, EigenVector3& B, EigenVector3& C, EigenVector3& n, EigenVector3& P, EigenVector3& dir, ScalarType& t)
{
	if (dir.dot(n) >= -COLLISION_EPSILON) // same direction or parallel
		return false;

	t = - (n.dot(P-A)) / n.dot(dir);
	
	if (t < -COLLISION_EPSILON || t > 1)
		return false;

	EigenVector3 Q = P+t*dir;

	if (((B-A).cross(Q-A)).dot(n) >= 0)
	{
		if (((C-B).cross(Q-B)).dot(n) >= 0)
		{
			if (((A-C).cross(Q-C)).dot(n) >= 0)
			{
				t = t-COLLISION_EPSILON;
				return true;
			}
		}
	}

	return false;
}

//----------Base Class-----------//
void Primitive::change_color(const glm::vec3& color)
{
	for (int i = 0; i < m_colors.size(); i++)
	{
		m_colors[i] = color;
	}
}

void Primitive::Draw(const VBO& vbos)
{
	// position
	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * m_positions.size() * sizeof(float), &m_positions[0], GL_STREAM_DRAW);

	// color
	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * m_colors.size() * sizeof(float), &m_colors[0], GL_STREAM_DRAW);

	// normal
	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * m_normals.size() * sizeof(float), &m_normals[0], GL_STREAM_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(unsigned short), &m_indices[0], GL_STATIC_DRAW);

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
	transformation = glm::translate(transformation, m_pos);

	glUniformMatrix4fv(vbos.m_uniform_transformation, 1, false, &transformation[0][0]);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
	glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_SHORT, 0);//GL_UNSIGNED_INT

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

//----------Plane Class----------//
void Plane::init_visualization()
{
    m_positions.clear();
    m_colors.clear();
    m_normals.clear();
    m_indices.clear();

    glm::vec3 center(0.0, 0.0, 0.0);
    glm::vec3 local_x, local_z;
    local_x = glm::cross(m_normal, glm::vec3(0.0, 0.0, 1.0));
    if(glm::length(local_x) < 0.00001f)
        local_x = glm::cross(m_normal, glm::vec3(1.0, 0.0, 0.0));
    local_x = glm::normalize(local_x);
    local_z = glm::normalize(glm::cross(local_x, m_normal));

    glm::vec3 mat_color(1.0);
    unsigned int slice = 24;

    glm::vec3 vertex(center);
    m_positions.push_back(center);
    m_normals.push_back(m_normal);
    m_colors.push_back(mat_color);

    float delta = 360.0 / slice;
    float radius = 100.0;
    glm::vec3 local_pos;
    for(float theta = 0.0; theta < 359.99; theta += delta)
    {
        local_pos.x = radius * cos(glm::radians(theta));
        local_pos.z = radius * sin(glm::radians(theta));

        vertex = local_pos.x * local_x - local_pos.z * local_z + center;

        m_positions.push_back(vertex);
        m_normals.push_back(m_normal);
        m_colors.push_back(mat_color);
    }
    for(unsigned int i = 0; i < slice-1; ++i)
    {
        m_indices.push_back(0);
        m_indices.push_back(i + 1);
        m_indices.push_back(i + 2);
    }
    m_indices.push_back(0);
    m_indices.push_back(slice);
    m_indices.push_back(1);
}

bool Plane::StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist)
{
	ScalarType height = m_pos[1];
	dist = p(1) - height - COLLISION_EPSILON;
	normal = EigenVector3(m_normal[0], m_normal[1], m_normal[2]);

	if (dist < 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//----------Sphere Class----------//
void Sphere::init_visualization()
{
    m_positions.clear();
    m_colors.clear();
    m_normals.clear();
    m_indices.clear();

    glm::vec3 mat_color(0.6);
    unsigned int slice = 24, stack = 10;

    glm::vec3 tnormal(0.0, 1.0, 0.0), tpos;
	tpos = m_radius * tnormal;

    m_positions.push_back(tpos);
    m_normals.push_back(tnormal);
    m_colors.push_back(mat_color);

	float theta_z, theta_y, sin_z;
    float delta_y = 360.0 / slice, delta_z = 180.0 / stack;
	//loop over the sphere
	for(theta_z = delta_z; theta_z < 179.99; theta_z += delta_z)
	{
		for(theta_y = 0.0; theta_y < 359.99; theta_y += delta_y)
		{
			sin_z = sin(glm::radians(theta_z));
			
            tnormal.x = sin_z * cos(glm::radians(theta_y));
			tnormal.y = cos(glm::radians(theta_z));
			tnormal.z = -sin_z * sin(glm::radians(theta_y));

			tpos = m_radius * tnormal;

            m_positions.push_back(tpos);
            m_normals.push_back(tnormal);
            m_colors.push_back(mat_color);
		}
	}
	tnormal = glm::vec3(0.0, -1.0, 0.0);
    tpos = m_radius * tnormal;

    m_positions.push_back(tpos);
    m_normals.push_back(tnormal);
    m_colors.push_back(mat_color);

	//indices
	unsigned int j = 0, k = 0;
	for(j = 0; j < slice - 1; ++j)
	{
		m_indices.push_back(0);
		m_indices.push_back(j + 1);
		m_indices.push_back(j + 2);
	}
	m_indices.push_back(0);
	m_indices.push_back(slice);
	m_indices.push_back(1);

	for(j = 0; j < stack - 2; ++j)
	{
		for(k = 1 + slice * j; k < slice * (j + 1); ++k)
		{
			m_indices.push_back(k);
			m_indices.push_back(k + slice);
			m_indices.push_back(k + slice + 1);

			m_indices.push_back(k);
			m_indices.push_back(k + slice + 1);
			m_indices.push_back(k + 1);
		}
		m_indices.push_back(k);
		m_indices.push_back(k + slice);
		m_indices.push_back(k + 1);

		m_indices.push_back(k);
		m_indices.push_back(k + 1);
		m_indices.push_back(k + 1 - slice);
	}

    unsigned int bottom_id = (stack - 1) * slice + 1;
    unsigned int offset = bottom_id - slice;
	for(j = 0; j < slice - 1; ++j)
	{
		m_indices.push_back(j + offset);
		m_indices.push_back(bottom_id);
		m_indices.push_back(j + offset + 1);
	}
	m_indices.push_back(bottom_id - 1);
	m_indices.push_back(bottom_id);
	m_indices.push_back(offset);

	if(m_indices.size() != 6 * (stack - 1) * slice)
		printf("indices number not correct!\n");
}

bool Sphere::StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist)
{
	EigenVector3 center = EigenVector3(m_pos[0], m_pos[1], m_pos[2]);
	EigenVector3 diff = p - center;
	dist = diff.norm() - m_radius - COLLISION_EPSILON;
	if (dist < 0)
	{
		normal = diff.normalized();

		return true;
	}
	else
	{
		return false;
	}
}

//----------Torus Class----------//
void Torus::init_visualization()
{
	m_positions.clear();
	m_colors.clear();
	m_normals.clear();
	m_indices.clear();

	glm::vec3 tnormal, tpos;

	glm::vec3 mat_color(0.6);
	unsigned int slice = 20, stack = 20;

	// pos, norm and color
	float delta_theta = 360.0 / slice, delta_phi = 360.0 / stack;
	for (unsigned int i = 0; i != slice; i++)
	{
		float theta = glm::radians(delta_theta * i);
		float cos_theta = cos(theta);
		float sin_theta = sin(theta);
		glm::vec3 tube_center(cos_theta*m_major_radius, 0.0, sin_theta*m_major_radius);
		for (unsigned int j = 0; j != stack; j++)
		{
			float phi = glm::radians(delta_phi * j);
			float cos_phi = cos(phi);
			float sin_phi = sin(phi);

			tnormal.x = cos_theta * cos_phi;
			tnormal.y = sin_phi;
			tnormal.z = sin_theta * cos_phi;

			tpos = tube_center + tnormal * m_minor_radius;

			m_positions.push_back(tpos);
			m_normals.push_back(tnormal);
			m_colors.push_back(mat_color);
		}
	}

	// indices
	unsigned short p[4];
	for (unsigned short i = 0; i != slice; i++)
	{
		unsigned short j = (i + 1) % slice;
		for (unsigned short k = 0; k != stack; k++)
		{
			unsigned short l = (k + 1) % stack;

			p[0] = i*stack + k;
			p[1] = i*stack + l;
			p[2] = j*stack + k;
			p[3] = j*stack + l;

			m_indices.push_back(p[0]);
			m_indices.push_back(p[2]);
			m_indices.push_back(p[1]);

			m_indices.push_back(p[1]);
			m_indices.push_back(p[2]);
			m_indices.push_back(p[3]);
		}
	}
}

bool Torus::StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist)
{
	// find the center on the tube
	EigenVector3 center = GLM2Eigen(m_pos);
	EigenVector3 diff_xz = p - center;
	diff_xz(1) = 0; // set y component to 0
	EigenVector3 tube_center = diff_xz.normalized() * m_major_radius + center;

	EigenVector3 diff = p - tube_center;
	dist = diff.norm() - m_minor_radius - COLLISION_EPSILON;
	if (dist < 0)
	{
		normal = diff.normalized();

		return true;
	}
	else
	{
		return false;
	}
}


//----------Cube Class----------//
void Cube::init_visualization()
{
    m_positions.clear();
    m_colors.clear();
    m_normals.clear();
    m_indices.clear();

    glm::vec3 mat_color(0.6);

	// front face 012, 321
	m_positions.push_back(glm::vec3(-m_hf_dims.x, -m_hf_dims.y, +m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,0,1));
	m_positions.push_back(glm::vec3(+m_hf_dims.x, -m_hf_dims.y, +m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,0,1));
	m_positions.push_back(glm::vec3(-m_hf_dims.x, +m_hf_dims.y, +m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,0,1));
	m_positions.push_back(glm::vec3(+m_hf_dims.x, +m_hf_dims.y, +m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,0,1));
	m_indices.push_back(0);
	m_indices.push_back(1);
	m_indices.push_back(2);
	m_indices.push_back(3);
	m_indices.push_back(2);
	m_indices.push_back(1);

	// back face 654 567
	m_positions.push_back(glm::vec3(-m_hf_dims.x, -m_hf_dims.y, -m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,0,-1));
	m_positions.push_back(glm::vec3(+m_hf_dims.x, -m_hf_dims.y, -m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,0,-1));
	m_positions.push_back(glm::vec3(-m_hf_dims.x, +m_hf_dims.y, -m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,0,-1));
	m_positions.push_back(glm::vec3(+m_hf_dims.x, +m_hf_dims.y, -m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,0,-1));
	m_indices.push_back(6);
	m_indices.push_back(5);
	m_indices.push_back(4);
	m_indices.push_back(5);
	m_indices.push_back(6);
	m_indices.push_back(7);

	// right face 8 9 10, 11 10 9
	m_positions.push_back(glm::vec3(+m_hf_dims.x, -m_hf_dims.y, +m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(1,0,0));
	m_positions.push_back(glm::vec3(+m_hf_dims.x, -m_hf_dims.y, -m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(1,0,0));
	m_positions.push_back(glm::vec3(+m_hf_dims.x, +m_hf_dims.y, +m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(1,0,0));
	m_positions.push_back(glm::vec3(+m_hf_dims.x, +m_hf_dims.y, -m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(1,0,0));
	m_indices.push_back(8);
	m_indices.push_back(9);
	m_indices.push_back(10);
	m_indices.push_back(11);
	m_indices.push_back(10);
	m_indices.push_back(9);

	// left face 14 13 12, 13 14 15
	m_positions.push_back(glm::vec3(-m_hf_dims.x, -m_hf_dims.y, +m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(-1,0,0));
	m_positions.push_back(glm::vec3(-m_hf_dims.x, -m_hf_dims.y, -m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(-1,0,0));
	m_positions.push_back(glm::vec3(-m_hf_dims.x, +m_hf_dims.y, +m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(-1,0,0));
	m_positions.push_back(glm::vec3(-m_hf_dims.x, +m_hf_dims.y, -m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(-1,0,0));
	m_indices.push_back(14);
	m_indices.push_back(13);
	m_indices.push_back(12);
	m_indices.push_back(13);
	m_indices.push_back(14);
	m_indices.push_back(15);

	// top face 16 17 18, 19 18 17
	m_positions.push_back(glm::vec3(-m_hf_dims.x, +m_hf_dims.y, +m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,1,0));
	m_positions.push_back(glm::vec3(+m_hf_dims.x, +m_hf_dims.y, +m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,1,0));
	m_positions.push_back(glm::vec3(-m_hf_dims.x, +m_hf_dims.y, -m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,1,0));
	m_positions.push_back(glm::vec3(+m_hf_dims.x, +m_hf_dims.y, -m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,1,0));
	m_indices.push_back(16);
	m_indices.push_back(17);
	m_indices.push_back(18);
	m_indices.push_back(19);
	m_indices.push_back(18);
	m_indices.push_back(17);

	// bottom face 22 21 20, 21 22 23
	m_positions.push_back(glm::vec3(-m_hf_dims.x, -m_hf_dims.y, +m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,-1,0));
	m_positions.push_back(glm::vec3(+m_hf_dims.x, -m_hf_dims.y, +m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,-1,0));
	m_positions.push_back(glm::vec3(-m_hf_dims.x, -m_hf_dims.y, -m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,-1,0));
	m_positions.push_back(glm::vec3(+m_hf_dims.x, -m_hf_dims.y, -m_hf_dims.z));
	m_colors.push_back(mat_color);
	m_normals.push_back(glm::vec3(0,-1,0));
	m_indices.push_back(22);
	m_indices.push_back(21);
	m_indices.push_back(20);
	m_indices.push_back(21);
	m_indices.push_back(22);
	m_indices.push_back(23);
}

void Cube::move_to(const glm::vec3& target)
{
	m_pos = target;
}

bool Cube::StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist)
{
	return false;
}

//----------OBJMesh Class----------//
void ObjMesh::read_from_file(char* filename)
{
	m_positions.clear();
	m_colors.clear();
	m_normals.clear();
	m_indices.clear();
	glm::vec3 mat_color(0.6); 
	// vertices and color

	std::ifstream infile(filename);
    if(!infile.good())
    {
        printf("Error in loading file %s\n", filename);
        exit(0);
    }
    char buffer[256];
    unsigned int ip0, ip1, ip2;
    unsigned int n0, n1, n2;
    glm::vec3 pos;

    while(!infile.getline(buffer,255).eof())
    {
        buffer[255] = '\0';
        if(buffer[0] == 'v' && (buffer[1] == ' ' || buffer[1] == 32))
        {
            if(sscanf_s(buffer, "v %f %f %f", &pos.x, &pos.y, &pos.z) == 3)
            {
				pos = m_scaling * pos;
                m_positions.push_back(pos);
            }
            else
            {
                printf("Vertex is not in desired format.\n");
                exit(0);
            }
        }
        else if (buffer[0] == 'v' && buffer[1] == 'n' && (buffer[2] == ' ' || buffer[2] == 32))
        {
            // load normals from obj file.
        }
        else if (buffer[0] == 'f' && (buffer[1] == ' ' || buffer[1] == 32))
        {
            if(sscanf_s(buffer, "f %u %u %u", &ip0, &ip1, &ip2) == 3)
			{
				m_indices.push_back(--ip0);
				m_indices.push_back(--ip1);
				m_indices.push_back(--ip2);
			}
            else if(sscanf_s(buffer, "f %u//%u %u//%u %u//%u", &ip0, &n0, &ip1, &n1, &ip2, &n2) == 6)
			{
				m_indices.push_back(--ip0);
				m_indices.push_back(--ip1);
				m_indices.push_back(--ip2);
			}
            else if(sscanf_s(buffer, "f %u/%u %u/%u %u/%u", &ip0, &n0, &ip1, &n1, &ip2, &n2) == 6)
			{
				m_indices.push_back(--ip0);
				m_indices.push_back(--ip1);
				m_indices.push_back(--ip2);
			}
            else
            {
                printf("Triangle indices is not in desired format.\n");
                exit(0);
            }
        }
    }
	// normals

    unsigned int id, size;
    bool vert_norm = (m_normals.size() != m_positions.size());
    if(vert_norm)
        m_normals.resize(m_positions.size(), glm::vec3(0.0f));

    size = m_indices.size();
    glm::uvec3 triangle;
    glm::vec3 p0, p1, p2;
    glm::vec3 norm;
    float phi0, phi1, phi2;
    float pi = glm::radians(180.0f);
    for(id = 0; id < size; id+=3)
    {
		triangle = glm::uvec3(m_indices[id], m_indices[id+1], m_indices[id+2]);
        p0 = m_positions[triangle.x];
        p1 = m_positions[triangle.y];
        p2 = m_positions[triangle.z];
        norm = glm::normalize(glm::cross(p1 - p0, p2 - p0));
        // calculate vertex normal
        if(vert_norm)
        {
            phi0 = std::acos(glm::dot(p1 - p0, p2 - p0) / (glm::length(p1 - p0) * glm::length(p2 - p0)));
            phi1 = std::acos(glm::dot(p0 - p1, p2 - p1) / (glm::length(p0 - p1) * glm::length(p2 - p1)));
            phi2 = pi - phi0 - phi1;

            m_normals[triangle.x] += phi0 * norm;
            m_normals[triangle.y] += phi1 * norm;
            m_normals[triangle.z] += phi2 * norm;
        }
    }
    // re-normalize all normals
    for(std::vector<glm::vec3>::iterator iter = m_normals.begin(); iter != m_normals.end(); ++iter)
	{
        *iter = glm::normalize(*iter);
		m_colors.push_back(mat_color);
		//m_colors.push_back(*iter);
	}
}

bool ObjMesh::StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist)
{
	// TODO

	return false;

}
