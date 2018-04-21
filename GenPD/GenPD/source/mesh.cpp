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

#pragma warning( disable : 4267)

#include <fstream>
#include "mesh.h"

void Mesh::Reset()
{
	Cleanup();
	Init();
	Update();
}

void Mesh::Cleanup()
{
	m_edge_list.clear();

	m_positions.clear();
	m_normals.clear();
	m_colors.clear();
	m_texcoords.clear();
	m_triangle_list.clear();
}

void Mesh::Update()
{
	Eigen2GLM(m_current_positions, m_positions);
}

void Mesh::Draw(const VBO& vbos, int show_texture)
{
	computeNormal();

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	unsigned int size = m_vertices_number;
	unsigned int element_num = m_triangle_list.size();

	// position
	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &m_positions[0], GL_DYNAMIC_DRAW);

	// color
	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &m_colors[0], GL_STATIC_DRAW);
	// normal
	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &m_normals[0], GL_DYNAMIC_DRAW);
	// texture
	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_tbo);
	glBufferData(GL_ARRAY_BUFFER, 2 * size * sizeof(float), &m_texcoords[0], GL_STATIC_DRAW);

	// indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, element_num * sizeof(unsigned int), &m_triangle_list[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glEnableVertexAttribArray(3);

	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_tbo);
	glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glm::mat4 identity = glm::mat4(); // identity matrix
	glUniformMatrix4fv(vbos.m_uniform_transformation, 1, false, &identity[0][0]);

	glUniform1i(vbos.m_uniform_enable_texture, show_texture); // enable/disable texture

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
	glDrawElements(GL_TRIANGLES, element_num, GL_UNSIGNED_INT, 0);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
	glDisableVertexAttribArray(3);
	glUniform1i(vbos.m_uniform_enable_texture, 0); // disable texture

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
}

void Mesh::DrawWireFrame(const VBO& vbos, int line_width)
{
	computeNormal();

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glLineWidth(line_width);

	unsigned int size = m_vertices_number;
	unsigned int element_num = m_triangle_list.size();

	// position
	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &m_positions[0], GL_DYNAMIC_DRAW);

	// color
	std::vector<glm::vec3> colors;
	colors.resize(m_positions.size());
	std::fill(colors.begin(), colors.end(), glm::vec3(0, 0, 0));
	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &colors[0], GL_STATIC_DRAW);

	// indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, element_num * sizeof(unsigned int), &m_triangle_list[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glm::mat4 identity = glm::mat4(); // identity matrix
	glUniformMatrix4fv(vbos.m_uniform_transformation, 1, false, &identity[0][0]);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
	glDrawElements(GL_TRIANGLES, element_num, GL_UNSIGNED_INT, 0);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
}

void Mesh::ExportToOBJ(const char* filename)
{
	std::ofstream outfile;
	outfile.open(filename, std::ifstream::out);
	if (outfile.is_open())
	{
		outfile << "//#vertices: " << m_vertices_number << std::endl;
		for (unsigned int i = 0; i < m_vertices_number; i++)
		{
			// save positions
			outfile << "v " << m_current_positions.block_vector(i)[0] << " " << m_current_positions.block_vector(i)[1] << " " << m_current_positions.block_vector(i)[2] << std::endl;
			// save velocities (which is hidden in obj files)
			outfile << "//vel " << m_current_velocities.block_vector(i)[0] << " " << m_current_velocities.block_vector(i)[1] << " " << m_current_velocities.block_vector(i)[2] << std::endl;
		}

		outfile << "//#faces: " << m_vertices_number << std::endl;
		for (unsigned int f = 0; f < m_triangle_list.size(); f += 3)
		{
			outfile << "f " << m_triangle_list[f+0]+1 << " " << m_triangle_list[f+1]+1 << " " << m_triangle_list[f+2]+1 << std::endl;
		}

		outfile.close();
	}
}

bool Mesh::ImportFromOBJ(const char* filename)
{
	std::ifstream infile;
	infile.open(filename, std::ifstream::in);
	char ignoreToken[256];
	if (infile.is_open())
	{
		unsigned int number_of_vertices;
		infile >> ignoreToken >> number_of_vertices;
		if (number_of_vertices != m_vertices_number)
		{
			printf("Warning: configuration of Mesh state is different with current setting. \n");
			infile.close();
			return false;
		}
		for (unsigned int i = 0; i < number_of_vertices; i++)
		{
			EigenVector3 temp_pos;
			infile >> ignoreToken >> m_current_positions[3*i+0] >> m_current_positions[3*i+1] >> m_current_positions[3*i+2];
			infile >> ignoreToken >> m_current_velocities[3*i+0] >> m_current_velocities[3*i+1] >> m_current_velocities[3*i+2];
		}
		m_previous_positions = m_current_positions;
		m_previous_velocities = m_current_velocities;

		infile.close();
	}

	return true;
}

void Mesh::computeNormal()
{
	// reset all the normal.
	glm::vec3 zero(0.0);
	for(std::vector<glm::vec3>::iterator n = m_normals.begin(); n != m_normals.end(); ++n)
	{
		*n = zero;
	}
	// calculate normal for each individual triangle
	unsigned int triangle_num = m_triangle_list.size() / 3;
	unsigned int id0, id1, id2;
	EigenVector3 p0, p1, p2;
	EigenVector3 normal;
	for(unsigned int i = 0; i < triangle_num; ++i)
	{
		id0 = m_triangle_list[3 * i];
		id1 = m_triangle_list[3 * i + 1];
		id2 = m_triangle_list[3 * i + 2];

		p0 = m_current_positions.block_vector(id0);
		p1 = m_current_positions.block_vector(id1);
		p2 = m_current_positions.block_vector(id2);

		normal = (p1-p0).cross(p2-p1);
		normal.normalize();
		glm::vec3 glm_normal = glm::vec3(normal[0], normal[1], normal[2]);

		m_normals[id0] += glm_normal;
		m_normals[id1] += glm_normal;
		m_normals[id2] += glm_normal;
	}
	// re-normalize all the normals.
	for(std::vector<glm::vec3>::iterator n = m_normals.begin(); n != m_normals.end(); ++n)
	{
		if (glm::length(*n) > EPSILON) // skip if norm is a zero vector
			*n = glm::normalize(*n);
	}
}

void Mesh::CopyFromClothMesh(const Mesh* src)
{
	if (src->m_mesh_type == MESH_TYPE_CLOTH)
	{
		m_mesh_type = src->m_mesh_type;
		m_total_mass = src->m_total_mass;
		for (unsigned i = 0; i != 2; i++)
		{
			m_dim[i] = src->m_dim[i];
		}
		// hard code corners
		m_corners[0] = EigenVector3(5, 0, 5);
		m_corners[1] = EigenVector3(-5, 0, -5);

		Init();
	}
}

void Mesh::jitterParticlesList()
{
	unsigned int index;
	for(index = 0; index < m_vertices_number; ++index)
	{
		index;
		ScalarType a = ((ScalarType)(rand())/(ScalarType)(RAND_MAX+1)-0.5)*1e-2;
		ScalarType b = ((ScalarType)(rand())/(ScalarType)(RAND_MAX+1)-0.5)*1e-2;
		ScalarType c = ((ScalarType)(rand())/(ScalarType)(RAND_MAX+1)-0.5)*1e-2;
		m_current_positions.block_vector(index) += EigenVector3(a,b,c);
	}
}

bool ClothMesh::Init()
{
	generateParticleList();
	generateTriangleList();
	generateEdgeList();

	return true;
}

void ClothMesh::generateParticleList()
{
	m_vertices_number = m_dim[0]*m_dim[1];
	m_system_dimension = m_vertices_number * 3; 

	EigenVector3 delta;
	delta[0] = (m_corners[1][0] - m_corners[0][0]) / (ScalarType)(m_dim[0] - 1);
	delta[1] = (m_corners[1][1] - m_corners[0][1]) / (ScalarType)(m_dim[1] - 1);
	delta[2] = (m_corners[1][2] - m_corners[0][2]) / (ScalarType)(m_dim[1] - 1);

	ScalarType unit_mass = m_total_mass / m_vertices_number;

	m_positions.resize(m_vertices_number);
	m_normals.resize(m_vertices_number);
	m_colors.resize(m_vertices_number);
	m_texcoords.resize(m_vertices_number);

	m_restpose_positions.resize(m_system_dimension);
	m_current_positions.resize(m_system_dimension);
	m_current_velocities.resize(m_system_dimension);
	m_mass_matrix.resize(m_system_dimension, m_system_dimension);
	m_inv_mass_matrix.resize(m_system_dimension, m_system_dimension);

	m_mass_matrix_1d.resize(m_vertices_number, m_vertices_number);
	m_inv_mass_matrix_1d.resize(m_vertices_number, m_vertices_number);

	// Assign initial position to all the vertices.
	m_restpose_positions.setZero();
	unsigned int i, k, index;
	for(i = 0; i < m_dim[0]; ++i)
	{
		for(k = 0; k < m_dim[1]; ++k)
		{
			index = m_dim[1] * i + k;
			m_restpose_positions.block_vector(index) = EigenVector3(delta[0] * i + m_corners[0][0], delta[1] * k + m_corners[0][1], delta[2] * k + m_corners[0][2]);
		}
	}
	// Assign initial velocity to zero
	m_current_velocities.setZero();
	m_current_positions = m_restpose_positions;
	m_previous_positions = m_current_positions;
	m_previous_velocities = m_current_velocities;

	// Assign mass matrix and an equally sized identity matrix
	std::vector<SparseMatrixTriplet> i_triplets;
	std::vector<SparseMatrixTriplet> m_triplets;
	std::vector<SparseMatrixTriplet> m_inv_triplets;
	i_triplets.clear();
	m_triplets.clear();
	ScalarType inv_unit_mass = 1.0 / unit_mass;
	for (index = 0; index < m_system_dimension; index++)
	{
		i_triplets.push_back(SparseMatrixTriplet(index, index, 1));
		m_triplets.push_back(SparseMatrixTriplet(index, index, unit_mass));
		m_inv_triplets.push_back(SparseMatrixTriplet(index, index, inv_unit_mass));
	}
	m_mass_matrix.setFromTriplets(m_triplets.begin(), m_triplets.end());
	m_inv_mass_matrix.setFromTriplets(m_inv_triplets.begin(), m_inv_triplets.end());
	m_triplets.clear();
	m_inv_triplets.clear();
	i_triplets.clear();
	for (index = 0; index < m_vertices_number; index++)
	{
		i_triplets.push_back(SparseMatrixTriplet(index, index, 1));
		m_triplets.push_back(SparseMatrixTriplet(index, index, unit_mass));
		m_inv_triplets.push_back(SparseMatrixTriplet(index, index, inv_unit_mass));
	}
	m_mass_matrix_1d.setFromTriplets(m_triplets.begin(), m_triplets.end());
	m_inv_mass_matrix_1d.setFromTriplets(m_inv_triplets.begin(), m_inv_triplets.end());

	// Assign color and texture uv to all the vertices.
	glm::vec3 mesh_color(0.3, 0.8, 1);
	assert(m_dim[0] >=2 && m_dim[1] >= 2);
	ScalarType inv_1 = 1.0 / (m_dim[0]-1);
	ScalarType inv_2 = 1.0 / (m_dim[1]-1);
	for(i = 0; i < m_dim[0]; ++i)
	{
		for(k = 0; k < m_dim[1]; ++k)
		{
			index = m_dim[1] * i + k;
			m_colors[index] = mesh_color;
			m_texcoords[index] = glm::vec2(inv_1*i, inv_2*k);
		}
	}
}

void ClothMesh::generateTriangleList()
{
	unsigned int i, k, index;
	// Generate the triangle list.
	m_triangle_list.resize(( m_dim[0] - 1) * ( m_dim[1] - 1) * 2 * 3);
	//printf("Triangle number: %u\n", m_triangle_list.size() / 3);
	// loop over all the small squares.
	bool row_flip = false, column_flip = false;
	for(i = 0; i <  m_dim[0] - 1; ++i)
	{
		for(k = 0; k <  m_dim[1] - 1; ++k)
		{
			index = ( m_dim[1] - 1) * i + k;

			// first triangle
			m_triangle_list[6 * index + 0] =  m_dim[1] * i + k;
			m_triangle_list[6 * index + 1] =  m_dim[1] * i + k + 1;
			m_triangle_list[6 * index + 2] =  m_dim[1] * (i + 1) + ((row_flip ^ column_flip) ? (k + 1) : k);
			// second triangle
			m_triangle_list[6 * index + 3] =  m_dim[1] * (i + 1) + k + 1;
			m_triangle_list[6 * index + 4] =  m_dim[1] * (i + 1) + k;
			m_triangle_list[6 * index + 5] =  m_dim[1] * i + ((row_flip ^ column_flip) ? k : (k + 1));

			row_flip = !row_flip;
		}
		column_flip = !column_flip;
		row_flip = false;
	}
}

void ClothMesh::generateEdgeList()
{
	// generate all the edges from the vertices and triangle list.
	// courtesy of Eric Lengyel, "Building an Edge List for an Arbitrary Mesh". Terathon Software 3D Graphics Library.
	// http://www.terathon.com/code/edges.html
	unsigned int vert_num = m_vertices_number;
	unsigned int tri_num = m_triangle_list.size() / 3;

	unsigned int *first_edge = new unsigned int[vert_num + 3 * tri_num];
	unsigned int *next_edge = first_edge + vert_num;

	for(unsigned int i = 0; i < vert_num; ++i)
		first_edge[i] = 0xFFFFFFFF;
	// First pass over all triangles. Finds out all the edges satisfying the condition that
	// the first vertex index is less than the second vertex index when the direction from 
	// the first to the second represents a counterclockwise winding around the triangle to
	// which the edge belongs. For each edge found, the edge index is stored in a linked 
	// list of edges belonging to the lower-numbered vertex index i. This allows us to 
	// quickly find an edge in the second pass whose higher-numbered vertex is i.

	unsigned int edge_count = 0;
	const unsigned int* triangle = &m_triangle_list[0];
	unsigned int i1, i2;
	for(unsigned int t = 0; t < tri_num; ++t)
	{
		i1 = triangle[2];
		for(unsigned int n = 0; n < 3; ++n)
		{
			i2 = triangle[n];
			if(i1 < i2)
			{
				Edge new_edge;
				new_edge.m_v1 = i1;
				new_edge.m_v2 = i2;
				new_edge.m_tri1 = t;
				new_edge.m_tri2 = t;
				m_edge_list.push_back(new_edge);

				unsigned int edge_idx = first_edge[i1];
				if(edge_idx == 0xFFFFFFFF)
				{
					first_edge[i1] = edge_count;
				}
				else
				{
					while(true)
					{
						unsigned int idx = next_edge[edge_idx];
						if(idx == 0xFFFFFFFF)
						{
							next_edge[edge_idx] = edge_count;
							break;
						}
						edge_idx = idx;
					}
				}

				next_edge[edge_count] = 0xFFFFFFFF;
				edge_count++;
			}
			i1 = i2;
		}
		triangle += 3;
	}

	// Second pass over all triangles. Finds out all the edges satisfying the condition that
	// the first vertex index is greater than the second vertex index when the direction from 
	// the first to the second represents a counterclockwise winding around the triangle to
	// which the edge belongs. For each of these edges, the same edge should have already been
	// found in the first pass for a different triangle. So we search the list of edges for the
	// higher-numbered index for the matching edge and fill in the second triangle index. The 
	// maximum number of the comparisons in this search for any vertex is the number of edges
	// having that vertex as an endpoint.
	triangle = &m_triangle_list[0];
	for(unsigned int t = 0; t < tri_num; ++t)
	{
		i1 = triangle[2];
		for(unsigned int n = 0; n < 3; ++n)
		{
			i2 = triangle[n];
			if(i1 > i2)
			{
				bool is_new_edge = true;
				for(unsigned int edge_idx = first_edge[i2]; edge_idx != 0xFFFFFFFF; edge_idx = next_edge[edge_idx])
				{
					Edge *edge = &m_edge_list[edge_idx];
					if((edge->m_v2 == i1) && (edge->m_tri1 == edge->m_tri2))
					{
						edge->m_tri2 = t;
						is_new_edge = false;
						break;
					}
				}
				// for case where a edge belongs to only one triangle. i.e. mesh is not watertight.
				if(is_new_edge)
				{
					Edge new_edge;
					new_edge.m_v1 = i1;
					new_edge.m_v2 = i2;
					new_edge.m_tri1 = t;
					new_edge.m_tri2 = t;
					m_edge_list.push_back(new_edge);

					unsigned int edge_idx = first_edge[i1];
					if(edge_idx == 0xFFFFFFFF)
					{
						first_edge[i1] = edge_count;
					}
					else
					{
						while(true)
						{
							unsigned int idx = next_edge[edge_idx];
							if(idx == 0xFFFFFFFF)
							{
								next_edge[edge_idx] = edge_count;
								break;
							}
							edge_idx = idx;
						}
					}

					next_edge[edge_count] = 0xFFFFFFFF;
					edge_count++;
				}
			}
			i1 = i2;
		}
		triangle += 3;
	}

	delete[] first_edge;
	//printf("Edge number: %u.\n", m_edge_list.size());
}

bool TetMesh::Init()
{
	m_loaded_mesh = new MeshLoader(m_tet_file_path, m_tet_scaling, m_tet_flip, m_tet_rotation);
	if (m_loaded_mesh->Info() == false)
	{
		std::cout << "Load mesh error. Using regular Mesh." << std::endl;
		delete m_loaded_mesh;
		
		return false;
	}

	generateParticleList();
	generateTriangleList();
	//generateEdgeList();

	return true;
}

void TetMesh::generateParticleList()
{
	m_vertices_number = m_loaded_mesh->m_vertices.size();
	m_system_dimension = m_vertices_number * 3;
	ScalarType unit_mass = m_total_mass / m_system_dimension;

	m_positions.resize(m_vertices_number);
	m_normals.resize(m_vertices_number);
	m_colors.resize(m_vertices_number);
	m_texcoords.resize(m_vertices_number);

	// Assign initial position, velocity and mass to all the vertices.
	// Assign color to all the vertices.
	m_restpose_positions.resize(m_system_dimension);
	m_current_positions.resize(m_system_dimension);
	m_current_velocities.resize(m_system_dimension);
	m_mass_matrix.resize(m_system_dimension, m_system_dimension);
	m_inv_mass_matrix.resize(m_system_dimension, m_system_dimension);

	m_mass_matrix_1d.resize(m_vertices_number, m_vertices_number);
	m_inv_mass_matrix_1d.resize(m_vertices_number, m_vertices_number);

	// Assign initial position to all the vertices.
	m_restpose_positions.setZero();
	unsigned int index;
	for(index = 0; index < m_vertices_number; ++index)
	{
		m_restpose_positions.block_vector(index) = GLM2Eigen(m_loaded_mesh->m_vertices[index]);
	}
	// Assign initial velocity to zero
	m_current_velocities.setZero();
	m_current_positions = m_restpose_positions;
	m_previous_positions = m_restpose_positions;
	m_previous_velocities = m_current_velocities;

	// Assign mass matrix and an equally sized identity matrix
	std::vector<SparseMatrixTriplet> i_triplets;
	std::vector<SparseMatrixTriplet> m_triplets;
	std::vector<SparseMatrixTriplet> m_inv_triplets;
	i_triplets.clear();
	m_triplets.clear();
	ScalarType inv_unit_mass = 1.0 / unit_mass;
	for (index = 0; index < m_system_dimension; index++)
	{
		i_triplets.push_back(SparseMatrixTriplet(index, index, 1));
		m_triplets.push_back(SparseMatrixTriplet(index, index, unit_mass));
		m_inv_triplets.push_back(SparseMatrixTriplet(index, index, inv_unit_mass));
	}
	m_mass_matrix.setFromTriplets(m_triplets.begin(), m_triplets.end());
	m_inv_mass_matrix.setFromTriplets(m_inv_triplets.begin(), m_inv_triplets.end());
	m_triplets.clear();
	m_inv_triplets.clear();
	i_triplets.clear();
	for (index = 0; index < m_vertices_number; index++)
	{
		i_triplets.push_back(SparseMatrixTriplet(index, index, 1));
		m_triplets.push_back(SparseMatrixTriplet(index, index, unit_mass));
		m_inv_triplets.push_back(SparseMatrixTriplet(index, index, inv_unit_mass));
	}
	m_mass_matrix_1d.setFromTriplets(m_triplets.begin(), m_triplets.end());
	m_inv_mass_matrix_1d.setFromTriplets(m_inv_triplets.begin(), m_inv_triplets.end());

	// color
	glm::vec3 mesh_color(0.3, 0.8, 1);
	for(index = 0; index < m_vertices_number; ++index)
	{
		m_colors[index] = mesh_color;
	}
}

void TetMesh::generateTriangleList()
{
	unsigned int size = m_loaded_mesh->m_faces.size();
	m_triangle_list.resize(size * 3);

	for (int i = 0; i != size; ++i)
	{
		MeshLoader::Face& face = m_loaded_mesh->m_faces[i]; 
		m_triangle_list[3 * i + 0] = face.id1;
		m_triangle_list[3 * i + 1] = face.id2;
		m_triangle_list[3 * i + 2] = face.id3;
	}
}
void TetMesh::generateEdgeList()
{
	unsigned int vert_num = m_vertices_number;

	SparseMatrix EdgeMatrix(vert_num, vert_num);
	EdgeMatrix.setZero();

	unsigned int i1, i2;
	for (std::vector<MeshLoader::Tet>::iterator it = m_loaded_mesh->m_tets.begin(); it != m_loaded_mesh->m_tets.end(); ++it)
	{
		i1 = it->id1;
		i2 = it->id2;
		if (EdgeMatrix.coeff(i1, i2) < EPSILON)
		{
			EdgeMatrix.coeffRef(i1, i2) = 1;
			EdgeMatrix.coeffRef(i2, i1) = 1;
			Edge new_edge;
			new_edge.m_v1 = i1;
			new_edge.m_v2 = i2;
			m_edge_list.push_back(new_edge);
		}

		i1 = it->id1;
		i2 = it->id3;
		if (EdgeMatrix.coeff(i1, i2) < EPSILON)
		{
			EdgeMatrix.coeffRef(i1, i2) = 1;
			EdgeMatrix.coeffRef(i2, i1) = 1;
			Edge new_edge;
			new_edge.m_v1 = i1;
			new_edge.m_v2 = i2;
			m_edge_list.push_back(new_edge);
		}

		i1 = it->id1;
		i2 = it->id4;
		if (EdgeMatrix.coeff(i1, i2) < EPSILON)
		{
			EdgeMatrix.coeffRef(i1, i2) = 1;
			EdgeMatrix.coeffRef(i2, i1) = 1;
			Edge new_edge;
			new_edge.m_v1 = i1;
			new_edge.m_v2 = i2;
			m_edge_list.push_back(new_edge);
		}

		i1 = it->id2;
		i2 = it->id3;
		if (EdgeMatrix.coeff(i1, i2) < EPSILON)
		{
			EdgeMatrix.coeffRef(i1, i2) = 1;
			EdgeMatrix.coeffRef(i2, i1) = 1;
			Edge new_edge;
			new_edge.m_v1 = i1;
			new_edge.m_v2 = i2;
			m_edge_list.push_back(new_edge);
		}

		i1 = it->id2;
		i2 = it->id4;
		if (EdgeMatrix.coeff(i1, i2) < EPSILON)
		{
			EdgeMatrix.coeffRef(i1, i2) = 1;
			EdgeMatrix.coeffRef(i2, i1) = 1;
			Edge new_edge;
			new_edge.m_v1 = i1;
			new_edge.m_v2 = i2;
			m_edge_list.push_back(new_edge);
		}

		i1 = it->id3;
		i2 = it->id4;
		if (EdgeMatrix.coeff(i1, i2) < EPSILON)
		{
			EdgeMatrix.coeffRef(i1, i2) = 1;
			EdgeMatrix.coeffRef(i2, i1) = 1;
			Edge new_edge;
			new_edge.m_v1 = i1;
			new_edge.m_v2 = i2;
			m_edge_list.push_back(new_edge);
		}
	}
}