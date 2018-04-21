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

#ifndef _MESH_H_
#define _MESH_H_

#include <vector>

#include "opengl_headers.h"
#include "global_headers.h"
#include "math_headers.h"
#include "io_mesh.h"
#include "anttweakbar_wrapper.h"
#include "simulation.h"
#include "camera.h"
#include "primitive.h"

//forward declarations
class Camera;
class AntTweakBarWrapper;
class Simulation;

typedef enum
{
	MESH_TYPE_CLOTH,
	MESH_TYPE_TET,

	MESH_TYPE_TOTAL_NUM
} MeshType;

struct Edge
{
	unsigned int m_v1, m_v2; // indices of endpoint vertices
	unsigned int m_tri1, m_tri2; // indices of adjacent faces
};

class Mesh
{
	friend class AntTweakBarWrapper;
	friend class Simulation;
	friend class Camera;

public:
	Mesh() : m_mesh_type() {}
	Mesh(MeshType mesh_type) : m_mesh_type(mesh_type) {}
	virtual ~Mesh() {Cleanup();}

	void Reset();
	virtual bool Init() {std::cout << "Warning: reach base class virtual init function." << std::endl; return false;}
	virtual void Cleanup();
	virtual void Update();

	// Display
	virtual void Draw(const VBO& vbos, int show_texture = 0);
	virtual void DrawWireFrame(const VBO& vbos, int line_width = 1);
	// IO
	virtual void ExportToOBJ(const char* filename);
	virtual bool ImportFromOBJ(const char* filename);

	// member accessor
	virtual inline unsigned int GetNumberOfVertices() { return m_vertices_number; }
	virtual inline unsigned int GetDimension() { return m_system_dimension; }
	inline MeshType GetMeshType() { return m_mesh_type; }

	void CopyFromClothMesh(const Mesh* src);

public:
	MeshType m_mesh_type;

	unsigned int m_vertices_number; // m
	unsigned int m_system_dimension; // 3m
	unsigned int m_expanded_system_dimension; //6s
	unsigned int m_expanded_system_dimension_1d; //2s

	// vertices positions/previous positions/mass
	VectorX m_restpose_positions; // 1x3m
	VectorX m_current_positions; // 1x3m
	VectorX m_current_velocities; // 1x3m
	VectorX m_previous_positions; // 1x3m
	VectorX m_previous_velocities; // 1x3m
	SparseMatrix m_mass_matrix; // 3mx3m
	SparseMatrix m_inv_mass_matrix; // 3mx3m

	SparseMatrix m_mass_matrix_1d;
	SparseMatrix m_inv_mass_matrix_1d;

	// for generating constraints.
	std::vector<Edge> m_edge_list;

	// for visualizing the cloth (and selection). currently only support single floating points 
	std::vector<glm::vec3> m_positions;
	std::vector<glm::vec3> m_normals;
	std::vector<glm::vec3> m_colors;
	std::vector<glm::vec2> m_texcoords;
	std::vector<unsigned int> m_triangle_list;

	// for all
	ScalarType m_total_mass;

	// I put them here because of anttweakbar...
	// for cloth
	unsigned int m_dim[2]; // width and length
	EigenVector3 m_corners[2]; // upper left and lower right corner

	// for tet
	// tet mesh location
	char m_tet_file_path[256];
	ScalarType m_tet_scaling;
	bool m_tet_flip;
	ScalarType m_tet_rotation;

protected:
	// initialize every particle pos / vel / mass / color.
	virtual void generateParticleList() {std::cout << "Warning: reach base class virtual function." << std::endl;}
	// generate triangle list from vetices
	virtual void generateTriangleList() {std::cout << "Warning: reach base class virtual function." << std::endl;}
	// generate edge list from the geometry representation.
	virtual void generateEdgeList() {std::cout << "Warning: reach base class virtual function." << std::endl;}

	// jitter the initial condition.
	void jitterParticlesList();
	// update the normal per frame for visualization.
	void computeNormal();
};

class ClothMesh : public Mesh
{
	friend class AntTweakBarWrapper;
	friend class Simulation;

public:
	ClothMesh() : Mesh(MESH_TYPE_CLOTH) {}
	ClothMesh(unsigned int dim0, unsigned int dim1) : Mesh(MESH_TYPE_CLOTH) {m_dim[0] = dim0; m_dim[1] = dim1;}
	virtual ~ClothMesh() {}

	virtual bool Init();

protected:

	// initialize every particle pos / vel / mass / color.
	virtual void generateParticleList();
	// generate triangle list from vetices
	virtual void generateTriangleList();
	// generate edge list from the geometry representation.
	virtual void generateEdgeList();
};

class TetMesh : public Mesh
{
	friend class AntTweakBarWrapper;
	friend class Simulation;

public:
	TetMesh() : Mesh(MESH_TYPE_TET), m_loaded_mesh(NULL) {}
	virtual ~TetMesh() {if(m_loaded_mesh) {delete m_loaded_mesh;}}

	virtual bool Init();

protected:
	// tet mesh if loaded from mesh file
	MeshLoader *m_loaded_mesh;

protected:

	// initialize every particle pos / vel / mass / color.
	virtual void generateParticleList();
	// generate triangle list from vetices
	virtual void generateTriangleList();
	// generate edge list from the geometry representation.
	virtual void generateEdgeList();
};

#endif