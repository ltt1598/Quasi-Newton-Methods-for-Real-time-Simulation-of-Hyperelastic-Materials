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

#ifndef _PRIMITIVE_H_
#define _PRIMITIVE_H_

#include "global_headers.h"
#include "math_headers.h"
#include "opengl_headers.h"

// TODO: add more primitives here.
enum PrimitiveType {PLANE, SPHERE, TORUS, CUBE, OBJMESH};
class Primitive
{
public:
	Primitive(const PrimitiveType& t) : m_type(t), m_pos(glm::vec3(0, 0, 0)), m_vel(glm::vec3(0, 0, 0)), m_has_vel(false), m_has_gravity(false), m_previous_pos(glm::vec3(0, 0, 0)), m_start_moving_frame(0) { init_visualization(); };
	Primitive(const PrimitiveType& t, glm::vec3 pos) : m_type(t), m_pos(pos), m_vel(glm::vec3(0, 0, 0)), m_has_vel(false), m_has_gravity(false), m_previous_pos(pos), m_start_moving_frame(0) { init_visualization(); };
	Primitive(const PrimitiveType& t, glm::vec3 pos, glm::vec3 vel) : m_type(t), m_pos(pos), m_vel(vel), m_has_vel(true), m_has_gravity(true), m_previous_pos(pos), m_start_moving_frame(0) { init_visualization(); };
	Primitive(const PrimitiveType& t, glm::vec3 pos, glm::vec3 vel, bool has_gravity) : m_type(t), m_pos(pos), m_vel(vel), m_has_vel(true), m_has_gravity(has_gravity), m_previous_pos(pos), m_start_moving_frame(0) { init_visualization(); };
    Primitive(const Primitive& other) : 
        m_type(other.m_type), 
        m_positions(other.m_positions), m_colors(other.m_colors), m_normals(other.m_normals),
        m_indices(other.m_indices),
		m_pos(other.m_pos), m_vel(other.m_vel), m_previous_pos(other.m_previous_pos),
		m_start_moving_frame(other.m_start_moving_frame)
    {
    }
    virtual ~Primitive()
    {
        m_positions.clear();
        m_colors.clear();
        m_normals.clear();
        m_indices.clear();
    }

    PrimitiveType type() const
    {
        return m_type;
    }

	unsigned int& StartFrame() { return m_start_moving_frame; }

	virtual void Update(ScalarType h, unsigned int current_frame = -1) { if (current_frame > m_start_moving_frame) { m_pos += h*m_vel; } }
	virtual inline glm::vec3& Vel() { return m_vel; }

	virtual void move_to(const glm::vec3& target) { m_pos = target; }
	virtual bool StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist) {return false;}

	virtual void change_color(const glm::vec3& color);
    virtual void Draw(const VBO& vbos);

	inline std::vector<glm::vec3>& GetPositions() {return m_positions;}
	inline std::vector<unsigned short>& GetTriangulation() {return m_indices;}

protected:
    virtual void init_visualization() {}
public:
	glm::vec3 m_pos;

	// animation
	glm::vec3 m_previous_pos;
	glm::vec3 m_vel;
	bool m_has_vel;
	bool m_has_gravity;
	unsigned int m_start_moving_frame;

protected:
    PrimitiveType m_type;
    std::vector<glm::vec3> m_positions, m_colors, m_normals;
    std::vector<unsigned short> m_indices;
};

class Plane : public Primitive
{
public:
    Plane() : Primitive(PLANE), m_normal(glm::vec3(0.0, 1.0, 0.0)) {init_visualization();};
    Plane(const glm::vec3 norm, float value) : Primitive(PLANE, glm::vec3(0.0, value, 0.0)), m_normal(norm) {init_visualization();};
    Plane(const Plane& other) : Primitive(other), m_normal(other.m_normal){};
    virtual ~Plane() {};

	virtual bool StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist);
        
protected:
    virtual void init_visualization();
protected:
    glm::vec3 m_normal;
};

class Sphere : public Primitive
{
public:
	Sphere() : Primitive(SPHERE), m_radius(DEFAULT_SELECTION_RADIUS) {init_visualization();};
    Sphere(float radius) : Primitive(SPHERE), m_radius(radius) {init_visualization();};
    Sphere(const glm::vec3 pos, float radius) : Primitive(SPHERE, pos), m_radius(radius) {init_visualization();};
    Sphere(const glm::vec3 pos, const glm::vec3 vel, float radius) : Primitive(SPHERE, pos, vel), m_radius(radius) {init_visualization();};
    Sphere(const glm::vec3 pos, const glm::vec3 vel, bool has_gravity, float radius) : Primitive(SPHERE, pos, vel, has_gravity), m_radius(radius) {init_visualization();};
    Sphere(const Sphere& other) : Primitive(other), m_radius(other.m_radius) {};
    virtual ~Sphere() {};

	virtual bool StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist);
protected:
    virtual void init_visualization();
protected:
    float m_radius;
};

class Torus : public Primitive // y-axis-aligned torus 
{
public:
	Torus(float R, float r) : Primitive(TORUS), m_major_radius(R), m_minor_radius(r) { init_visualization(); };
	Torus(const glm::vec3 pos, float R, float r) : Primitive(TORUS, pos), m_major_radius(R), m_minor_radius(r) { init_visualization(); };
	Torus(const glm::vec3 pos, const glm::vec3 vel, float R, float r) : Primitive(TORUS, pos, vel), m_major_radius(R), m_minor_radius(r) { init_visualization(); };
	Torus(const glm::vec3 pos, const glm::vec3 vel, bool has_gravity, float R, float r) : Primitive(TORUS, pos, vel, has_gravity), m_major_radius(R), m_minor_radius(r) { init_visualization(); };
	Torus(const Torus& other) : Primitive(other), m_major_radius(other.m_major_radius), m_minor_radius(other.m_minor_radius) {};
	virtual ~Torus() {};

	virtual bool StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist);
protected:
	virtual void init_visualization();
protected:
	float m_major_radius;
	float m_minor_radius;
};

class Cube : public Primitive
{
public:
	Cube() : Primitive(CUBE), m_hf_dims(glm::vec3(DEFAULT_SELECTION_RADIUS, DEFAULT_SELECTION_RADIUS, DEFAULT_SELECTION_RADIUS)) { init_visualization(); };
    Cube(float hf_x, float hf_y, float hf_z) : Primitive(CUBE), m_hf_dims(glm::vec3(hf_x, hf_y, hf_z)) {init_visualization();};
    Cube(glm::vec3 hf_dims) : Primitive(CUBE), m_hf_dims(hf_dims) {init_visualization();};
	Cube(const glm::vec3 pos, glm::vec3 hf_dims) : Primitive(CUBE, pos), m_hf_dims(hf_dims) { init_visualization(); };
	Cube(const glm::vec3 pos, const glm::vec3 vel, glm::vec3 hf_dims) : Primitive(CUBE, pos, vel), m_hf_dims(hf_dims) { init_visualization(); };
	Cube(const Cube& other) : Primitive(other), m_hf_dims(other.m_hf_dims) {};
    virtual ~Cube() {};

	virtual void move_to(const glm::vec3& target);
	virtual bool StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist);
protected:
    virtual void init_visualization();
protected:
	glm::vec3 m_hf_dims;
};

class ObjMesh : public Primitive
{
public:
    ObjMesh(char* filename) : Primitive(OBJMESH), m_scaling(1.0) {read_from_file(filename);};
    ObjMesh(char* filename, float scaling) : Primitive(OBJMESH), m_scaling(scaling) {read_from_file(filename);};
    ObjMesh(char* filename, const glm::vec3 pos, float scaling) : Primitive(OBJMESH, pos), m_scaling(scaling) {read_from_file(filename);};
    ObjMesh(const ObjMesh& other) : Primitive(other), m_scaling(other.m_scaling) {};
    virtual ~ObjMesh() {};

	virtual bool StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist);
protected:
	void read_from_file(char* filename);
protected:
	char* m_filename;
    float m_scaling;
};

#endif