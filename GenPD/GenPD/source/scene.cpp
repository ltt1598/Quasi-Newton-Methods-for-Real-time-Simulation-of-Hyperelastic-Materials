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

#pragma warning( disable : 4244 4267)

#include <cassert>

#include "scene.h"

//----------Scene Class----------//
Scene::Scene(const char* file_name)
{
	strcpy_s(m_file_name, file_name);
	Reset();
}

Scene::~Scene()
{
    for(std::vector<Primitive*>::iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
    {
        delete (*iter);
    }
    m_primitives.clear();
}

void Scene::Reset()
{
	for (std::vector<Primitive*>::iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
	{
		delete (*iter);
	}

	m_primitives.clear();
	LoadFromFile(m_file_name);
}

void Scene::LoadFromFile(const char* file_name)
{
	tinyxml2::XMLDocument xml_file;
	if (xml_file.LoadFile(file_name) == tinyxml2::XML_NO_ERROR)
	{
		XMLSceneVisitor visitor(this);
		xml_file.Accept(&visitor);
	}
}

void Scene::Draw(const VBO& vbos)
{
    for(std::vector<Primitive*>::iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
    {
        (*iter)->Draw(vbos);
    }
}

void Scene::Update(ScalarType h, unsigned int start_frame)
{
	for (std::vector<Primitive*>::iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
	{
		(*iter)->Update(h, start_frame);
	}
}

bool Scene::StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist)
{
	dist = 0;

    for(std::vector<Primitive*>::iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
    {
		ScalarType d;
		EigenVector3 n;
		if((*iter)->StaticIntersectionTest(p, n, d))
		{
			if (d < dist)
			{
				dist = d;
				normal = n;
			}
		}
    }

	if (dist < 0)
		return true;
	else
		return false;
}

void Scene::InsertPrimitve(Primitive* const new_primitive)
{
    m_primitives.push_back(new_primitive);
}

//----------Scene Visitor Class----------//
XMLSceneVisitor::XMLSceneVisitor(Scene* const scene) : 
    tinyxml2::XMLVisitor(),
    m_scene(scene),
    m_current(NULL)
{
    ;
}

XMLSceneVisitor::XMLSceneVisitor(const XMLSceneVisitor& other) : 
	tinyxml2::XMLVisitor(other),
    m_scene(other.m_scene),
    m_current(other.m_current)
{
    ;
}

XMLSceneVisitor::~XMLSceneVisitor()
{
    ;
}

bool XMLSceneVisitor::VisitEnter(const tinyxml2::XMLElement& element, const tinyxml2::XMLAttribute* attribute)
{
    if(std::string(element.Value()) == "scene")
    {
        return (element.Parent() == element.GetDocument());
    }
	else if (std::string(element.Value()) == "materials")
    {
        return true;
    }
	else if (std::string(element.Value()) == "primitives")
    {
		return (std::string(element.Parent()->Value()) == "scene");
    }
	else if (std::string(element.Value()) == "plane")
    {
		if (std::string(element.Parent()->Value()) != "primitives")
            return false;
        assert(m_current == NULL);

        double nx(0.0), ny(1.0), nz(0.0), value(0.0);

		nx = element.DoubleAttribute("nx");
		ny = element.DoubleAttribute("ny");
		nz = element.DoubleAttribute("nz");
		value = element.DoubleAttribute("value");

        glm::vec3 normal(nx, ny, nz);
        normal = glm::normalize(normal);

        m_current = new Plane(normal, value);
        return true;
    }
	else if (std::string(element.Value()) == "sphere")
    {
		if (std::string(element.Parent()->Value()) != "primitives")
            return false;
        assert(m_current == NULL);

        double cx(0.0), cy(1.0), cz(0.0);

		cx = element.DoubleAttribute("cx");
		cy = element.DoubleAttribute("cy");
		cz = element.DoubleAttribute("cz");
		glm::vec3 center(cx, cy, cz);

		double vx(0.0), vy(0.0), vz(0.0);
		vx = element.DoubleAttribute("vx");
		vy = element.DoubleAttribute("vy");
		vz = element.DoubleAttribute("vz");
		glm::vec3 velocity(vx, vy, vz);

		double radius(0.0);
		radius = element.DoubleAttribute("radius");

		int start_frame(0);
		start_frame = element.IntAttribute("start");

		m_current = new Sphere(center, velocity, radius);
		m_current->StartFrame() = start_frame;
		return true;
    }
	else if (std::string(element.Value()) == "torus")
	{
		if (std::string(element.Parent()->Value()) != "primitives")
			return false;
		assert(m_current == NULL);

		double cx(0.0), cy(0.0), cz(0.0);
		cx = element.DoubleAttribute("cx");
		cy = element.DoubleAttribute("cy");
		cz = element.DoubleAttribute("cz");
		glm::vec3 center(cx, cy, cz);

		double vx(0.0), vy(0.0), vz(0.0);
		vx = element.DoubleAttribute("vx");
		vy = element.DoubleAttribute("vy");
		vz = element.DoubleAttribute("vz");
		glm::vec3 velocity(vx, vy, vz);

		double R(4.0), r(1.0);

		R = element.DoubleAttribute("major");
		r = element.DoubleAttribute("minor");

		//m_current = new Sphere(center, velocity, R);
		m_current = new Torus(center, velocity, R, r);
		return true;
	}
	else if (std::string(element.Value()) == "cube")
    {
		if (std::string(element.Parent()->Value()) != "primitives")
            return false;
        assert(m_current == NULL);

        double cx(0.0), cy(1.0), cz(0.0), hx(0.0), hy(0.0), hz(0.0);

		cx = element.DoubleAttribute("cx");
		cy = element.DoubleAttribute("cy");
		cz = element.DoubleAttribute("cz");
		hx = element.DoubleAttribute("hx");
		hy = element.DoubleAttribute("hy");
		hz = element.DoubleAttribute("hz");

        glm::vec3 center(cx, cy, cz);
        glm::vec3 hf_dims(hx, hy, hz);
 
		double vx(0.0), vy(0.0), vz(0.0);
		vx = element.DoubleAttribute("vx");
		vy = element.DoubleAttribute("vy");
		vz = element.DoubleAttribute("vz");
		glm::vec3 velocity(vx, vy, vz);

		m_current = new Cube(center, velocity, hf_dims);
        return true;
    }
  //  else if (element.ValueStr() == "obj")
  //  {
  //      if(element.Parent()->ValueStr() != "primitives")
  //          return false;
  //      assert(m_current == NULL);

  //      double cx(0.0), cy(1.0), cz(0.0), scale(0.0);

  //      element.Attribute("cx", &cx);
  //      element.Attribute("cy", &cy);
  //      element.Attribute("cz", &cz);
  //      element.Attribute("scale", &scale);

  //      glm::vec3 center(cx, cy, cz);
		//m_current = new ObjMesh(DEFAULT_OBJ_MODEL, center, scale);
  //      return true;
  //  }
    else
        return false;
}

bool XMLSceneVisitor::VisitExit( const tinyxml2::XMLElement& element)
{
	if (std::string(element.Value()) == "scene")
    {
        return (element.Parent() == element.GetDocument());
    }
	else if (std::string(element.Value()) == "materials")
    {
        return true;
    }
	else if (std::string(element.Value()) == "primitives")
    {
        return true;
    }
	else if (std::string(element.Value()) == "plane")
    {
        m_scene->InsertPrimitve(m_current);
        m_current = NULL;
        return true;
    }
	else if (std::string(element.Value()) == "sphere")
    {
        m_scene->InsertPrimitve(m_current);
        m_current = NULL;
        return true;
    }
	else if (std::string(element.Value()) == "torus")
	{
		m_scene->InsertPrimitve(m_current);
		m_current = NULL;
		return true;
	}
	else if (std::string(element.Value()) == "cube")
    {
        m_scene->InsertPrimitve(m_current);
        m_current = NULL;
        return true;
    }
	else if (std::string(element.Value()) == "obj")
    {
        m_scene->InsertPrimitve(m_current);
        m_current = NULL;
        return true;
    }
    else
        return false;
}