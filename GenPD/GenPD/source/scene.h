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

#ifndef _SCENE_H_
#define _SCENE_H_

#include <vector>

#include "openGL_headers.h"
#include "math_headers.h"
#include "global_headers.h"
#include "primitive.h"
#include "tinyxml2.h"

class Scene;
class XMLSceneVisitor;

class Scene
{
public:
    Scene(const char* file_name);
    virtual ~Scene();

	void Reset();
    void LoadFromFile(const char* file_name);
    virtual void Draw(const VBO& vbos);
	void Update(ScalarType h, unsigned int start_frame = -1);
    void InsertPrimitve(Primitive* const new_primitive);

	bool StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist);

	bool IsEmpty() { return m_primitives.empty(); }

protected:
    std::vector<Primitive*> m_primitives;
	char m_file_name[255];

private:
};

class XMLSceneVisitor : public tinyxml2::XMLVisitor
{
public:
	XMLSceneVisitor(Scene* const scene);
	XMLSceneVisitor(const XMLSceneVisitor& other);
	virtual ~XMLSceneVisitor();

	virtual bool VisitEnter(const tinyxml2::XMLElement& element, const tinyxml2::XMLAttribute* attribute);
	virtual bool VisitExit(const tinyxml2::XMLElement& element);

protected:
	Scene* const m_scene;
	Primitive* m_current;
};

#endif