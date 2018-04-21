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

#ifndef _IO_MESH_H_
#define _IO_MESH_H_

#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include "glm.hpp"

class MeshLoader{
public:
	
	struct Face{
		unsigned int id1,id2,id3;
		Face() {}
		Face(int a, int b, int c) : id1(a), id2(b), id3(c){}
		void IDMinusMinus() {id1--; id2--; id3--;}
	};

	struct Tet{
		unsigned int id1,id2,id3,id4;
		Tet() {}
		Tet(int a, int b, int c, int d) : id1(a), id2(b), id3(c), id4(d){}
		void IDMinusMinus() {id1--; id2--; id3--; id4--;}
	};

	MeshLoader();
	MeshLoader(char* filename, float scale = 10.0f, bool flip = false, float theta = 0.0, glm::vec3 translate = glm::vec3(0.0f, 0.0f, 0.0f));
	virtual ~MeshLoader();

	inline bool Info() {return load_success;}

	//Vertices, edges, and faces information
	std::vector<glm::vec3> m_vertices;
	std::vector<Face> m_faces;
	std::vector<Tet> m_tets;
	
	bool load_success;
};

#endif