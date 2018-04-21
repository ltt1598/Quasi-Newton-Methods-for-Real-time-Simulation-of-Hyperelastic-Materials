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

#ifndef _OPENGL_HEADERS_H_
#define _OPENGL_HEADERS_H_

// opengl headers
#include "GL/glew.h"
#include "GL/freeglut.h" // note: glut needs to be included after glew

struct VBO
{
    VBO()
    {
        if(!glIsBuffer(m_vbo))
            glGenBuffers(1, &m_vbo);
        if(!glIsBuffer(m_cbo))
            glGenBuffers(1, &m_cbo);
        if(!glIsBuffer(m_nbo))
            glGenBuffers(1, &m_nbo);
        if(!glIsBuffer(m_tbo))
            glGenBuffers(1, &m_tbo);
        if(!glIsBuffer(m_ibo))
            glGenBuffers(1, &m_ibo);
    }

    virtual ~VBO()
    {
        if(glIsBuffer(m_vbo))
            glDeleteBuffers(1, &m_vbo);
        if(glIsBuffer(m_cbo))
            glDeleteBuffers(1, &m_cbo);
        if(glIsBuffer(m_nbo))
            glDeleteBuffers(1, &m_nbo);
        if(glIsBuffer(m_tbo))
            glDeleteBuffers(1, &m_tbo);
        if(glIsBuffer(m_ibo))
            glDeleteBuffers(1, &m_ibo);
    }
    // vertex, color, normal, texture, index
    GLuint m_vbo, m_cbo, m_nbo, m_tbo, m_ibo;

	//0: modelview; 1: projection; 2: transformation; 3: enable_texture; 4: texture_sampler
	GLuint m_uniform_modelview;
	GLuint m_uniform_projection;
	GLuint m_uniform_transformation;
	GLuint m_uniform_enable_texture;
	GLuint m_uniform_texture_sampler;
};

#endif