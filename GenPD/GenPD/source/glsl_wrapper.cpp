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

#include <iostream>
#include "glsl_wrapper.h"
#include "stb_image.h"

RenderWrapper::RenderWrapper()
{
}

RenderWrapper::~RenderWrapper()
{
}

void RenderWrapper::InitShader(const char* vert_path, const char* frag_path)
{
    // create shaders and shader program
    m_vert_handle = glCreateShader(GL_VERTEX_SHADER);
    m_frag_handle = glCreateShader(GL_FRAGMENT_SHADER);
    m_shaderprog_handle = glCreateProgram();

    // load shader source from file
    const char* vert_source = textFileRead(vert_path);
    const char* frag_source = textFileRead(frag_path);
    glShaderSource(m_vert_handle, 1, &vert_source, NULL);
    glShaderSource(m_frag_handle, 1, &frag_source, NULL);
    glCompileShader(m_vert_handle);
    glCompileShader(m_frag_handle);

    // compile shader source
    GLint compiled;
    glGetShaderiv(m_vert_handle, GL_COMPILE_STATUS, &compiled);
    if(!compiled)
        printShaderInfoLog(m_vert_handle);
    glGetShaderiv(m_frag_handle, GL_COMPILE_STATUS, &compiled);
    if(!compiled)
        printShaderInfoLog(m_frag_handle);

    // bind attribute locations for the shaders
    // 0 for position, 1 for color, 2 for normal.
    glBindAttribLocation(m_shaderprog_handle, 0, "v_position");
    glBindAttribLocation(m_shaderprog_handle, 1, "v_color");
    glBindAttribLocation(m_shaderprog_handle, 2, "v_normal");
	glBindAttribLocation(m_shaderprog_handle, 3, "v_texcoord");

    // attach shader to the shader program
    glAttachShader(m_shaderprog_handle, m_vert_handle);
    glAttachShader(m_shaderprog_handle, m_frag_handle);
    glLinkProgram(m_shaderprog_handle);
    GLint linked;
    glGetProgramiv(m_shaderprog_handle, GL_LINK_STATUS, &linked);
    if(!linked)
        printLinkInfoLog(m_shaderprog_handle);

    // query uniform locations from openGL.
	m_vbo_handle.m_uniform_modelview = glGetUniformLocation(m_shaderprog_handle, "u_modelviewMatrix");
	m_vbo_handle.m_uniform_projection = glGetUniformLocation(m_shaderprog_handle, "u_projMatrix");
	m_vbo_handle.m_uniform_transformation = glGetUniformLocation(m_shaderprog_handle, "u_transformMatrix");
	m_vbo_handle.m_uniform_enable_texture = glGetUniformLocation(m_shaderprog_handle, "u_choose_tex");
	m_vbo_handle.m_uniform_texture_sampler = glGetUniformLocation(m_shaderprog_handle, "u_sampler1");

    // activate the shader program.
    glUseProgram(m_shaderprog_handle);
}

bool RenderWrapper::InitTexture(const char* tex_path)
{
	int x,y,n;
    unsigned char *data = stbi_load(tex_path, &x, &y, &n, 0);

	if (data == NULL) {
		fprintf(stderr, "Error, texture was missing.\n");
		return false;
    } else {
		GLint mode;
		if (n == 4) 
		{
			mode = GL_RGBA;
		}
		else if (n == 3) 
		{
			mode = GL_RGB;
		}
		else
		{
			fprintf(stderr, "Error, texture formap unknown.\n");
		}
		
		glGenTextures(1, &m_texture);
		glBindTexture(GL_TEXTURE_2D, m_texture);
		glTexImage2D(GL_TEXTURE_2D, 0, mode, x, y, 0, mode, GL_UNSIGNED_BYTE, data);
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		glActiveTexture(GL_TEXTURE0);
		glUniform1i(m_vbo_handle.m_uniform_enable_texture, 0);
		glBindTexture(GL_TEXTURE_2D, m_texture);

		free(data);
		return true;
    }
}

void RenderWrapper::CleanupShader()
{
    glDetachShader(m_shaderprog_handle, m_vert_handle);
    glDetachShader(m_shaderprog_handle, m_frag_handle);
    glDeleteShader(m_vert_handle);
    glDeleteShader(m_frag_handle);
    glDeleteProgram(m_shaderprog_handle);
}

void RenderWrapper::SetCameraProjection(glm::mat4 projection)
{
    glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(&projection[0][0]);
	
	ActivateShaderprog();
	glUniformMatrix4fv(m_vbo_handle.m_uniform_projection, 1, false, &projection[0][0]);
}

void RenderWrapper::SetCameraModelview(glm::mat4 modelview)
{
    glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(&modelview[0][0]);
	
	ActivateShaderprog();
	glUniformMatrix4fv(m_vbo_handle.m_uniform_modelview, 1, false, &modelview[0][0]);
}

void RenderWrapper::ActivateShaderprog()
{
    GLint current_prog;
    glGetIntegerv(GL_CURRENT_PROGRAM, &current_prog);
    if(current_prog != (GLint)m_shaderprog_handle)
        glUseProgram(m_shaderprog_handle);
}

void RenderWrapper::DeactivateShaderprog()
{
    GLint current_prog;
    glGetIntegerv(GL_CURRENT_PROGRAM, &current_prog);
    if(current_prog == (GLint)m_shaderprog_handle)
        glUseProgram(0);
}

// private fields
// helper function to read shader source and put it in a char array
// thanks to Swiftless
char* RenderWrapper::textFileRead(const char* fileName) 
{
    char* text = NULL;

    if (fileName != NULL) {
		FILE* file;
        fopen_s(&file, fileName, "rt");

        if (file != NULL) {
            fseek(file, 0, SEEK_END);
            int count = ftell(file);
            rewind(file);

            if (count > 0) {
                text = (char*)malloc(sizeof(char) * (count + 1));
                count = fread(text, sizeof(char), count, file);
                text[count] = '\0';	//cap off the string with a terminal symbol, fixed by Cory
            }
            fclose(file);
        }
    }
    return text;
}

void RenderWrapper::printLinkInfoLog(int prog) 
{
    int infoLogLen = 0;
    int charsWritten = 0;
    GLchar *infoLog;

    glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &infoLogLen);

    // should additionally check for OpenGL errors here

    if (infoLogLen > 0)
    {
        infoLog = new GLchar[infoLogLen];
        // error check for fail to allocate memory omitted
        glGetProgramInfoLog(prog,infoLogLen, &charsWritten, infoLog);
        std::cout << "InfoLog:" << std::endl << infoLog << std::endl;
        delete [] infoLog;
    }
}

void RenderWrapper::printShaderInfoLog(int shader)
{
    int infoLogLen = 0;
    int charsWritten = 0;
    GLchar *infoLog;

    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLogLen);

    // should additionally check for OpenGL errors here

    if (infoLogLen > 0)
    {
        infoLog = new GLchar[infoLogLen];
        // error check for fail to allocate memory omitted
        glGetShaderInfoLog(shader,infoLogLen, &charsWritten, infoLog);
        std::cout << "InfoLog:" << std::endl << infoLog << std::endl;
        delete [] infoLog;
    }

    // should additionally check for OpenGL errors here
}
