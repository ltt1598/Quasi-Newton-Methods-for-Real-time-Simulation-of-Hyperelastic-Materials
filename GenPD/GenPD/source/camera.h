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

#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "openGL_headers.h"
#include "global_headers.h"
#include "math_headers.h"
#include "mesh.h"

//forward declaration
class Mesh;

class Camera
{
public:
	// constructor and destructor
	Camera(void);
	virtual ~Camera(void);

	// reset / save / load
	void Reset(int width, int height);
	void SaveCamera();
	void LoadCamera();

	void Lookat(Mesh* mesh);
	
	// get camera matrices:
	inline glm::mat4 GetViewMatrix() {return m_view;}
	inline glm::mat4 GetProjectionMatrix() {return m_projection;}
	inline glm::mat4 GetMVP() { return m_projection*m_view; }

	// get camera position and raycast direction:
	inline glm::vec3 GetCameraPosition() {return m_position;}
	inline float GetCameraDistance() {return m_eye_distance;}
	inline void SetProjectionPlaneDistance(float distance) {m_cached_projection_plane_distance = distance;}
	glm::vec3 GetRaycastDirection(int x, int y);
	glm::vec3 GetCurrentTargetPoint(int x, int y);

	void GetCurrentRotation(int x, int y, glm::vec3& axis, ScalarType& theta);
	std::vector<glm::vec3> CastRay(int x, int y);
	inline void CacheLastSelectedPointLocalCoM(const glm::vec3& point) { m_cached_last_selection_local_com = point; }
	inline void CacheLastSelectedPointGlobalCoM(const glm::vec3& point) { m_cached_last_selection_global_com = point; }

	// mouse interactions
	void MouseChangeDistance(float coe, float dx, float dy);
	void MouseChangeLookat(float coe, float dx, float dy);
	void MouseChangeHeadPitch(float coe, float dx, float dy);

	// Draw axis
	void DrawAxis();

	// resize
	void ResizeWindow(int w, int h);

protected:
	int m_width;
	int m_height;
	float m_znear;
	float m_zfar;
	float m_fovy;

	float m_eye_distance;
	float m_head;
	float m_pitch;

	glm::vec3 m_position;
	glm::vec3 m_up;
	glm::vec3 m_lookat;
	glm::vec3 m_cached_projection_plane_center;
	glm::vec3 m_cached_projection_plane_xdir;
	glm::vec3 m_cached_projection_plane_ydir;
	float m_cached_projection_plane_distance;
	glm::vec3 m_cached_last_selection_local_com;
	glm::vec3 m_cached_last_selection_global_com;

	glm::mat4 m_view;
	glm::mat4 m_projection;
private:
	// update camera matrices:
	void updateViewMatrix();
	void updateProjectionMatrix();
};

#endif