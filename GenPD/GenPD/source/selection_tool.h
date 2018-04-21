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


#ifndef _SELECTION_TOOL_H_
#define _SELECTION_TOOL_H_

#include <vector>

#include "opengl_headers.h"
#include "math_headers.h"
#include "anttweakbar_wrapper.h"
#include "simulation.h"

// forward declarationg
class AntTweakBarWrapper;
class Simulation;

typedef enum
{
	GUI_MODE_SELECTION,
	GUI_MODE_TRANSLATION,
	GUI_MODE_ROTATION

} GUIMode;

// a bunch of functions for selection, moving and rotating
class SelectionTool
{
	friend class AntTweakBarWrapper;

public:
	SelectionTool() {}
	virtual ~SelectionTool() {}

	inline void SetMode(GUIMode mode) {Reset(); m_gui_mode = mode;}
	inline GUIMode GetMode() {return m_gui_mode;}
	inline std::vector<unsigned int>& SelectedIndices() {return m_selected_indices;}
	inline std::vector<glm::vec3>& SelectedVertices() {return m_selected_vertices;}

	void Reset();

	// select
	// box selection for vertices (selection mode: click and drag) 
	void SelectFirstPoint(int x, int y, int width, int height, int selection_mode); // 1 = new, 2 = append, 4 = delete
	void SelectSecondPoint(int x, int y, int width, int height, bool finalize = false);
	void SelectVertices(const std::vector<glm::vec3>& vertices, glm::mat4 mvp); // 1 = new, 2 = append, 4 = delete
	void SelectVerticesHardCoded(const std::vector<glm::vec3>& vertices); 
	// hover selection for handles (translate or rotate mode: hover to select)
	bool HoverSelectHandle(Simulation* sim, const std::vector<glm::vec3>& ray, glm::mat4 mvp); 
	// !select

	// translate (translate mode: drag to translate, finalize when release button)
	void TranslateRotateFirstPoint();

	// !translate

	// rotate (rotation mode: drag to rotate, finalize when release button)

	// !rotate

	// visualize
	void Draw();
	// circle on hovered handle
	void DrawHoverCircle();
	// 2d lines and circles
	void DrawSelectionBox();
	void DrawTranslationArrows();
	void DrawRotationCircle();
	// 3d cubes for selection
	void HighlightSelectedVertices(const VBO& vbos);

	//// accesser
	//inline const float& P1X() { return p1x; }
	//inline const float& P1Y() { return p1y; }
	//inline const float& P2X() { return p2x; }
	//inline const float& P2Y() { return p2y; }

protected:
	GUIMode m_gui_mode;
	// selection;
	float x_min, x_max, y_min, y_max;
	float p1x, p1y, p2x, p2y;
	int m_selection_mode; // 1 = new, 2 = append, 4 = delete
	std::vector<unsigned int> m_selected_indices;
	std::vector<glm::vec3> m_selected_vertices;

	// visualization
	bool m_draw_selection_box;
	bool m_draw_translation_rotation;
	bool m_hover_with_handle;

protected:
	bool inside_selection_box(const glm::vec4& p);
};

#endif