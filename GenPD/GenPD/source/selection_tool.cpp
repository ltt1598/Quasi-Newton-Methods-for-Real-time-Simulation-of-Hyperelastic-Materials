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


#include <iostream>
#include "selection_tool.h"
#include "primitive.h"

extern int g_screen_width;
extern int g_screen_height;

void SelectionTool::Reset()
{
	m_gui_mode = GUI_MODE_SELECTION;
	m_draw_selection_box = false;
	m_draw_translation_rotation = false;
	m_hover_with_handle = false;
	m_selection_mode = 1;
	m_selected_indices.clear();
	m_selected_vertices.clear();
}

void SelectionTool::SelectFirstPoint(int x, int y, int width, int height, int selection_mode)
{
	if (m_gui_mode == GUI_MODE_SELECTION)
	{
		p1x = (float)x/(float)width;
		p1y = (float)y/(float)height;

		p1x = 2.0f*p1x-1.0f;
		p1y = 1.0f-2.0f*p1y;

		p2x = p1x; p2y = p1y;

		m_draw_selection_box = true;
		m_selection_mode = selection_mode;
	}
	else // either select the center of a selected handle, or nothing
	{

	}
}

void SelectionTool::SelectSecondPoint(int x, int y, int width, int height, bool finalize)
{
	p2x = (float)x/(float)width;
	p2y = (float)y/(float)height;

	p2x = 2.0f*p2x-1.0f;
	p2y = 1.0f-2.0f*p2y;

	//std::cout << "p = (" << p2x << ", " << p2y << ")" << std::endl;

	if (finalize)
	{
		if (m_gui_mode == GUI_MODE_SELECTION)
		{
			if (p1x > p2x)
			{
				x_max = p1x;
				x_min = p2x;
			}
			else
			{
				x_max = p2x;
				x_min = p1x;
			}

			if (p1y > p2y)
			{
				y_max = p1y;
				y_min = p2y;
			}
			else
			{
				y_max = p2y;
				y_min = p1y;
			}
			m_draw_selection_box = false;
		}
		else
		{
			m_draw_translation_rotation = false;
		}
	}
}

void SelectionTool::SelectVertices(const std::vector<glm::vec3>& vertices, glm::mat4 mvp) // 1 = new, 2 = append, 4 = delete
{
	switch (m_selection_mode)
	{
		case 1: // new
			m_selected_indices.clear(); 
			m_selected_vertices.clear();
			// no break, go on!!!, the only difference between append and new is that "new" needs clear the selected indices from the beginning
		case 2: // append
			for (unsigned int i = 0; i < vertices.size(); ++i)
			{
				glm::vec4 v = glm::vec4(vertices[i], 1.0f);
				v = mvp*v;
				if (inside_selection_box(v))
				{
					// selected, add the index of it to selected_indices
					if (m_selection_mode == 1)
					{
						m_selected_indices.push_back(i);
						m_selected_vertices.push_back(vertices[i]);
					}
					else // one more check for append, check if it is already in the array
					{
						if (std::find(m_selected_indices.begin(), m_selected_indices.end(), i) == m_selected_indices.end())
						{
							m_selected_indices.push_back(i);
							m_selected_vertices.push_back(vertices[i]);
						}
					}
				}
			}
			break;
		case 4: // delete
			for (int i = m_selected_indices.size()-1; i >= 0; --i)
			{
				glm::vec4 v = glm::vec4(vertices[m_selected_indices[i]], 1.0f);
				v = mvp*v;
				if (inside_selection_box(v))
				{
					// selected, delete this selection, don't move the iterator
					m_selected_indices.erase(m_selected_indices.begin() + i);
					m_selected_vertices.erase(m_selected_vertices.begin() + i);
				}
			}
			break;
	}
	//std::cout << "selected_indices: ";
	//for (unsigned int i = 0; i < m_selected_indices.size(); ++i)
	//{
	//	std::cout << m_selected_indices[i] << " ";
	//}
	//std::cout << std::endl;
}

void SelectionTool::SelectVerticesHardCoded(const std::vector<glm::vec3>& vertices)
{
	ScalarType y_value;
	std::cout << "select vertices whose y-axis projection is larger than: (please input): \n";
	std::cin >> y_value;

	for (unsigned int i = 0; i < vertices.size(); ++i)
	{
		const glm::vec3& v = vertices[i];

		if (v.y >= y_value)
		{
			m_selected_indices.push_back(i);
			m_selected_vertices.push_back(vertices[i]);
		}
	}
}

bool SelectionTool::HoverSelectHandle(Simulation* sim, const std::vector<glm::vec3>& ray, glm::mat4 mvp)
{
	m_hover_with_handle = false;
	if (sim->SelectHandle(ray)) // selected
	{
		glm::vec4 CoM = mvp * glm::vec4(sim->SelectedHandleCoM(), 1.0f);
		p1x = CoM.x/CoM.w;
		p1y = CoM.y/CoM.w;
		m_hover_with_handle = true; // hover
	}
	return m_hover_with_handle;
}

void SelectionTool::TranslateRotateFirstPoint()
{
	m_draw_translation_rotation = true;
	p2x = p1x; p2y = p1y;
}

void SelectionTool::Draw()
{
	DrawHoverCircle();
	switch(m_gui_mode)
	{
	case GUI_MODE_SELECTION:
		DrawSelectionBox();
		break;
	case GUI_MODE_TRANSLATION:
		DrawTranslationArrows();
		break;
	case GUI_MODE_ROTATION:
		DrawRotationCircle();
		break;
	}
}

void SelectionTool::DrawHoverCircle()
{
	if (m_hover_with_handle == true && m_draw_translation_rotation == false)
	{
		glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT);
		glDisable(GL_LIGHTING);
		glDisable(GL_DEPTH);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		float aspect = (float)g_screen_width/(float)g_screen_height;
		float radius = 0.2f;
		if (m_gui_mode == GUI_MODE_ROTATION )
		{
			float x, y, theta;
			int segments = 60;
			//Draw axis.
			glBegin(GL_LINE_LOOP);
			glColor3f(1.0f, 0.2f, 0.2f);
			for (unsigned int i = 0; i < segments; i++)
			{
				theta = 3.1415926535f * 2 / segments * i;
				x = p1x + std::cos(theta) * radius / aspect;
				y = p1y + std::sin(theta) * radius;
				glVertex2f(x, y);
			}
			glEnd();
		}
		else if (m_gui_mode == GUI_MODE_TRANSLATION)
		{
			glBegin(GL_LINES);

			glColor3f(0.2f, 1.0f, 0.2f);
			glVertex2f(p1x, p1y);
			glVertex2f(p1x, p1y+radius);

			glColor3f(1.0f, 0.2f, 0.2f);
			glVertex2f(p1x, p1y);
			glVertex2f(p1x+radius/aspect, p1y);

			glEnd();
		}
		// restore everything
		glPopMatrix();
		glPopMatrix();
		glPopAttrib();
		glEnable(GL_LIGHTING);
		glEnable(GL_DEPTH);
	}
}

void SelectionTool::DrawSelectionBox()
{
	if (m_draw_selection_box)
	{
		glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT);
		glDisable(GL_LIGHTING);
		glDisable(GL_DEPTH);
		
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glLineStipple(4, 0xAAAA);
		glEnable(GL_LINE_STIPPLE);
		glBegin(GL_LINE_LOOP);
		switch (m_selection_mode)
		{
		case 1: // new
			glColor3f(0.2f, 0.2f, 1.0f);
			break;
		case 2: // append
			glColor3f(0.2f, 1.0f, 0.2f);
			break;
		case 4: // delete
			glColor3f(1.0f, 0.2f, 0.2f);
			break;
		}
		glVertex2f(p1x, p1y);
		glVertex2f(p1x, p2y);
		glVertex2f(p2x, p2y);
		glVertex2f(p2x, p1y);
		glEnd();
		glDisable(GL_LINE_STIPPLE);

		// restore everything
		glPopMatrix();
		glPopMatrix();
		glPopAttrib();
		glEnable(GL_LIGHTING);
		glEnable(GL_DEPTH);
	}
}
void SelectionTool::DrawTranslationArrows()
{
	if (m_hover_with_handle && m_draw_translation_rotation)
	{
		glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT);
		glDisable(GL_LIGHTING);
		glDisable(GL_DEPTH);
		
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glBegin(GL_LINES);
		glColor3f(0.2f, 0.2f, 1.0f);
		glVertex2f(p1x, p1y);
		glVertex2f(p2x, p2y);
		glEnd();

		// restore everything
		glPopMatrix();
		glPopMatrix();
		glPopAttrib();
		glEnable(GL_LIGHTING);
		glEnable(GL_DEPTH);
	}
}
void SelectionTool::DrawRotationCircle()
{
	if (m_hover_with_handle && m_draw_translation_rotation)
	{
		glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT);
		glDisable(GL_LIGHTING);
		glDisable(GL_DEPTH);
		
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		//gluOrtho2D(- (float)g_screen_width/(float)g_screen_height, (float)g_screen_width/(float)g_screen_height, -1.0f, 1.0f);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		float aspect = (float)g_screen_width/(float)g_screen_height;
		float x, y, radius, theta;
		radius = std::sqrt((p1x-p2x)*(p1x-p2x) * (aspect*aspect) + (p1y-p2y)*(p1y-p2y));
		int segments = 60;
		//Draw axis.
		glColor3f(1.0f, 0.2f, 0.2f);
		glBegin(GL_LINE_LOOP);
		for (unsigned int i = 0; i < segments; i++)
		{
			theta = 3.1415926535f * 2 / segments * i;
			x = p1x + std::cos(theta) * radius / aspect;
			y = p1y + std::sin(theta) * radius;
			glVertex2f(x, y);
		}
		glEnd();

		glBegin(GL_LINES);
		glColor3f(0.2f, 0.2f, 1.0f);
		glVertex2f(p1x, p1y);
		glVertex2f(p2x, p2y);
		glEnd();

		glLineStipple(4, 0xAAAA);
		glEnable(GL_LINE_STIPPLE);
		glBegin(GL_LINES);
		glColor3f(0.2f, 1.0f, 0.2f);
		glVertex2f(p1x, p1y);
		glVertex2f(p1x+radius / aspect, p1y);
		glEnd();
		glDisable(GL_LINE_STIPPLE);

		// restore everything
		glPopMatrix();
		glPopMatrix();
		glPopAttrib();
		glEnable(GL_LIGHTING);
		glEnable(GL_DEPTH);
	}
}

void SelectionTool::HighlightSelectedVertices(const VBO& vbos)
{
	Cube cube(DEFAULT_SELECTION_RADIUS, DEFAULT_SELECTION_RADIUS, DEFAULT_SELECTION_RADIUS);
	cube.change_color(glm::vec3(0.3f, 0.3f, 1.0f));

	for (std::vector<glm::vec3>::iterator it = m_selected_vertices.begin(); it != m_selected_vertices.end(); ++it)
	{
		cube.move_to(*it);
		cube.Draw(vbos);
	}
}

bool SelectionTool::inside_selection_box(const glm::vec4& p)
{
	float px = p.x/p.w;
	float py = p.y/p.w;

	if (px > x_min && px < x_max && py > y_min && py < y_max)
	{
		return true;
	}
	else
	{
		return false;
	}
}