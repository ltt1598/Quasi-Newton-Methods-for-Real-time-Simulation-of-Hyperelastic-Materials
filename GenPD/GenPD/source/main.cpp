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

#pragma warning( disable : 4244)
#include <omp.h>

#include <iostream>
#include <string>

//----------Headers--------------//
#include "global_headers.h"
#include "math_headers.h"
#include "openGL_headers.h"
//----------Framework--------------//
#include "fps.h"
#include "timer_wrapper.h"
#include "stb_image_write.h"
#include "glsl_wrapper.h"
#include "AntTweakBar.h"
#include "anttweakbar_wrapper.h"
#include "camera.h"
#include "scene.h"
#include "selection_tool.h"
//----------Core--------------//
#include "mesh.h"
#include "simulation.h"

//----------Project Key Globals--------------//
AntTweakBarWrapper* g_config_bar;
Camera* g_camera;
RenderWrapper* g_renderer;
Scene* g_scene;
Mesh* g_mesh;
Simulation * g_simulation;
SelectionTool* g_selection_tool;
#ifdef ENABLE_MATLAB_DEBUGGING
#include "matlab_debugger.h"
MatlabDebugger * g_debugger;
#endif // ENABLE_MATLAB_DEBUGGING
TimerWrapper g_global_timer;
int g_interval = 100;
ScalarType total_time = 0;

//----------Global Parameters----------------//
int g_screen_width = DEFAULT_SCREEN_WIDTH;
int g_screen_height = DEFAULT_SCREEN_HEIGHT;
glm::vec3 g_handle_color;
bool g_random_handle_color;

//----------State Control--------------------//
bool g_only_show_sim = false;
bool g_record = false;
bool g_pause = true;
bool g_show_mesh = true;
bool g_show_wireframe = false;
int  g_wireframe_linewidth = 1;
bool g_show_texture = false;
bool g_texture_load_succeed = false;

//----------Mouse Control--------------------//
int g_mouse_old_x, g_mouse_old_y;
int g_mouse_wheel_pos;
unsigned char g_button_mask = 0x00;
bool g_mouse_down = false;

//----------Frame Rate/Frame Number----------//
mmc::FpsTracker g_fps_tracker;
int g_max_fps = 30;
int g_timestep = 1000 / g_max_fps;

//----------Recording Related----------------//
bool g_recording_limit = false;
int g_current_frame = 0;
int g_total_frame = 0;
bool g_export_obj = true;

//----------glut function handlers-----------//
void resize(int, int);
void timeout(int);
void display(void);
void key_press(unsigned char, int, int);
void mouse_click(int, int, int, int);
void mouse_motion(int, int);
void mouse_wheel(int, int, int, int);
void mouse_over(int, int);

//----------anttweakbar handlers----------//
void TW_CALL set_handle(void*);
void TW_CALL save_handle(void*);
void TW_CALL load_handle(void*);
void TW_CALL reset_handle(void*);
void TW_CALL reset_simulation(void*);
void TW_CALL step_through(void*);
void TW_CALL reset_camera(void*);
void TW_CALL set_partial_material_property(void*);
void TW_CALL matlab_reset_current_data(void*);
void TW_CALL matlab_reset_all_data(void*);
void TW_CALL matlab_set_converged_energy(void*);
void TW_CALL matlab_new_data(void*);
void TW_CALL matlab_remove_last_data(void*);
void TW_CALL matlab_export_data(void*);
void TW_CALL matlab_import_data(void*);
void TW_CALL matlab_plot(void*);
void TW_CALL matlab_plot_all(void*);
void TW_CALL matlab_visualize_vector(void*);

//----------other utility functions----------//
void init(void);
void cleanup(void);
void draw_overlay(void);
void grab_screen(void);
void grab_screen(char* filename);

//#include "src\plugins\BlockMethods.h"

inline const Eigen::Matrix3Xd::ConstColXpr getColumn(const Eigen::Matrix3Xd& A, unsigned int i)
{
	const Eigen::Matrix3Xd::ConstColXpr col = A.col(i);
	std::cout << "in function col address\n" << &col << std::endl;
	std::cout << "in function col\n" << col << std::endl;
	return col;
	//return A.col(i);
}

void test()
{
	Eigen::Matrix3Xd A(3, 5);
	A.setRandom();
	std::cout << "A address\n" << &A << std::endl;
	std::cout << "A data address\n" << A.data() << std::endl;
	std::cout << A << std::endl;

	const Eigen::Vector3d& a = A.col(0);
	std::cout << "a address\n" << &a << std::endl;
	std::cout << "a\n" << a << std::endl;

	const Eigen::Vector3d& b = getColumn(A, 0);
	std::cout << "b address\n" << &b << std::endl;
	std::cout << "b\n" << b << std::endl;

	return;
}

int main(int argc, char ** argv)
{
	//test();

    // gl init
    glutInit(&argc, argv);
#ifdef HIGH_PRECISION
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
#else
	glutInitDisplayMode(GLUT_RGBA);
#endif

    glutCreateWindow("Mass-Spring System Simulation T.L.");
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glutInitWindowSize(g_screen_width, g_screen_height);
    glViewport(0, 0, g_screen_width, g_screen_height);

    // user init
    init();
	glutReshapeWindow(g_screen_width, g_screen_height);

    // bind function callbacks
    glutDisplayFunc(display);
    glutTimerFunc(g_timestep, timeout, g_timestep);
    glutReshapeFunc(resize);
    glutKeyboardFunc(key_press);
    glutMouseFunc(mouse_click);
    glutMotionFunc(mouse_motion);
    glutPassiveMotionFunc(mouse_over);
    glutMouseWheelFunc(mouse_wheel);
    glutCloseFunc(cleanup);
    glutIdleFunc(display);

	omp_set_num_threads(6);

    glutMainLoop();

    return 0;
}

void resize(int width, int height) {
	g_screen_width = width;
	g_screen_height = height;
    //set the viewport, more boilerplate
    glViewport(0, 0, width, height);
    g_camera->ResizeWindow(width, height);
	g_config_bar->ChangeTwBarWindowSize(g_screen_width, g_screen_height);

    glutPostRedisplay();
}

void timeout(int value)
{
    glutTimerFunc(g_timestep, timeout, g_timestep);
    // keep track of time
    g_fps_tracker.timestamp();

    // ant tweak bar update
    int atb_feed_back = g_config_bar->Update();
	if (atb_feed_back&ATB_RESHAPE_WINDOW)
	{
		glutReshapeWindow(g_screen_width, g_screen_height);
	}
	if (atb_feed_back&ATB_CHANGE_MATERIAL_PROPERTY)
	{
		g_simulation->SetMaterialProperty();
	}
	if (atb_feed_back&(ATB_CHANGE_TIME_STEP | ATB_CHANGE_INTEGRATION))
	{
		g_simulation->SetReprefactorFlag();
	}
	if (atb_feed_back&(ATB_INIT_MATLAB))
	{
#ifdef ENABLE_MATLAB_DEBUGGING
		g_debugger->Init();
#endif // ENABLE_MATLAB_DEBUGGING
	}

	if (g_recording_limit && g_current_frame > g_total_frame)
	{
		g_pause = true;
	}

	// simulation update
    if (!g_pause) 
    {
		// grab screen
		if (g_record)
		{
			char cap_filename[256];
			sprintf_s(cap_filename, 256, "output/screenshots/ScreenCap%04d.png", g_current_frame);
			grab_screen(cap_filename);

			if (g_export_obj)
			{
				char mesh_filename[256];
				sprintf_s(mesh_filename, 256, "output/mesh/Mesh%04d.obj", g_current_frame);
				g_mesh->ExportToOBJ(mesh_filename);

				char handle_filename[256];
				sprintf_s(handle_filename, 256, "output/handles/Handle%04d.obj", g_current_frame);
				g_simulation->SaveAttachmentConstraint(handle_filename);
			}
		}

		// update scene
		g_scene->Update(g_simulation->Timestep(), g_current_frame);

		// update animation
		g_simulation->AnimateHandle(g_current_frame);
		//g_simulation->UpdateAnimation(g_current_frame);

		//g_global_timer.Tic();
		// update mesh
        g_simulation->Update();
		g_mesh->Update();
		//g_global_timer.Toc();
		//total_time += g_global_timer.Duration();
		//if ((g_current_frame+1)%g_interval == 0)
		//{
		//	std::cout << "Time per Frame = " << total_time/g_interval << " seconds" << std::endl;
		//	total_time = 0;
		//}
		g_current_frame ++;
    }

    glutPostRedisplay();
}

void display() {

    //Always and only do this at the start of a frame, it wipes the slate clean
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    // aim camera
    g_renderer->SetCameraModelview(g_camera->GetViewMatrix());
    g_renderer->SetCameraProjection(g_camera->GetProjectionMatrix());

    // Draw world and cloth (using programmable shaders)
    g_renderer->ActivateShaderprog();
    g_scene->Draw(g_renderer->getVBO());

	if (g_show_mesh)
	{
		g_mesh->Draw(g_renderer->getVBO(), g_show_texture & g_texture_load_succeed);
	}
	if (g_show_wireframe)
	{
		g_mesh->DrawWireFrame(g_renderer->getVBO(), g_wireframe_linewidth);
	}
	g_simulation->Draw(g_renderer->getVBO());

	// highlight selections
	g_selection_tool->HighlightSelectedVertices(g_renderer->getVBO());

	g_renderer->DeactivateShaderprog();

	g_selection_tool->Draw();
	
	if (!g_only_show_sim)
    {
        // Draw axis
        g_camera->DrawAxis();

        // Draw overlay
        draw_overlay();
    }

    // Draw tweak bar
    g_config_bar->Draw();

    glutSwapBuffers();
}

void key_press(unsigned char key, int x, int y) {
    if (!TwEventKeyboardGLUT(key, x, y))
    {
        switch(key) {
        case 32:
            g_pause = !g_pause;
            break;
		case 'q':
		case 'Q':
			g_selection_tool->SetMode(GUI_MODE_SELECTION);
			break;
		case 'w':
		case 'W':
			g_selection_tool->SetMode(GUI_MODE_TRANSLATION);
			break;
		case 'e':
		case 'E':
			break;
		case 'r':
		case 'R':
			g_selection_tool->SetMode(GUI_MODE_ROTATION);
			break;
		case 'b':
		case 'B':
			g_show_mesh = !g_show_mesh;
			break;
		case 'n':
		case 'N':
			g_show_wireframe = !g_show_wireframe;
			break;
		case 't':
		case 'T':
			g_show_texture = !g_show_texture;
			break;
        case 'p':
        case 'P':
            step_through(NULL);
            break;
        case 27: // ascii code of esc key
            cleanup();
            exit(EXIT_SUCCESS);
            break;
        case '0':
            if (g_only_show_sim)
            {
                g_only_show_sim = false;
                g_config_bar->Show();
            }
            else
            {
                g_only_show_sim = true;
                g_config_bar->Hide();
            }
            break;
		case 's':
		case 'S':
			g_camera->SaveCamera();
			g_mesh->ExportToOBJ(DEFAULT_CONFIG_OBJ_FILE);
			g_simulation->SaveAttachmentConstraint(DEFAULT_CONFIG_TARGET_CONSTRAINT_FILE); // save for rendering
			g_simulation->SaveHandles(DEFAULT_CONFIG_HANDLE_FILE);
			g_simulation->SaveHandleAnimation(DEFAULT_CONFIG_ANIMATION_FILE);
			g_simulation->SavePerConstraintMaterialProperties(DEFAULT_CONFIG_MATERIAL_PROPERTIES);
			//g_simulation->SaveLaplacianMatrix(DEFAULT_LAPLACIAN_FILE);
			break;
		case 'l':
			if (g_mesh->ImportFromOBJ(DEFAULT_CONFIG_OBJ_FILE))
			{
				//g_simulation->LoadAttachmentConstraint(DEFAULT_CONFIG_TARGET_CONSTRAINT_FILE);
				g_simulation->LoadHandles(DEFAULT_CONFIG_HANDLE_FILE);
				g_simulation->LoadHandleAnimation(DEFAULT_CONFIG_ANIMATION_FILE);
			}
			g_camera->LoadCamera();
			g_mesh->Update();

			g_simulation->SetVisualizationMesh();
			g_simulation->ResetVisualizationMeshHeight();

			glutPostRedisplay();
			break;
		case 'L':
			if (g_mesh->ImportFromOBJ(DEFAULT_CONFIG_OBJ_FILE))
			{
				//g_simulation->LoadAttachmentConstraint(DEFAULT_CONFIG_TARGET_CONSTRAINT_FILE);
				g_simulation->LoadHandles(DEFAULT_CONFIG_HANDLE_FILE);
				g_simulation->LoadHandleAnimation(DEFAULT_CONFIG_ANIMATION_FILE);
				g_simulation->LoadPerConstraintMaterialProperties(DEFAULT_CONFIG_MATERIAL_PROPERTIES);
			}
			g_camera->LoadCamera();
			g_mesh->Update();

			g_simulation->SetVisualizationMesh();
			g_simulation->ResetVisualizationMeshHeight();

			glutPostRedisplay();
			break;
		case 'g':
		case 'G':
			grab_screen(DEFAULT_SCREEN_SHOT_FILE);
			g_mesh->ExportToOBJ(DEFAULT_OUTPUT_OBJ_FILE);
			g_simulation->SaveAttachmentConstraint(DEFAULT_OUTPUT_ATTACHMENT_OBJ_FILE);
			break;
		case 'a':
		case 'A':
			g_selection_tool->SelectVerticesHardCoded(g_mesh->m_positions);
			break;
        case 'f':
        case 'F':
            g_camera->Lookat(g_mesh);
            break;
		case '9':
			g_simulation->RandomizePoints();
			g_mesh->Update();
			glutPostRedisplay();
			break;
		case '.':
			//g_simulation->RotateHandleToValue();
			g_simulation->SetHandleTranslationAnimation();
			break;
		case '>':
			//g_simulation->RotateHandleToValue();
			g_simulation->SetHandleTranslation();
			break;
		case '/':
			g_simulation->SetHandleRotationAnimation();
			break;
		case '?':
			g_simulation->SetHandleRotation();
			break;
		case 8: // backspace
		case 127: // delete
			g_simulation->DeleteHandle();
			g_simulation->SetReprefactorFlag();
			break;
		}
    }

    glutPostRedisplay();
}

void mouse_click(int button, int state, int x, int y)
{
   // if (!TwEventMouseButtonGLUT(button, state, x, y))
   // {
   //     switch(state)
   //     {
   //     case GLUT_DOWN:
   //         if (glutGetModifiers() == GLUT_ACTIVE_ALT)
   //         {
   //             // left: 0. right: 2. middle: 1.
   //             g_button_mask |= 0x01 << button;
   //             g_mouse_old_x = x;
   //             g_mouse_old_y = y;
   //         }
   //         else if (glutGetModifiers() == GLUT_ACTIVE_CTRL)
   //         {
   //             // ctrl: 3
   //             g_button_mask |= 0x01 << 3;
   //             g_mouse_old_x = x;
   //             g_mouse_old_y = y;
   //         }
 		//	else
			//{
			//	if (g_simulation->TryToToggleAttachmentConstraint(GLM2Eigen(g_camera->GetCameraPosition()), GLM2Eigen(g_camera->GetRaycastDirection(x, y))))
			//	{ // hit something
			//		g_simulation->SetReprefactorFlag();
			//	}
			//}
   //        break;
   //     case GLUT_UP:
   //         if (glutGetModifiers() == GLUT_ACTIVE_CTRL)
   //         {// special case for ctrl
   //             button = 3;
   //         }

   //         g_simulation->UnselectAttachmentConstraint();

   //         unsigned char mask_not = ~g_button_mask;
   //         mask_not |= 0x01 << button;
   //         g_button_mask = ~mask_not;
   //         break;
   //     }
   // }
	if (!TwEventMouseButtonGLUT(button, state, x, y))
	{
		switch (state)
		{
		case GLUT_DOWN:
			// left: 2^0. right: 2^2. middle: 2^1.
			g_button_mask |= 0x01 << button;
			g_mouse_old_x = x;
			g_mouse_old_y = y;
			if (glutGetModifiers() != GLUT_ACTIVE_ALT)
			{
				switch (g_selection_tool->GetMode())
				{
				case GUI_MODE_SELECTION:
					// if selection mode
					// selection box start
					g_selection_tool->SelectFirstPoint(x, y, g_screen_width, g_screen_height, g_button_mask);
					break;
				case GUI_MODE_TRANSLATION:
					g_selection_tool->TranslateRotateFirstPoint();
					g_mouse_down = true;
					break;
				case GUI_MODE_ROTATION:
					g_selection_tool->TranslateRotateFirstPoint();
					g_mouse_down = true;
					break;
				}
			}
			break;
		case GLUT_UP:
			if (glutGetModifiers() != GLUT_ACTIVE_ALT)
			{
				g_selection_tool->SelectSecondPoint(x, y, g_screen_width, g_screen_height, true);
				switch (g_selection_tool->GetMode())
				{
				case GUI_MODE_SELECTION:
					// if selection mode
					// selection box end and selection
					g_selection_tool->SelectVertices(g_mesh->m_positions, g_camera->GetMVP());
					g_simulation->SelectTetConstraints(g_selection_tool->SelectedIndices());
					g_simulation->GetPartialMaterialProperty();
					break;
				case GUI_MODE_TRANSLATION:
					g_simulation->MoveHandleFinalize();
					g_mouse_down = false;
					break;
				case GUI_MODE_ROTATION:
					g_simulation->RotateHandleFinalize();
					g_mouse_down = false;
					break;
				}
			}

			unsigned char mask_not = ~g_button_mask;
			mask_not |= 0x01 << button;
			g_button_mask = ~mask_not;
			break;
		}
	}
}

void mouse_motion(int x, int y)
{
	if (!TwEventMouseMotionGLUT(x, y))
	{
		float dx, dy;
		dx = (float)(x - g_mouse_old_x);
		dy = (float)(y - g_mouse_old_y);

		if (g_button_mask & 0x01)
		{// left button
			// alt + left button
			if (glutGetModifiers() == GLUT_ACTIVE_ALT)
			{
				g_camera->MouseChangeHeadPitch(0.2f, dx, dy);
			}
			else
			{
				g_selection_tool->SelectSecondPoint(x, y, g_screen_width, g_screen_height);
				if (g_selection_tool->GetMode() == GUI_MODE_TRANSLATION)
				{
					g_simulation->MoveHandleTemporary(g_camera->GetCurrentTargetPoint(x, y));
				}
				else if (g_selection_tool->GetMode() == GUI_MODE_ROTATION)
				{
					ScalarType theta;
					glm::vec3 axis;
					g_camera->GetCurrentRotation(x, y, axis, theta);
					g_simulation->RotateHandleTemporary(axis, theta);
				}
			}
		}
		else if (g_button_mask & 0x02)
		{// middle button
			if (glutGetModifiers() == GLUT_ACTIVE_ALT)
			{
				g_camera->MouseChangeLookat(0.01f, dx, dy);
			}
			else
			{
				g_selection_tool->SelectSecondPoint(x, y, g_screen_width, g_screen_height);
				if (g_selection_tool->GetMode() == GUI_MODE_TRANSLATION)
				{
					g_simulation->MoveHandleTemporary(g_camera->GetCurrentTargetPoint(x, y));
				}
				else if (g_selection_tool->GetMode() == GUI_MODE_ROTATION)
				{
					//ScalarType theta;
					//glm::vec3 axis;
					////g_camera->GetCurrentRotation(x, y, axis, theta);
					//g_simulation->RotateHandleTemporary(axis, theta);
				}
			}
		}
		else if (g_button_mask & 0x04)
		{// right button
			if (glutGetModifiers() == GLUT_ACTIVE_ALT)
			{
				g_camera->MouseChangeDistance(0.05f, dx, dy);
			}
			else
			{
				g_selection_tool->SelectSecondPoint(x, y, g_screen_width, g_screen_height);
				if (g_selection_tool->GetMode() == GUI_MODE_TRANSLATION)
				{
					g_simulation->MoveHandleTemporary(g_camera->GetCurrentTargetPoint(x, y));
				}
				else if (g_selection_tool->GetMode() == GUI_MODE_ROTATION)
				{
					ScalarType theta;
					glm::vec3 axis;
					g_camera->GetCurrentRotation(x, y, axis, theta);
					g_simulation->RotateHandleTemporary(axis, theta);
				}
			}
		}
		//else if (g_button_mask & 0x08)
		//{// ctrl + button
		//	g_simulation->MoveSelectedAttachmentConstraintTo(GLM2Eigen(g_camera->GetCurrentTargetPoint(x, y)));
		//}

		g_mouse_old_x = x;
		g_mouse_old_y = y;
	}
}

void mouse_wheel(int button, int dir, int x, int y)
{
    if (!TwMouseWheel(g_mouse_wheel_pos+=dir))
    {
        g_camera->MouseChangeDistance(1.0f, 0, (ScalarType)(dir));
    }
}

void mouse_over(int x, int y)
{
	if (!TwEventMouseMotionGLUT(x, y))
	{
		if (g_button_mask == 0)
		{
			if (g_selection_tool->HoverSelectHandle(g_simulation, g_camera->CastRay(x, y), g_camera->GetMVP()))
			{
				g_camera->CacheLastSelectedPointLocalCoM(g_simulation->SelectedHandleLocalCoM());
				g_camera->CacheLastSelectedPointGlobalCoM(g_simulation->SelectedHandleCoM());
			}
		}
	}
}

void init()
{
    // glew init
    fprintf(stdout, "Initializing glew...\n");
    glewInit();
    if (!glewIsSupported( "GL_VERSION_2_0 " 
        "GL_ARB_pixel_buffer_object"
        )) {
            std::cerr << "ERROR: Support for necessary OpenGL extensions missing." << std::endl;
            exit(EXIT_FAILURE);
    }

    // config init
    fprintf(stdout, "Initializing AntTweakBar...\n");
    g_config_bar = new AntTweakBarWrapper();
    g_config_bar->ChangeTwBarWindowSize(g_screen_width, g_screen_height);

#ifdef ENABLE_MATLAB_DEBUGGING
	// debugger init
	g_debugger = new MatlabDebugger();
#endif // ENABLE_MATLAB_DEBUGGING

    // render wrapper init
    fprintf(stdout, "Initializing render wrapper...\n");
    g_renderer = new RenderWrapper();
    g_renderer->InitShader(DEFAULT_VERT_SHADER_FILE, DEFAULT_FRAG_SHADER_FILE);
    g_texture_load_succeed = g_renderer->InitTexture(DEFAULT_TEXTURE_FILE);

	// selection tool
	g_selection_tool = new SelectionTool();

    // camera init
    fprintf(stdout, "Initializing camera...\n");
    g_camera = new Camera();

    // scene init
    fprintf(stdout, "Initializing scene...\n");
    g_scene = new Scene(DEFAULT_SCENE_FILE);

    // mesh init
    fprintf(stdout, "Initializing mesh...\n");
    g_mesh = new Mesh();

    // simulation init
    fprintf(stdout, "Initializing simulation...\n");
    g_simulation = new Simulation();

	// load or get default value
	g_config_bar->LoadSettings();

	reset_camera(NULL);
    reset_simulation(NULL);
}

void cleanup() // clean up in a reverse order
{
    //if (g_simulation)
    //    delete g_simulation;
    ////if (g_mesh)
    ////    delete g_mesh;
    if (g_scene)
        delete g_scene;
    if (g_camera)
        delete g_camera;
    if (g_renderer)
    {
        g_renderer->CleanupShader();
        delete g_renderer;
    }
	if (g_selection_tool)
	{
		delete g_selection_tool;
	}
#ifdef ENABLE_MATLAB_DEBUGGING
	if (g_debugger)
	{
		delete g_debugger;
	}
#endif // ENABLE_MATLAB_DEBUGGING
    if (g_config_bar)
    {
        delete g_config_bar;
    }
}

void TW_CALL set_handle(void*)
{
	// set handle
	if (!g_selection_tool->SelectedIndices().empty())
	{
		g_simulation->NewHandle(g_selection_tool->SelectedIndices(), g_handle_color);
		// change color
		if (g_random_handle_color)
		{
			glm::vec3 hsv_color;
			hsv_color[0] = rand() / (float)RAND_MAX * 360.0f;
			hsv_color[1] = 0.95f;
			hsv_color[2] = 1.0f;
			g_handle_color = glm::rgbColor(hsv_color);
		}
		g_simulation->SetReprefactorFlag();
	}
	g_selection_tool->Reset();
}
void TW_CALL save_handle(void*)
{
	g_simulation->SaveHandles(DEFAULT_CONFIG_HANDLE_FILE);
}
void TW_CALL load_handle(void*)
{
	g_simulation->LoadHandles(DEFAULT_CONFIG_HANDLE_FILE);
}
void TW_CALL reset_handle(void*)
{
	g_simulation->ResetHandles();
}

void TW_CALL reset_simulation(void*)
{
	// save current setting before reset
	AntTweakBarWrapper::SaveSettings(g_config_bar);

	// reset frame#
	g_current_frame = 0;
	g_pause = true;

	switch(g_mesh->GetMeshType())
    {
	case MESH_TYPE_CLOTH:
		delete g_mesh;
		g_mesh = new ClothMesh();
        break;
	case MESH_TYPE_TET:
		delete g_mesh;
        g_mesh = new TetMesh();
        break;
    }
	g_config_bar->LoadSettings();
    g_mesh->Reset();

    // reset simulation
    g_simulation->SetMesh(g_mesh);
	g_simulation->ResetVisualizationMesh();
	g_simulation->SetVisualizationMesh();
	g_simulation->ResetVisualizationMeshHeight();
	g_simulation->SetScene(g_scene);

    g_simulation->Reset();

	// reset selection
	g_selection_tool->Reset();
	
	// reset config, (config bar is recommended to reset last)
	g_config_bar->Reset();

	// reset scene
	g_scene->Reset();

	// reset matlab debugger related stuff
#ifdef ENABLE_MATLAB_DEBUGGING
	g_debugger->InitVisualizationVector(g_mesh->GetNumberOfVertices());
#endif // ENABLE_MATLAB_DEBUGGING
}

void TW_CALL reset_camera(void*)
{
    // reset camera
	g_camera->Reset(g_screen_width, g_screen_height);
	// reset selection
	g_selection_tool->Reset();
}

void TW_CALL set_partial_material_property(void*)
{
	g_simulation->SetPartialMaterialProperty();
}

void TW_CALL step_through(void*)
{
    if(!g_pause)
    {
        g_pause = true;
    }

	g_scene->Update(g_simulation->Timestep(), g_current_frame);

	g_simulation->AnimateHandle(g_current_frame);

    // enable step mode
	g_simulation->SetStepMode(true);
    // update cloth
    g_simulation->Update();
    // disable step mode
	g_simulation->SetStepMode(false);

	g_mesh->Update();

    g_current_frame++;
}

void grab_screen(void)
{
    char anim_filename[256];
    sprintf_s(anim_filename, 256, "output/Simulation%04d.png", g_current_frame);
	grab_screen(anim_filename);
}

void grab_screen(char* filename)
{
	unsigned char* bitmapData = new unsigned char[3 * g_screen_width * g_screen_height];

    for (int i=0; i < g_screen_height; i++) 
    {
        glReadPixels(0, i, g_screen_width, 1, GL_RGB, GL_UNSIGNED_BYTE, 
            bitmapData + (g_screen_width * 3 * ((g_screen_height - 1) - i)));
    }

    stbi_write_png(filename, g_screen_width, g_screen_height, 3, bitmapData, g_screen_width * 3);

    delete [] bitmapData;
}

void draw_overlay()
{
    // Draw Overlay
    glColor4d(0.0, 0.0, 0.0, 1.0);
    glPushAttrib(GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0, 0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRasterPos2d(0.03, 0.01);

    char overlay_char_from_simulation[255] = ""; 
    g_simulation->GetOverlayChar(overlay_char_from_simulation, 255);

    char info[1024];
	sprintf_s(info, "FPS: %3.1f | Frame#: %d%s", g_fps_tracker.fpsAverage(), g_current_frame, overlay_char_from_simulation);

    for (unsigned int i = 0; i < strlen(info); i++)
    {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, info[i]);
    }

    glPopAttrib();
}

// matlab calls
void TW_CALL matlab_reset_current_data(void*)
{
#ifdef ENABLE_MATLAB_DEBUGGING
	g_debugger->ResetCurrent();
#endif
}
void TW_CALL matlab_reset_all_data(void*)
{
#ifdef ENABLE_MATLAB_DEBUGGING
	g_debugger->Reset();
#endif
}
void TW_CALL matlab_set_converged_energy(void*)
{
#ifdef ENABLE_MATLAB_DEBUGGING
	g_simulation->SetConvergedEnergy();
#endif
}
void TW_CALL matlab_new_data(void*)
{
#ifdef ENABLE_MATLAB_DEBUGGING
	g_debugger->AddNewPlotData();
	g_debugger->ResetCurrent();
#endif
}
void TW_CALL matlab_remove_last_data(void*)
{
#ifdef  ENABLE_MATLAB_DEBUGGING
	g_debugger->RemoveLastData();
#endif //  ENABLE_MATLAB_DEBUGGING

}
void TW_CALL matlab_export_data(void*)
{
#ifdef ENABLE_MATLAB_DEBUGGING
	g_debugger->Export();
#endif
}
void TW_CALL matlab_import_data(void*)
{
#ifdef ENABLE_MATLAB_DEBUGGING
	g_debugger->Import();
#endif
}

void TW_CALL matlab_plot(void*)
{
#ifdef ENABLE_MATLAB_DEBUGGING
	g_debugger->Plot();
#endif
}

void TW_CALL matlab_plot_all(void*)
{
#ifdef ENABLE_MATLAB_DEBUGGING
	g_debugger->PlotAll();
#endif
}

void TW_CALL matlab_visualize_vector(void*)
{
#ifdef ENABLE_MATLAB_DEBUGGING
	g_debugger->SetVisualizationVariableName();
#endif
}
