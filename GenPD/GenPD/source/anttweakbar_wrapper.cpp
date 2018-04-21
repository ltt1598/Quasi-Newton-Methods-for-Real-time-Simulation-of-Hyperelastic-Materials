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

#pragma warning( disable : 4305 4996) 

#include "anttweakbar_wrapper.h"
#include "mesh.h"
#include "simulation.h"
#include "selection_tool.h"

//----------Events Related Variables--------------------//
static int g_old_screen_width;
static int g_old_screen_height;
static ScalarType g_old_timestep;
static unsigned int g_old_substepping;
static ScalarType g_old_stretch_stiffness;
static ScalarType g_old_bending_stiffness;
static ScalarType g_old_attachment_stiffness;
static ScalarType g_old_kappa;
static ScalarType g_old_laplacian_stiffness;
static MaterialType g_old_material_type;
static bool g_old_enable_matlab_debugger;
static IntegrationMethod g_old_integration_method;

//----------Global Parameters----------------//
extern int g_screen_width;
extern int g_screen_height;
extern glm::vec3 g_handle_color;
extern bool g_random_handle_color;

//----------State Control--------------------//
extern bool g_only_show_sim;
extern bool g_record;
extern bool g_pause;
extern bool g_show_mesh;
extern bool g_show_wireframe;
extern int  g_wireframe_linewidth;
extern bool g_show_texture;

//----------Recording Related----------------//
extern bool g_recording_limit;
extern int g_current_frame;
extern int g_total_frame;
extern bool g_export_obj;

//----------anttweakbar handlers----------//
extern void TW_CALL set_handle(void*);
extern void TW_CALL save_handle(void*);
extern void TW_CALL load_handle(void*);
extern void TW_CALL reset_handle(void*);
extern void TW_CALL reset_simulation(void*);
extern void TW_CALL step_through(void*);
extern void TW_CALL reset_camera(void*);
extern void TW_CALL set_partial_material_property(void*);
extern void TW_CALL matlab_reset_current_data(void*);
extern void TW_CALL matlab_reset_all_data(void*);
extern void TW_CALL matlab_set_converged_energy(void*);
extern void TW_CALL matlab_new_data(void*);
extern void TW_CALL matlab_remove_last_data(void*);
extern void TW_CALL matlab_export_data(void*);
extern void TW_CALL matlab_import_data(void*);
extern void TW_CALL matlab_plot(void*);
extern void TW_CALL matlab_plot_all(void*);
extern void TW_CALL matlab_visualize_vector(void*);

//----------key components--------------//
extern SelectionTool* g_selection_tool;
extern Mesh* g_mesh;
extern Simulation* g_simulation;
#ifdef ENABLE_MATLAB_DEBUGGING
#include "matlab_debugger.h"
extern MatlabDebugger* g_debugger;
#endif // ENABLE_MATLAB_DEBUGGING

AntTweakBarWrapper::AntTweakBarWrapper()
{
	
}

AntTweakBarWrapper::~AntTweakBarWrapper()
{
	SaveSettings(NULL);
	Cleanup();
}

void AntTweakBarWrapper::Init()
{
	TwInit(TW_OPENGL, NULL);

	//Control Panel bar
	m_control_panel_bar = TwNewBar("Control Panel");
	TwDefine(" 'Control Panel' size='200 900' position='814 10' color='255 255 255' text=dark ");
	char control_bar_pos_string [255];
	sprintf(control_bar_pos_string, "'Control Panel' position='%d 10'", g_screen_width-210);
	TwDefine(control_bar_pos_string);
	// state control
	TwAddVarRW(m_control_panel_bar, "Pause", TwType(sizeof(bool)), &(g_pause), "group='State Control'");
	TwAddButton(m_control_panel_bar, "Step Once", step_through, NULL, "group='State Control' ");
	TwAddVarRW(m_control_panel_bar, "Record", TwType(sizeof(bool)), &(g_record), "group='State Control'");
	TwAddVarRW(m_control_panel_bar, "Has End", TwType(sizeof(bool)), &(g_recording_limit), "group='Recording'");
	TwAddVarRW(m_control_panel_bar, "Total Frames", TW_TYPE_INT32, &(g_total_frame), "group='Recording'");
	TwAddVarRW(m_control_panel_bar, "Export OBJ", TwType(sizeof(bool)), &(g_export_obj), "group='Recording'");
	TwDefine(" 'Control Panel'/'Recording' group='State Control'");
	TwAddSeparator(m_control_panel_bar, NULL, "");
	// visualization
	TwAddVarRW(m_control_panel_bar, "Mesh Body", TwType(sizeof(bool)), &(g_show_mesh), "group='Visualization'");
	TwAddVarRW(m_control_panel_bar, "Wireframe", TwType(sizeof(bool)), &(g_show_wireframe), "group='Visualization'");
	TwAddVarRW(m_control_panel_bar, "Line Width", TW_TYPE_INT32, &(g_wireframe_linewidth), "min=1 group='Visualization'");
	TwAddVarRW(m_control_panel_bar, "Texture", TwType(sizeof(bool)), &(g_show_texture), "group='Visualization'");
	TwAddVarRW(m_control_panel_bar, "Width", TW_TYPE_INT32, &(g_screen_width), "min=640 group='Screen Resolution'");
	TwAddVarRW(m_control_panel_bar, "Height", TW_TYPE_INT32, &(g_screen_height), "min=480 group='Screen Resolution'");
	TwAddSeparator(m_control_panel_bar, NULL, "");
	// matlab
#ifdef ENABLE_MATLAB_DEBUGGING
	TwAddVarRW(m_control_panel_bar, "Matlab Debugger", TwType(sizeof(bool)), &(g_debugger->m_enable), "group = 'Matlab'");
	TwAddVarRW(m_control_panel_bar, "Variable Name", TW_TYPE_CSSTRING(sizeof(g_debugger->m_vis_name)), &(g_debugger->m_vis_name), " group='Matlab Visualization' ");
	TwAddVarRW(m_control_panel_bar, "Current Variable", TW_TYPE_CSSTRING(sizeof(g_debugger->m_vis_name_current)), &(g_debugger->m_vis_name_current), " group='Matlab Visualization' readonly=true ");
	TwAddButton(m_control_panel_bar, "Visualize", matlab_visualize_vector, (void*)(this), " group='Matlab Visualization' ");
	TwAddVarRW(m_control_panel_bar, "Amplify", TW_TYPE_SCALAR_TYPE, &g_debugger->m_vis_amplify, "group='Matlab Visualization'");
	TwDefine(" 'Control Panel'/'Matlab Visualization' group='Matlab'");
	TwEnumVal axisStyleEV[PLOT_TYPE_TOTAL_NUM] = { { PLOT_TYPE_PLOT, "plot" },
								{PLOT_TYPE_SEMILOG_X, "semilogx"},
								{PLOT_TYPE_SEMILOG_Y, "semilogy"},
								{PLOT_TYPE_LOGLOG, "loglog"}};
	TwType axisStyle = TwDefineEnum("AStyle", axisStyleEV, PLOT_TYPE_TOTAL_NUM);
	TwAddVarRW(m_control_panel_bar, "Axis Style", axisStyle, &g_debugger->m_plot_type, " group='Matlab Plot' ");
	TwEnumVal plotDataEV[PLOT_DATA_TYPE_TOTAL_NUM] = {  { PLOT_DATA_TYPE_DISTANCE_TO_SOLUTION, "distance to solution" },
														{ PLOT_DATA_TYPE_ENERGY, "energy" },
														{ PLOT_DATA_TYPE_GRADIENT_NORM, "||gradient||" }};
	TwType plotData = TwDefineEnum("PData", plotDataEV, PLOT_DATA_TYPE_TOTAL_NUM);
	TwAddVarRW(m_control_panel_bar, "Plot Data", plotData, &g_debugger->m_plot_data_type, " group='Matlab Plot' ");
	TwAddVarRW(m_control_panel_bar, "x-Axis Variable", TwType(sizeof(bool)), &g_debugger->m_plot_x_axis_plot_iterations, "true='Iteration' false='Time' group='Matlab Plot'");
	TwAddButton(m_control_panel_bar, "Plot", matlab_plot, (void*)(this), " group='Matlab Plot' ");
	TwAddButton(m_control_panel_bar, "Plot All", matlab_plot_all, (void*)(this), " group='Matlab Plot' ");
	TwDefine(" 'Control Panel'/'Matlab Plot' group='Matlab'");
	TwAddButton(m_control_panel_bar, "Reset Current Data", matlab_reset_current_data, (void*)(this), " group='Matlab Data' ");
	TwAddButton(m_control_panel_bar, "Reset All Data", matlab_reset_all_data, (void*)(this), " group='Matlab Data' ");
	TwAddButton(m_control_panel_bar, "Set Converged Energy", matlab_set_converged_energy, (void*)(this), " group='Matlab Data' ");
	TwAddVarRW(m_control_panel_bar, "Legend", TW_TYPE_CSSTRING(sizeof(g_debugger->m_legend_string)), &(g_debugger->m_legend_string), " group='Matlab Data' ");
	TwAddButton(m_control_panel_bar, "Add This Data", matlab_new_data, (void*)(this), " group='Matlab Data' ");
	TwAddButton(m_control_panel_bar, "Remove Last Data", matlab_remove_last_data, (void*)(this), " group='Matlab Data' ");
	TwAddButton(m_control_panel_bar, "Export Data", matlab_export_data, (void*)(this), " group='Matlab Data' ");
	TwAddButton(m_control_panel_bar, "Import Data", matlab_import_data, (void*)(this), " group='Matlab Data' ");
	TwDefine(" 'Control Panel'/'Matlab Data' group='Matlab'");
	TwAddSeparator(m_control_panel_bar, NULL, "");
#endif
	// verbose
	TwAddVarRW(m_control_panel_bar, "Converge", TwType(sizeof(bool)), &(g_simulation->m_verbose_show_converge), "group='Verbose'");
	TwAddVarRW(m_control_panel_bar, "Optimization Time", TwType(sizeof(bool)), &(g_simulation->m_verbose_show_optimization_time), "group='Verbose'");
	TwAddVarRW(m_control_panel_bar, "Energy", TwType(sizeof(bool)), &(g_simulation->m_verbose_show_energy), "group='Verbose'");
	TwAddVarRW(m_control_panel_bar, "Factorization Warnings", TwType(sizeof(bool)), &(g_simulation->m_verbose_show_factorization_warning), "group='Verbose'");
	TwAddSeparator(m_control_panel_bar, NULL, "");
	// buttons
	TwAddButton(m_control_panel_bar, "Save Settings", SaveSettings, this, " ");
	TwAddButton(m_control_panel_bar, "Load Settings", LoadSettings, this, " ");
	TwAddButton(m_control_panel_bar, "Default Settings", SetDefaultSettings, this, " ");
	TwAddSeparator(m_control_panel_bar, NULL, "");
	TwAddButton(m_control_panel_bar, "Reset Camera", reset_camera, NULL, " ");
	TwAddButton(m_control_panel_bar, "Reset Simulation", reset_simulation, NULL, " ");
	//!Control Panel bar

	// mesh settings bar
	m_mesh_bar = TwNewBar("Mesh Settings");
	TwDefine(" 'Mesh Settings' size='200 250' position='10 10' color='210 240 255' text=dark ");
	// mesh type
	TwEnumVal meshTypeStyleEV[2] =  {{MESH_TYPE_CLOTH, "Cloth"},
									 {MESH_TYPE_TET, "Tet Mesh"}};
	TwType meshTypeStyle = TwDefineEnum("MeshType", meshTypeStyleEV, 2);
	TwAddVarRW(m_mesh_bar, "Mesh Type", meshTypeStyle, &g_mesh->m_mesh_type, " ");
	TwAddVarRW(m_mesh_bar, "Total Mass", TW_TYPE_SCALAR_TYPE, &(g_mesh->m_total_mass), " ");
	// tet settings
	TwAddVarRW(m_mesh_bar, "Tet File", TW_TYPE_CSSTRING(sizeof(g_mesh->m_tet_file_path)), &(g_mesh->m_tet_file_path), " group='Tet Settings' ");
	TwAddVarRW(m_mesh_bar, "Tet Scaling", TW_TYPE_SCALAR_TYPE, &(g_mesh->m_tet_scaling), " min=0.01 group='Tet Settings' ");
	TwAddVarRW(m_mesh_bar, "Tet Flip", TwType(sizeof(bool)), &(g_mesh->m_tet_flip), " group='Tet Settings' ");
	TwAddVarRW(m_mesh_bar, "Tet Rotation", TW_TYPE_SCALAR_TYPE, &(g_mesh->m_tet_rotation), "group='Tet Settings'");
	// cloth settings
	// cloth dimensions
	TwAddVarRW(m_mesh_bar, "dim1", TW_TYPE_INT32, &(g_mesh->m_dim[0]), " label='Width' min=2 group='Cloth Dimension' ");
	TwAddVarRW(m_mesh_bar, "dim2", TW_TYPE_INT32, &(g_mesh->m_dim[1]), " label='Length' min=2 group='Cloth Dimension' ");
	TwDefine(" 'Mesh Settings'/'Cloth Dimension' group='Cloth Settings'");
	// cloth corner position1
	TwAddVarRW(m_mesh_bar, "c1x", TW_TYPE_SCALAR_TYPE, &(g_mesh->m_corners[0][0]), " label='X' group='Corner1 Position' ");
	TwAddVarRW(m_mesh_bar, "c1y", TW_TYPE_SCALAR_TYPE, &(g_mesh->m_corners[0][1]), " label='Y' group='Corner1 Position' ");
	TwAddVarRW(m_mesh_bar, "c1z", TW_TYPE_SCALAR_TYPE, &(g_mesh->m_corners[0][2]), " label='Z' group='Corner1 Position' ");
	TwDefine(" 'Mesh Settings'/'Corner1 Position' group='Cloth Settings'");
	//TwDefine(" 'Mesh Settings'/'Corner1 Position' opened=false ");
	// cloth corner position2
	TwAddVarRW(m_mesh_bar, "c2x", TW_TYPE_SCALAR_TYPE, &(g_mesh->m_corners[1][0]), " label='X' group='Corner2 Position' ");
	TwAddVarRW(m_mesh_bar, "c2y", TW_TYPE_SCALAR_TYPE, &(g_mesh->m_corners[1][1]), " label='Y' group='Corner2 Position' ");
	TwAddVarRW(m_mesh_bar, "c2z", TW_TYPE_SCALAR_TYPE, &(g_mesh->m_corners[1][2]), " label='Z' group='Corner2 Position' ");
	TwDefine(" 'Mesh Settings'/'Corner2 Position' group='Cloth Settings'");
	// selection
	TwEnumVal mouseModeEV[3] = { { GUI_MODE_SELECTION, "Selection (Q)" },
							 	 { GUI_MODE_TRANSLATION, "Translation (W)" },
								 { GUI_MODE_ROTATION, "Rotation (R)" } };
	TwType mouseMode = TwDefineEnum("MouseMode", mouseModeEV, 3);
	TwAddVarRW(m_mesh_bar, "Mouse Mode", mouseMode, &g_selection_tool->m_gui_mode, " group='Handle' ");
	TwAddVarRW(m_mesh_bar, "Random Handle Color", TwType(sizeof(bool)), &(g_random_handle_color), " group='Handle' ");
	TwAddVarRW(m_mesh_bar, "Handle Color", TW_TYPE_COLOR3F, &(g_handle_color[0]), " group='Handle' colormode=hls ");
	TwAddButton(m_mesh_bar, "Set Handle (H)", set_handle, NULL, " group='Handle' key='h' ");
	TwAddButton(m_mesh_bar, "Save Handle", save_handle, NULL, " group='Handle' ");
	TwAddButton(m_mesh_bar, "Load Handle", load_handle, NULL, " group='Handle' ");
	TwAddButton(m_mesh_bar, "Reset Handle", reset_handle, NULL, " group='Handle' ");
	//TwDefine(" 'Mesh Settings'/'Corner2 Position' opened=false ");
	// !mesh settings bar

	// simulation settings bar
	m_sim_bar = TwNewBar("Simulation Settings");
	TwDefine(" 'Simulation Settings' size='200 600' position='10 270' color='255 216 224' text=dark ");
	// integration
	TwAddVarRW(m_sim_bar, "Time Step", TW_TYPE_SCALAR_TYPE, &g_simulation->m_h, " min=0.0000001");
	TwAddVarRW(m_sim_bar, "Sub Stepping", TW_TYPE_INT32, &g_simulation->m_sub_stepping, "min = 1");
	// integration method
	TwEnumVal integrationStyleEV[INTEGRATION_TOTAL_NUM] = { \
															{INTEGRATION_QUASI_STATICS, "Quasi Statics"},\
															{INTEGRATION_IMPLICIT_EULER, "Implicit Euler"}, \
															{INTEGRATION_IMPLICIT_BDF2, "Implicit BDF2"},\
															{INTEGRATION_IMPLICIT_MIDPOINT, "Implicit Midpoint"},\
															{INTEGRATION_IMPLICIT_NEWMARK_BETA, "Newmark Beta"}\
														  };
	TwType integrationStyle = TwDefineEnum("Integration Method", integrationStyleEV, INTEGRATION_TOTAL_NUM);
	TwAddVarRW(m_sim_bar, "Method", integrationStyle, &g_simulation->m_integration_method, " group='Integration' ");
	// optimization method
	TwEnumVal optimizationStyleEV[OPTIMIZATION_METHOD_TOTAL_NUM] = { \
																	{OPTIMIZATION_METHOD_GRADIENT_DESCENT, "Gradient Descent"},\
																	{OPTIMIZATION_METHOD_NEWTON, "Newton's Method"},\
																	{OPTIMIZATION_METHOD_LBFGS, "L-BFGS (Our Method)"}
																  };
	TwType optimizationStyle = TwDefineEnum("OptimizationMethod", optimizationStyleEV, OPTIMIZATION_METHOD_TOTAL_NUM);
	TwAddVarRW(m_sim_bar, "Optimization Method", optimizationStyle, &g_simulation->m_optimization_method, " group='Optimization' ");
	TwAddVarRW(m_sim_bar, "Iterations/Frame", TW_TYPE_INT32, &g_simulation->m_iterations_per_frame, " group='Optimization' ");
	//TwAddVarRW(m_sim_bar, "Jacobi iterations", TW_TYPE_INT32, &g_simulation->m_jacobi_iterations, "group='Optimization'");
	TwAddVarRW(m_sim_bar, "Definiteness Fix", TwType(sizeof(bool)), &g_simulation->m_definiteness_fix, "group='Optimization'");
	TwAddVarRW(m_sim_bar, "Parallelize", TwType(sizeof(bool)), &g_simulation->m_enable_openmp, " group='Optimization' ");
	// Linear Solver
	TwEnumVal solverTypeEV[SOLVER_TYPE_TOTAL_NUM] = { \
													{SOLVER_TYPE_DIRECT_LLT, "Direct LLT"},\
													{SOLVER_TYPE_CG, "CG"}
													};
	TwType solverType = TwDefineEnum("SolverType", solverTypeEV, SOLVER_TYPE_TOTAL_NUM);
	TwAddVarRW(m_sim_bar, "Solver Type", solverType, &g_simulation->m_solver_type, " group='Linear Solver' ");
	TwAddVarRW(m_sim_bar, "Max Iteration", TW_TYPE_INT32, &g_simulation->m_iterative_solver_max_iteration, " group='Linear Solver' ");
	TwDefine(" 'Simulation Settings'/'Linear Solver' group='Optimization'");
	// lbfgs
	TwAddVarRW(m_sim_bar, "Restart Every Frame", TwType(sizeof(bool)), &g_simulation->m_lbfgs_restart_every_frame, "group='l-BFGS'");
	TwEnumVal lbfgsH0TypeEV[LBFGS_H0_TOTAL_NUM] =  {{LBFGS_H0_IDENTITY, "Scaled Identity"},
													{ LBFGS_H0_LAPLACIAN, "Weighted Laplacian" }};
	TwType lbfgsH0Type = TwDefineEnum("l-BFGS H0", lbfgsH0TypeEV, LBFGS_H0_TOTAL_NUM);
	TwAddVarRW(m_sim_bar, "l-BFGS H0 Type", lbfgsH0Type, &g_simulation->m_lbfgs_H0_type, " group='l-BFGS' ");
	TwAddVarRW(m_sim_bar, "History Length", TW_TYPE_INT32, &g_simulation->m_lbfgs_m, "group='l-BFGS'");
	TwDefine(" 'Simulation Settings'/'l-BFGS' group='Optimization'");
	// laplacian
	TwAddVarRW(m_sim_bar, "Auto laplacian coefficient", TwType(sizeof(bool)), &g_simulation->m_stiffness_auto_laplacian_stiffness, "group='Laplacian'");
	TwAddVarRW(m_sim_bar, "Laplacian coefficient", TW_TYPE_SCALAR_TYPE, &g_simulation->m_stiffness_laplacian, " group='Laplacian' ");
	TwDefine(" 'Simulation Settings'/'Laplacian' group='l-BFGS'");
	// line search
	TwAddVarRW(m_sim_bar, "Enable", TwType(sizeof(bool)), &g_simulation->m_enable_line_search, " group='Line Search' ");
	TwEnumVal linesearchTypeEV[LS_TYPE_TOTAL_NUM] = { { LS_TYPE_ARMIJO, "Armijo" },
													{ LS_TYPE_WOLFE, "Wolfe" }};
	TwType linsearchType = TwDefineEnum("LS Type", linesearchTypeEV, LS_TYPE_TOTAL_NUM);
	TwAddVarRW(m_sim_bar, "Line Search Type", linsearchType, &g_simulation->m_ls_type, " group='Line Search' ");
	TwAddVarRW(m_sim_bar, "Step Size", TW_TYPE_SCALAR_TYPE, &g_simulation->m_ls_step_size, " min=0 group='Line Search' ");
	TwAddVarRW(m_sim_bar, "Alpha", TW_TYPE_SCALAR_TYPE, &g_simulation->m_ls_alpha, " min=0 max=0.5 group='Line Search' ");
	TwAddVarRW(m_sim_bar, "Beta", TW_TYPE_SCALAR_TYPE, &g_simulation->m_ls_beta, " min=0 max=1 group='Line Search' ");
	TwDefine(" 'Simulation Settings'/'Line Search' group='Optimization'");

	TwDefine(" 'Simulation Settings'/'Optimization' group='Integration'");

	// Material Type
	TwEnumVal materialTypeEV[MATERIAL_TYPE_TOTAL_NUM] = { \
														{MATERIAL_TYPE_COROT, "Corot"}, \
														{MATERIAL_TYPE_StVK, "StVK"}, \
														{MATERIAL_TYPE_NEOHOOKEAN_EXTEND_LOG, "NH(Clamped Log)"}, \
														};
	TwType materialType = TwDefineEnum("Material Type", materialTypeEV, MATERIAL_TYPE_TOTAL_NUM);
	TwAddVarRW(m_sim_bar, "Tet Constraint Type", materialType, &g_simulation->m_material_type, " group='Material' ");
	// constants
	TwAddVarRW(m_sim_bar, "Attachment Stiffness", TW_TYPE_SCALAR_TYPE, &g_simulation->m_stiffness_attachment, " group='Material' ");
	TwAddVarRW(m_sim_bar, "Stretch Stiffness", TW_TYPE_SCALAR_TYPE, &g_simulation->m_stiffness_stretch, " group='Material' ");
	TwAddVarRW(m_sim_bar, "Bending Stiffness", TW_TYPE_SCALAR_TYPE, &g_simulation->m_stiffness_bending, " group='Material' ");
	TwAddVarRW(m_sim_bar, "mu", TW_TYPE_SCALAR_TYPE, &g_simulation->m_stiffness_stretch, " group='Material' ");
	TwAddVarRW(m_sim_bar, "lambda", TW_TYPE_SCALAR_TYPE, &g_simulation->m_stiffness_bending, " group='Material' ");
	TwAddVarRW(m_sim_bar, "kappa", TW_TYPE_SCALAR_TYPE, &g_simulation->m_stiffness_kappa, " group='Material' ");
	// partial material
	TwAddVarRW(m_sim_bar, "Selected Tet Constraint Type", materialType, &g_simulation->m_partial_material_type, " group='Selected Material' ");
	TwAddVarRW(m_sim_bar, "Selected Stretch Stiffness", TW_TYPE_SCALAR_TYPE, &g_simulation->m_partial_stiffness_stretch, " group='Selected Material' ");
	TwAddVarRW(m_sim_bar, "Selected Bending Stiffness", TW_TYPE_SCALAR_TYPE, &g_simulation->m_partial_stiffness_bending, " group='Selected Material' ");
	TwAddVarRW(m_sim_bar, "Selected mu", TW_TYPE_SCALAR_TYPE, &g_simulation->m_partial_stiffness_stretch, " group='Selected Material' ");
	TwAddVarRW(m_sim_bar, "Selected lambda", TW_TYPE_SCALAR_TYPE, &g_simulation->m_partial_stiffness_bending, " group='Selected Material' ");
	TwAddVarRW(m_sim_bar, "Selected kappa", TW_TYPE_SCALAR_TYPE, &g_simulation->m_partial_stiffness_kappa, " group='Selected Material' ");
	TwAddButton(m_sim_bar, "Set Selection", set_partial_material_property, NULL, " group='Selected Material' ");
	TwDefine(" 'Simulation Settings'/'Selected Material' group='Material'");

	TwAddVarRW(m_sim_bar, "Gravity", TW_TYPE_SCALAR_TYPE, &g_simulation->m_gravity_constant, " group='Constants' ");
	TwAddVarRW(m_sim_bar, "Damping Coefficient", TW_TYPE_SCALAR_TYPE, &g_simulation->m_damping_coefficient, " min=0 group='Constants' ");
	TwAddVarRW(m_sim_bar, "Restitution Coefficient", TW_TYPE_SCALAR_TYPE, &g_simulation->m_restitution_coefficient, " min=0 group='Constants' ");
	TwAddVarRW(m_sim_bar, "Friction Coefficient", TW_TYPE_SCALAR_TYPE, &g_simulation->m_friction_coefficient, " min=0 group='Constants' ");
	// Demo
	TwAddVarRW(m_sim_bar, "Process Collision", TwType(sizeof(bool)), &g_simulation->m_processing_collision, " group='Demo' ");
	// !simulation settings bar

	TwDefine(" TW_HELP visible=false ");
}

void AntTweakBarWrapper::Cleanup()
{
	m_control_panel_bar = NULL;
	m_mesh_bar = NULL;
	m_sim_bar = NULL;

	TwTerminate();
}

void AntTweakBarWrapper::Reset()
{
	Cleanup(); 
	Init();
}

void AntTweakBarWrapper::Hide()
{
	TwDefine(" 'Control Panel' visible=false ");
	TwDefine(" 'Mesh Settings' visible=false ");
	TwDefine(" 'Simulation Settings' visible=false ");
}

void AntTweakBarWrapper::Show()
{
	TwDefine(" 'Control Panel' visible=true ");
	TwDefine(" 'Mesh Settings' visible=true ");
	TwDefine(" 'Simulation Settings' visible=true ");
}

int AntTweakBarWrapper::Update()
{
	// update

	// control panel pos
	char control_bar_pos_string [255];
	sprintf(control_bar_pos_string, "'Control Panel' position='%d 10'", g_screen_width-210);
	TwDefine(control_bar_pos_string);

	// control panel settings
	if (g_record)
	{
		TwDefine(" 'Control Panel'/'Recording' visible=true");
	}
	else
	{
		g_recording_limit = false;
		TwDefine(" 'Control Panel'/'Recording' visible=false");
	}
#ifdef ENABLE_MATLAB_DEBUGGING
	// matlab settings display
	if (g_debugger->m_enable)
	{
		TwDefine(" 'Control Panel'/'Matlab Visualization' visible=true");
		TwDefine(" 'Control Panel'/'Matlab Plot' visible=true");
		TwDefine(" 'Control Panel'/'Matlab Data' visible=true");
	}
	else
	{
		TwDefine(" 'Control Panel'/'Matlab Visualization' visible=false");
		TwDefine(" 'Control Panel'/'Matlab Plot' visible=false");
		TwDefine(" 'Control Panel'/'Matlab Data' visible=false");
	}
#endif // ENABLE_MATLAB_DEBUGGING

	// mesh settings display
	switch (g_mesh->m_mesh_type)
	{
	case MESH_TYPE_TET:
		TwDefine(" 'Mesh Settings'/'Tet Settings' visible=true");
		TwDefine(" 'Mesh Settings'/'Cloth Settings' visible=false");

		TwDefine(" 'Simulation Settings'/'Tet Constraint Type' visible=true");
		TwDefine(" 'Simulation Settings'/'Stretch Stiffness' visible=false");
		TwDefine(" 'Simulation Settings'/'Bending Stiffness' visible=false");
		TwDefine(" 'Simulation Settings'/'mu' visible=true");
		TwDefine(" 'Simulation Settings'/'lambda' visible=true");
		TwDefine(" 'Simulation Settings'/'kappa' visible=true");
		TwDefine(" 'Simulation Settings'/'Laplacian' visible=true");

		TwDefine(" 'Simulation Settings'/'Selected Tet Constraint Type' visible=true");
		TwDefine(" 'Simulation Settings'/'Selected Stretch Stiffness' visible=false");
		TwDefine(" 'Simulation Settings'/'Selected Bending Stiffness' visible=false");
		TwDefine(" 'Simulation Settings'/'Selected mu' visible=true");
		TwDefine(" 'Simulation Settings'/'Selected lambda' visible=true");
		TwDefine(" 'Simulation Settings'/'Selected kappa' visible=true");
		break;
	case MESH_TYPE_CLOTH:
		TwDefine(" 'Mesh Settings'/'Tet Settings' visible=false");
		TwDefine(" 'Mesh Settings'/'Cloth Settings' visible=true");

		TwDefine(" 'Simulation Settings'/'Tet Constraint Type' visible=false");
		TwDefine(" 'Simulation Settings'/'Stretch Stiffness' visible=true");
		TwDefine(" 'Simulation Settings'/'Bending Stiffness' visible=true");
		TwDefine(" 'Simulation Settings'/'mu' visible=false");
		TwDefine(" 'Simulation Settings'/'lambda' visible=false");
		TwDefine(" 'Simulation Settings'/'kappa' visible=false");
		TwDefine(" 'Simulation Settings'/'Laplacian' visible=false");

		TwDefine(" 'Simulation Settings'/'Selected Tet Constraint Type' visible=false");
		TwDefine(" 'Simulation Settings'/'Selected Stretch Stiffness' visible=true");
		TwDefine(" 'Simulation Settings'/'Selected Bending Stiffness' visible=true");
		TwDefine(" 'Simulation Settings'/'Selected mu' visible=false");
		TwDefine(" 'Simulation Settings'/'Selected lambda' visible=false");
		TwDefine(" 'Simulation Settings'/'Selected kappa' visible=false");
		break;
	}
	if (g_simulation->m_stiffness_auto_laplacian_stiffness)
	{
		TwDefine(" 'Simulation Settings'/'Laplacian coefficient' readonly=true");
	}
	else
	{
		TwDefine(" 'Simulation Settings'/'Laplacian coefficient' readonly=false");
	}
	if (g_simulation->m_selected_constraints.empty())
	{
		TwDefine(" 'Simulation Settings'/'Selected Material' visible=false");
	}
	else
	{
		TwDefine(" 'Simulation Settings'/'Selected Material' visible=true");
	}

	switch (g_simulation->m_integration_method)
	{
	case INTEGRATION_QUASI_STATICS:
	case INTEGRATION_IMPLICIT_EULER:
	case INTEGRATION_IMPLICIT_BDF2:
	case INTEGRATION_IMPLICIT_MIDPOINT:
	case INTEGRATION_IMPLICIT_NEWMARK_BETA:
		TwDefine(" 'Simulation Settings'/'Optimization' visible=true");
		break;
	default:
		break;
	}
	switch (g_simulation->m_optimization_method)
	{
	case OPTIMIZATION_METHOD_LBFGS:
		TwDefine(" 'Simulation Settings'/'l-BFGS' visible=true");
		TwDefine(" 'Simulation Settings'/'Definiteness Fix' visible=false");
		TwDefine(" 'Simulation Settings'/'Linear Solver' visible=true");
		if (g_simulation->m_lbfgs_H0_type == LBFGS_H0_LAPLACIAN)
		{
			TwDefine(" 'Simulation Settings'/'Laplacian' visible=true");
		}
		else
		{
			TwDefine(" 'Simulation Settings'/'Laplacian' visible=false");
		}
		break;
	case OPTIMIZATION_METHOD_NEWTON:
		TwDefine(" 'Simulation Settings'/'l-BFGS' visible=false");
		TwDefine(" 'Simulation Settings'/'Definiteness Fix' visible=true");
		TwDefine(" 'Simulation Settings'/'Linear Solver' visible=true");
		break;
	default:
		TwDefine(" 'Simulation Settings'/'l-BFGS' visible=false");
		TwDefine(" 'Simulation Settings'/'Definiteness Fix' visible=false");
		TwDefine(" 'Simulation Settings'/'Linear Solver' visible=false");
		break;
	}
	switch (g_simulation->m_solver_type)
	{
	case SOLVER_TYPE_DIRECT_LLT:
		TwDefine("  'Simulation Settings'/'Max Iteration' visible=false");
		break;
	case SOLVER_TYPE_CG:
		TwDefine("  'Simulation Settings'/'Max Iteration' visible=true");
		break;
	default:
		TwDefine("  'Simulation Settings'/'Max Iteration' visible=false");
		break;
	}
	if (g_simulation->m_enable_line_search)
	{
		TwDefine(" 'Simulation Settings'/'Line Search Type' visible=true");
		TwDefine(" 'Simulation Settings'/'Alpha' visible=true");
		TwDefine(" 'Simulation Settings'/'Beta' visible=true");
		TwDefine(" 'Simulation Settings'/'Step Size' readonly=true");
	}
	else
	{
		TwDefine(" 'Simulation Settings'/'Line Search Type' visible=false");
		TwDefine(" 'Simulation Settings'/'Alpha' visible=false");
		TwDefine(" 'Simulation Settings'/'Beta' visible=false");
		TwDefine(" 'Simulation Settings'/'Step Size' readonly=false");
	}
	//if (g_simulation->m_animation_enable_swinging)
	//{
	//	TwDefine(" 'Simulation Settings'/'Swing Half Period' visible=true");
	//	TwDefine(" 'Simulation Settings'/'Swing Amplitude' visible=true");
	//	TwDefine(" 'Simulation Settings'/'Swing Direction' visible=true");
	//	TwDefine(" 'Simulation Settings'/'Swing Quantity' visible=true");
	//}
	//else
	//{
	//	TwDefine(" 'Simulation Settings'/'Swing Half Period' visible=false");
	//	TwDefine(" 'Simulation Settings'/'Swing Amplitude' visible=false");
	//	TwDefine(" 'Simulation Settings'/'Swing Direction' visible=false");
	//	TwDefine(" 'Simulation Settings'/'Swing Quantity' visible=false");
	//}

	// give feed back
	int atb_feedback = ATB_DEFAULT;
	if (g_old_screen_width!=g_screen_width || g_old_screen_height!=g_screen_height)
	{
		g_old_screen_width = g_screen_width;
		g_old_screen_height = g_screen_height;
		atb_feedback |= ATB_RESHAPE_WINDOW;
	}
	if (g_simulation->m_h != g_old_timestep ||\
		g_simulation->m_sub_stepping != g_old_substepping)
	{
		g_old_timestep = g_simulation->m_h;
		g_old_substepping = g_simulation->m_sub_stepping;

		atb_feedback |= ATB_CHANGE_TIME_STEP;
	}
	if (g_simulation->m_integration_method != g_old_integration_method)
	{
		g_old_integration_method = g_simulation->m_integration_method;

		atb_feedback |= ATB_CHANGE_INTEGRATION;
	}
	if (g_old_stretch_stiffness != g_simulation->m_stiffness_stretch ||\
		g_old_bending_stiffness != g_simulation->m_stiffness_bending ||\
		g_old_attachment_stiffness != g_simulation->m_stiffness_attachment ||\
		g_old_kappa != g_simulation->m_stiffness_kappa ||\
		g_old_laplacian_stiffness != g_simulation->m_stiffness_laplacian||\
		g_old_material_type != g_simulation->m_material_type)
	{
		g_old_stretch_stiffness = g_simulation->m_stiffness_stretch;
		g_old_bending_stiffness = g_simulation->m_stiffness_bending;
		g_old_attachment_stiffness = g_simulation->m_stiffness_attachment;
		g_old_kappa = g_simulation->m_stiffness_kappa;
		g_old_laplacian_stiffness = g_simulation->m_stiffness_laplacian;
		g_old_material_type = g_simulation->m_material_type;

		atb_feedback |= ATB_CHANGE_MATERIAL_PROPERTY;
	}
#ifdef ENABLE_MATLAB_DEBUGGING
	if (g_old_enable_matlab_debugger != g_debugger->m_enable)
	{
		g_old_enable_matlab_debugger = g_debugger->m_enable;

		atb_feedback |= ATB_INIT_MATLAB;
	}
#endif // ENABLE_MATLAB_DEBUGGING
	
	return atb_feedback;
}

void AntTweakBarWrapper::SaveSettings()
{
	std::ofstream outfile;
	outfile.open(DEFAULT_CONFIG_FILE, std::ifstream::out);
	if (outfile.is_open())
	{
		// TODO: change it to memory dump.
		// global settings:
		outfile << "HasEnd              " << g_recording_limit << std::endl;
		outfile << "TotalFrame          " << g_total_frame << std::endl;
		outfile << "MeshBody            " << g_show_mesh << std::endl;
		outfile << "Wireframe           " << g_show_wireframe << std::endl;
		outfile << "Wireframe           " << g_wireframe_linewidth << std::endl;
		outfile << "Texture             " << g_show_texture << std::endl;
		outfile << "ScreenWidth         " << g_screen_width << std::endl;
		outfile << "ScreenHeight        " << g_screen_height << std::endl;
		outfile << std::endl;

		// mesh settings:
		outfile << "MeshType            " << g_mesh->m_mesh_type << std::endl;
		outfile << "MeshMass            " << g_mesh->m_total_mass << std::endl;
		outfile << "TetFilePath         " << g_mesh->m_tet_file_path<< std::endl;
		outfile << "TetScaling          " << g_mesh->m_tet_scaling << std::endl;
		outfile << "TetFlip             " << g_mesh->m_tet_flip << std::endl;
		outfile << "TetRotation         " << g_mesh->m_tet_rotation << std::endl;
		outfile << "ClothDimension      " << g_mesh->m_dim[0] << " " \
										  << g_mesh->m_dim[1] \
										  << std::endl;
		outfile << "ClothCorners        " << g_mesh->m_corners[0][0] << " " \
										  << g_mesh->m_corners[0][1] << " " \
										  << g_mesh->m_corners[0][2] << " " \
										  << g_mesh->m_corners[1][0] << " " \
										  << g_mesh->m_corners[1][1] << " " \
										  << g_mesh->m_corners[1][2] \
										  << std::endl;
		outfile << std::endl;

		// selection tool settings
		outfile << "HandleColor         " << g_handle_color.x << " " \
										  << g_handle_color.y << " " \
										  << g_handle_color.z << std::endl;
		outfile << "RandomHandleColor   " << g_random_handle_color << std::endl;
		outfile << std::endl;

		// simulation settings:
		outfile << "SimMethod           " << g_simulation->m_integration_method << std::endl;
		outfile << "OptimizationMethod  " << g_simulation->m_optimization_method << std::endl;
		outfile << "Timestep            " << g_simulation->m_h << std::endl;
		outfile << "SubStepping         " << g_simulation->m_sub_stepping <<std::endl;
		//outfile << "QuasiStatics        " << g_simulation->m_quasi_statics << std::endl;
		outfile << "OpenMP              " << g_simulation->m_enable_openmp << std::endl;

		outfile << "TetConstraintType   " << g_simulation->m_material_type << std::endl;
		outfile << "AttachmentStiffness " << g_simulation->m_stiffness_attachment << std::endl;
		outfile << "StretchStiffness    " << g_simulation->m_stiffness_stretch << std::endl;
		outfile << "BendingStiffness    " << g_simulation->m_stiffness_bending << std::endl;
		outfile << "KappaStiffness      " << g_simulation->m_stiffness_kappa << std::endl;
		outfile << "AutoLaplacian       " << g_simulation->m_stiffness_auto_laplacian_stiffness << std::endl;
		outfile << "LaplacianStiffness  " << g_simulation->m_stiffness_laplacian << std::endl;
		outfile << "GravityConstant     " << g_simulation->m_gravity_constant << std::endl;
		outfile << "DampingCoefficient  " << g_simulation->m_damping_coefficient << std::endl;
		outfile << "RestiCoefficient    " << g_simulation->m_restitution_coefficient << std::endl;
		outfile << "FrictionCoefficient " << g_simulation->m_friction_coefficient << std::endl;

		outfile << "IterationsPerFrame  " << g_simulation->m_iterations_per_frame << std::endl;
		//outfile << "JacobiIterations    " << g_simulation->m_jacobi_iterations << std::endl;
		outfile << "DefinitenessFix     " << g_simulation->m_definiteness_fix << std::endl;

		outfile << "LSEnable            " << g_simulation->m_enable_line_search << std::endl;
		outfile << "LSExact             " << g_simulation->m_enable_exact_search << std::endl;
		outfile << "LSType              " << g_simulation->m_ls_type << std::endl;
		outfile << "LSStep              " << g_simulation->m_ls_step_size << std::endl;
		outfile << "LSAlpha             " << g_simulation->m_ls_alpha << std::endl;
		outfile << "LSBeta              " << g_simulation->m_ls_beta << std::endl;

		outfile << "AnimEnableSwing     " << g_simulation->m_animation_enable_swinging << std::endl;
		outfile << "AnimSwingAll        " << g_simulation->m_animation_swing_num << std::endl;
		outfile << "AnimHalfPeriod      " << g_simulation->m_animation_swing_half_period << std::endl;
		outfile << "AnimAmp             " << g_simulation->m_animation_swing_amp << std::endl;
		outfile << "AnimDir             " << g_simulation->m_animation_swing_dir[0] << " " \
										  << g_simulation->m_animation_swing_dir[1] << " " \
										  << g_simulation->m_animation_swing_dir[2] \
										  << std::endl;

		outfile << "LBFGSType           " << g_simulation->m_lbfgs_H0_type << std::endl;
		outfile << "LBFGSWindowSize     " << g_simulation->m_lbfgs_m << std::endl;

		outfile.close();
	}
	else
	{
		std::cerr << "Warning: Can not write config file. Settings not saved." << std::endl; 
	}
}

void AntTweakBarWrapper::LoadSettings()
{
	bool successfulRead = false;

	//read file
	std::ifstream infile;
	infile.open(DEFAULT_CONFIG_FILE, std::ifstream::in);
	if (successfulRead = infile.is_open())
	{
		int tempEnum;
		char ignoreToken[256];

		// global settings:
		infile >> ignoreToken >> g_recording_limit;
		infile >> ignoreToken >> g_total_frame;
		infile >> ignoreToken >> g_show_mesh;
		infile >> ignoreToken >> g_show_wireframe;
		infile >> ignoreToken >> g_wireframe_linewidth;
		infile >> ignoreToken >> g_show_texture;
		infile >> ignoreToken >> g_screen_width;
		infile >> ignoreToken >> g_screen_height;

		// mesh settings:
		infile >> ignoreToken >> tempEnum; g_mesh->m_mesh_type = MeshType(tempEnum);
		infile >> ignoreToken >> g_mesh->m_total_mass;
		infile >> ignoreToken >> g_mesh->m_tet_file_path;
		infile >> ignoreToken >> g_mesh->m_tet_scaling;
		infile >> ignoreToken >> g_mesh->m_tet_flip;
		infile >> ignoreToken >> g_mesh->m_tet_rotation;
		infile >> ignoreToken >> g_mesh->m_dim[0] \
							  >> g_mesh->m_dim[1];
		infile >> ignoreToken >> g_mesh->m_corners[0][0] \
							  >> g_mesh->m_corners[0][1] \
							  >> g_mesh->m_corners[0][2] \
							  >> g_mesh->m_corners[1][0] \
							  >> g_mesh->m_corners[1][1] \
							  >> g_mesh->m_corners[1][2];	
		
		// selection tool settings
		infile >> ignoreToken >> g_handle_color.x \
							  >> g_handle_color.y \
							  >> g_handle_color.z;
		infile >> ignoreToken >> g_random_handle_color;

		// simulation settings:
		infile >> ignoreToken >> tempEnum; g_simulation->m_integration_method = IntegrationMethod(tempEnum);
		infile >> ignoreToken >> tempEnum; g_simulation->m_optimization_method = OptimizationMethod(tempEnum);
		infile >> ignoreToken >> g_simulation->m_h;
		infile >> ignoreToken >> g_simulation->m_sub_stepping;
		//infile >> ignoreToken >> g_simulation->m_quasi_statics;
		infile >> ignoreToken >> g_simulation->m_enable_openmp;

		infile >> ignoreToken >> tempEnum; g_simulation->m_material_type = MaterialType(tempEnum);
		infile >> ignoreToken >> g_simulation->m_stiffness_attachment;
		infile >> ignoreToken >> g_simulation->m_stiffness_stretch;
		infile >> ignoreToken >> g_simulation->m_stiffness_bending;
		infile >> ignoreToken >> g_simulation->m_stiffness_kappa;
		infile >> ignoreToken >> g_simulation->m_stiffness_auto_laplacian_stiffness;
		infile >> ignoreToken >> g_simulation->m_stiffness_laplacian;
		infile >> ignoreToken >> g_simulation->m_gravity_constant;
		infile >> ignoreToken >> g_simulation->m_damping_coefficient;
		infile >> ignoreToken >> g_simulation->m_restitution_coefficient;
		infile >> ignoreToken >> g_simulation->m_friction_coefficient;

		infile >> ignoreToken >> g_simulation->m_iterations_per_frame;
		//infile >> ignoreToken >> g_simulation->m_jacobi_iterations;
		infile >> ignoreToken >> g_simulation->m_definiteness_fix;

		infile >> ignoreToken >> g_simulation->m_enable_line_search;
		infile >> ignoreToken >> g_simulation->m_enable_exact_search;
		infile >> ignoreToken >> tempEnum; g_simulation->m_ls_type = LinesearchType(tempEnum);
		infile >> ignoreToken >> g_simulation->m_ls_step_size;
		infile >> ignoreToken >> g_simulation->m_ls_alpha;
		infile >> ignoreToken >> g_simulation->m_ls_beta;

		infile >> ignoreToken >> g_simulation->m_animation_enable_swinging;
		infile >> ignoreToken >> g_simulation->m_animation_swing_num;
		infile >> ignoreToken >> g_simulation->m_animation_swing_half_period;
		infile >> ignoreToken >> g_simulation->m_animation_swing_amp;
		infile >> ignoreToken >> g_simulation->m_animation_swing_dir[0] \
			                  >> g_simulation->m_animation_swing_dir[1] \
							  >> g_simulation->m_animation_swing_dir[2];

		infile >> ignoreToken >> tempEnum; g_simulation->m_lbfgs_H0_type = LBFGSH0Type(tempEnum);
		infile >> ignoreToken >> g_simulation->m_lbfgs_m;

		infile.close();
	}

	// setup default values
	if (!successfulRead)
	{
		std::cerr << "Waning: failed loading settings, set to defaults." << std::endl;
		DefaultSettings();
	}

	// init event related variables
	g_old_screen_width = g_screen_width;
	g_old_screen_height = g_screen_height;
}

void AntTweakBarWrapper::DefaultSettings()
{
	// global settings
	g_recording_limit = true;
	g_total_frame = 1000;
	g_show_mesh = true;
	g_show_wireframe = false;
	g_wireframe_linewidth = 1;
	g_show_texture = false;
	g_screen_width = 1024;
	g_screen_height = 768;

	// mesh settings
	g_mesh->m_mesh_type = MESH_TYPE_CLOTH;
	g_mesh->m_total_mass = 1.0;
	// tet
	strcpy(g_mesh->m_tet_file_path, DEFAULT_MODEL);
	g_mesh->m_tet_scaling = 1.0;
	g_mesh->m_tet_flip = false;
	g_mesh->m_tet_rotation = 0;
	// cloth
	g_mesh->m_dim[0] = 21;
	g_mesh->m_dim[1] = 21;
	g_mesh->m_corners[0] = EigenVector3(-5, 8, -5); 
	g_mesh->m_corners[1] = EigenVector3(5, -1.85, -3.26);

	// selection
	g_handle_color = glm::vec3(0.3f, 1.0f, 0.3f);
	g_random_handle_color = false;

	//simulation settings
	g_simulation->m_integration_method = INTEGRATION_IMPLICIT_EULER;
	g_simulation->m_optimization_method = OPTIMIZATION_METHOD_LBFGS;
	g_simulation->m_h = 0.0333;
	g_simulation->m_sub_stepping = 1;
	//g_simulation->m_quasi_statics = false;

	g_simulation->m_stiffness_attachment = 120;
	g_simulation->m_stiffness_stretch = 80;
	g_simulation->m_stiffness_bending = 20;
	g_simulation->m_stiffness_kappa = 100;
	g_simulation->m_stiffness_auto_laplacian_stiffness;
	g_simulation->m_stiffness_laplacian = 2 * g_simulation->m_stiffness_stretch + g_simulation -> m_stiffness_bending;
	g_simulation->m_gravity_constant = 100;
	g_simulation->m_damping_coefficient = 0.001;
	g_simulation->m_restitution_coefficient = 1;
	g_simulation->m_friction_coefficient = 0.001;

	g_simulation->m_iterations_per_frame = 10;
	//g_simulation->m_jacobi_iterations = 1;
	g_simulation->m_definiteness_fix = true;
	g_simulation->m_enable_line_search = true;
	g_simulation->m_enable_exact_search = false;
	g_simulation->m_ls_type = LS_TYPE_ARMIJO;
	g_simulation->m_ls_step_size = 1.0;
	g_simulation->m_ls_alpha = 0.03;
	g_simulation->m_ls_beta = 0.5;

	g_simulation->m_animation_enable_swinging = false;
	g_simulation->m_animation_swing_num = 1;
	g_simulation->m_animation_swing_amp = 2.0;
	g_simulation->m_animation_swing_half_period = 10;
	g_simulation->m_animation_swing_dir[0] = 0;
	g_simulation->m_animation_swing_dir[1] = 1;
	g_simulation->m_animation_swing_dir[2] = 0;

	g_simulation->m_lbfgs_H0_type = LBFGS_H0_LAPLACIAN;
	g_simulation->m_lbfgs_m = 5;
}

void TW_CALL AntTweakBarWrapper::SaveSettings(void* atb_wrapper)
{
	AntTweakBarWrapper* atb_wrapper_ref = (AntTweakBarWrapper*) atb_wrapper;
	atb_wrapper_ref->SaveSettings();
}

void TW_CALL AntTweakBarWrapper::LoadSettings(void* atb_wrapper)
{
	AntTweakBarWrapper* atb_wrapper_ref = (AntTweakBarWrapper*) atb_wrapper;
	atb_wrapper_ref->LoadSettings();
	//resetSimulation(NULL);
}

void TW_CALL AntTweakBarWrapper::SetDefaultSettings(void* atb_wrapper)
{
	AntTweakBarWrapper* atb_wrapper_ref = (AntTweakBarWrapper*) atb_wrapper;
	atb_wrapper_ref->DefaultSettings();
	//resetSimulation(NULL);
}