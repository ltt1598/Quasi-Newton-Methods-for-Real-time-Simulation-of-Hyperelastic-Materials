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

#ifndef _SIMULATION_H_
#define _SIMULATION_H_

//#define PARDISO_SUPPORT

#include <vector>
#include <deque>

#include "global_headers.h"
#include "anttweakbar_wrapper.h"
#include "mesh.h"
#include "constraint.h"
#include "scene.h"
#include "handle.h"

#ifdef PARDISO_SUPPORT
#include <Eigen/PardisoSupport>
#endif

//#include "Eigen/SVD"

class Mesh;
class AntTweakBarWrapper;

typedef enum
{
	INTEGRATION_IMPLICIT_EULER,
	INTEGRATION_IMPLICIT_BDF2,
	INTEGRATION_IMPLICIT_MIDPOINT,
	INTEGRATION_IMPLICIT_NEWMARK_BETA,
	INTEGRATION_QUASI_STATICS,
	INTEGRATION_TOTAL_NUM

} IntegrationMethod;

typedef enum
{
	OPTIMIZATION_METHOD_GRADIENT_DESCENT,
	OPTIMIZATION_METHOD_NEWTON,
	OPTIMIZATION_METHOD_LBFGS,
	OPTIMIZATION_METHOD_TOTAL_NUM

} OptimizationMethod;

typedef enum
{
	SOLVER_TYPE_DIRECT_LLT,
	SOLVER_TYPE_CG,
	SOLVER_TYPE_TOTAL_NUM
} SolverType;

typedef enum
{
	LBFGS_H0_IDENTITY,
	LBFGS_H0_LAPLACIAN,
	LBFGS_H0_TOTAL_NUM
} LBFGSH0Type;

typedef enum
{
	LS_TYPE_ARMIJO,
	LS_TYPE_WOLFE,
	LS_TYPE_TOTAL_NUM,
} LinesearchType;

class Simulation
{
	friend class AntTweakBarWrapper;

public:
	Simulation();
	virtual ~Simulation();

	void Reset();
	void UpdateAnimation(const int fn);
	void Update();
	void Draw(const VBO& vbos);

	void GetOverlayChar(char* overlay, unsigned int overlay_length = 255);

	// select/unselect/move/save/load attachment constratins
	ScalarType TryToSelectAttachmentConstraint(const EigenVector3& p0, const EigenVector3& dir); // return ray_projection_plane_distance if hit; return -1 otherwise.
	bool TryToToggleAttachmentConstraint(const EigenVector3& p0, const EigenVector3& dir); // true if hit some vertex/constraint
	void SelectAtttachmentConstraint(AttachmentConstraint* ac);
	void UnselectAttachmentConstraint();
	AttachmentConstraint* AddAttachmentConstraint(unsigned int vertex_index); // add one attachment constraint at vertex_index
	AttachmentConstraint* AddAttachmentConstraint(unsigned int vertex_index, const EigenVector3& target); // add one attachment constraint at vertex_index
	void MoveSelectedAttachmentConstraintTo(const EigenVector3& target); // move selected attachement constraint to target
	void SaveAttachmentConstraint(const char* filename);
	void LoadAttachmentConstraint(const char* filename);

	// handles
	void NewHandle(const std::vector<unsigned int>& indices, const glm::vec3 color);
	void DeleteHandle();
	bool SelectHandle(std::vector<glm::vec3> ray);
	void MoveHandleTemporary(const glm::vec3& trans);
	void MoveHandleFinalize();
	void RotateHandleToValue();
	void RotateHandleSetStepSize();
	void RotateHandleTemporary(const glm::vec3& axis, const float& theta);
	void RotateHandleFinalize();
	void UpdateHandleInfoToConstraints(Handle& selected_handle);
	//void CombineVertexClassificationRegions(std::vector<RegionClassification>& rc);
	glm::vec3 SelectedHandleLocalCoM();
	glm::vec3 SelectedHandleCoM();
	// handle animations
	void SetHandleTranslationAnimation();
	void SetHandleTranslation();
	void SetHandleRotationAnimation();
	void SetHandleRotation();
	void AnimateHandle(const int current_frame);
	void SaveHandleAnimation(const char* filename);
	void LoadHandleAnimation(const char* filename);
	// save load reset
	void SaveHandles(const char* filename);
	void LoadHandles(const char* filename);
	void ResetHandles();

	// save laplacian matrix
	void SaveSparseMatrix(const SparseMatrix& A, const char* filename);
	void SaveLaplacianMatrix(const char* filename);

	// matlab debugger related
	void SetConvergedEnergy();

	// randomize points
	void RandomizePoints();

	// set material property for selected elements
	void SetMaterialProperty(std::vector<Constraint*>& constraints);
	void SetMaterialProperty(std::vector<Constraint*>& constraints, MaterialType type, ScalarType stretch, ScalarType bending, ScalarType kappa, ScalarType laplacian_coeff);
	// set material property for all elements
	void SetMaterialProperty();

	// select constraints and change material properties
	void GetPartialMaterialProperty();
	void SetPartialMaterialProperty();
	void SavePerConstraintMaterialProperties(const char* filename);
	void LoadPerConstraintMaterialProperties(const char* filename);
	void SelectTetConstraints(const std::vector<unsigned int>& indices);

	// eigen value visualization mesh
	void NewVisualizationMesh();
	void DeleteVisualizationMesh();
	void ResetVisualizationMesh();
	void SetVisualizationMesh();
	inline Mesh* GetEigenVectorVisMesh(){ return m_eigenvector_vis_mesh; }
	void ResetVisualizationMeshHeight();

	// inline functions
	inline void SetReprefactorFlag() 
	{
		m_precomputing_flag = false;
		m_prefactorization_flag = false;
		m_prefactorization_flag_newton = false;
	}
	inline void SetMesh(Mesh* mesh) {m_mesh = mesh;}
	inline void SetScene(Scene* scene) {m_scene = scene;}
	inline void SetStepMode(bool step_mode) {m_step_mode = step_mode;}
	inline ScalarType Timestep() { return m_h; }

protected:

	// simulation constants
	ScalarType m_h; // time_step
	unsigned int m_sub_stepping; // 
	bool m_step_mode;

	// simulation constants
	ScalarType m_gravity_constant;
	MaterialType m_material_type;
	ScalarType m_stiffness_attachment;
	ScalarType m_stiffness_stretch;
	ScalarType m_stiffness_high;
	ScalarType m_stiffness_bending;
	ScalarType m_stiffness_kappa;
	bool m_stiffness_auto_laplacian_stiffness;
	ScalarType m_stiffness_laplacian;
	ScalarType m_damping_coefficient;
	ScalarType m_restitution_coefficient;
	ScalarType m_friction_coefficient;

	// integration and optimization method
	IntegrationMethod m_integration_method;
	OptimizationMethod m_optimization_method;

	// key simulation components: mesh and scene
	Mesh *m_mesh;
	Scene *m_scene;
	// for other visualizations
	Mesh *m_eigenvector_vis_mesh = NULL;
	// key simulation components: constraints
	std::vector<Constraint*> m_constraints;
	//std::vector<Constraint*>::iterator m_attachment_constraint_start_point;
	AttachmentConstraint* m_selected_attachment_constraint;
	// collision constraints
	std::vector<CollisionSpringConstraint> m_collision_constraints;

	// partial material control
	std::vector<Constraint*> m_selected_constraints;
	MaterialType m_partial_material_type;
	ScalarType m_partial_stiffness_stretch;
	ScalarType m_partial_stiffness_bending;
	ScalarType m_partial_stiffness_kappa;

	// handle control
	std::vector<int> m_handle_id; // the id of the correspond handle of a vertex, -1 if no handle
	int m_selected_handle_id;
	std::vector<Handle> m_handles;
	// handle animation
	int m_keyframe_handle_id_translation;
	int m_keyframe_handle_id_rotation;
	int m_keyframe_handle_unit_translation_total_segments;
	int m_keyframe_handle_unit_rotation_total_segments;
	// translation	
	std::vector<int> m_keyframe_handle_unit_translation_end_frames;
	std::vector<EigenVector3> m_keyframe_handle_unit_translation_axis;
	std::vector<ScalarType> m_keyframe_handle_unit_translation_amount;
	std::vector<int> m_keyframe_handle_unit_rotation_end_frames;
	std::vector<EigenVector3> m_keyframe_handle_unit_rotation_axis;
	std::vector<ScalarType> m_keyframe_handle_unit_rotation_degree;

	// key simulation states
	VectorX m_qn;
	VectorX m_vn;
	VectorX m_qn_minus_one;
	VectorX m_vn_minus_one;
	VectorX m_qn_minus_two;
	VectorX m_vn_minus_two;

	// constant term in optimization:
	// 0.5(x-y)^2 M (x-y) + (c) * h^2 * E(x) - h^2 * x^T * z;
	VectorX m_y;
	VectorX m_z;

	// external force (gravity, wind, etc...)
	VectorX m_external_force;

	// for optimization method, number of iterations
	unsigned int m_iterations_per_frame;

	// for optimization method
	unsigned int m_current_iteration;

	// line search 
	bool m_enable_line_search;
	bool m_enable_exact_search;
	LinesearchType m_ls_type;
	ScalarType m_ls_alpha;
	ScalarType m_ls_beta;
	ScalarType m_ls_step_size;
	// prefetched instructions in linesearch
	bool m_ls_is_first_iteration;
	VectorX m_ls_prefetched_gradient;
	ScalarType m_ls_prefetched_energy;

	// local global method
	bool m_enable_openmp;
	VectorX m_last_descent_dir;

	// for prefactorization
	SparseMatrix m_weighted_laplacian_1D;
	SparseMatrix m_weighted_laplacian;

	bool m_precomputing_flag;
	bool m_prefactorization_flag;
	bool m_prefactorization_flag_newton;
#ifdef PARDISO_SUPPORT
	Eigen::PardisoLLT<SparseMatrix, Eigen::Upper> m_prefactored_solver;
	Eigen::PardisoLLT<SparseMatrix, Eigen::Upper> m_newton_solver;
	Eigen::PardisoLLT<SparseMatrix, Eigen::Upper> m_prefactored_solver_restpose_hessian;
	Eigen::PardisoLLT<SparseMatrix, Eigen::Upper> m_prefactored_solver_dual;
#else
	Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper> m_prefactored_solver_1D;
	Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper> m_prefactored_solver;
	Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper> m_newton_solver;
#endif
	Eigen::ConjugateGradient<SparseMatrix> m_preloaded_cg_solver_1D;
	Eigen::ConjugateGradient<SparseMatrix> m_preloaded_cg_solver;

	// for Newton's method
	bool m_definiteness_fix;

	// solver type
	SolverType m_solver_type;
	int m_iterative_solver_max_iteration;

	// LBFGS
	bool m_lbfgs_restart_every_frame;
	LBFGSH0Type m_lbfgs_H0_type;
	int m_lbfgs_m; // back-track history length
	int m_lbfgs_max_m;
	bool m_lbfgs_need_update_H0;
	SparseMatrix m_lbfgs_B0;
	VectorX m_lbfgs_H0_diagonal;
	Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper> m_lbfgs_H0_solver;
	VectorX m_lbfgs_last_x;
	VectorX m_lbfgs_last_gradient;
	std::deque<VectorX> m_lbfgs_y_queue;
	std::deque<VectorX> m_lbfgs_s_queue;
	QueueLBFGS* m_lbfgs_queue;

	// volume
	ScalarType m_restshape_volume;
	ScalarType m_current_volume;

	// verbose
	bool m_verbose_show_converge;
	bool m_verbose_show_optimization_time;
	bool m_verbose_show_energy;
	bool m_verbose_show_factorization_warning;

	// animation for demo
	bool m_animation_enable_swinging;
	int m_animation_swing_num;
	int m_animation_swing_half_period;
	ScalarType m_animation_swing_amp;
	ScalarType m_animation_swing_dir[3];

	// hard coded collision plane for demo
	bool m_processing_collision;

private:

	// main update sub-routines
	void clearConstraints(); // cleanup all constraints
	void setupConstraints(); // initialize constraints
	void dampVelocity(); // damp velocity at the end of each iteration.
	void calculateExternalForce(); // wind force is propotional to the area of triangles projected on the tangential plane
	VectorX collisionDetectionPostProcessing(const VectorX& x); // detect collision and return a vector of penetration
	void collisionDetection(const VectorX& x); 
	void collisionResolution(const VectorX& penetration, VectorX& x, VectorX& v);

	void integrateImplicitMethod();

	// all those "OneIteration" functions will be called in a loop
	// x is initially passed as the initial guess of the next postion (i.e. inertia term): x = y = current_pos + current_vel*h
	// x will be changed during these subroutines in EVERY iteration
	// the final value of x will be the next_pos that we used to update all vertices.
	bool performGradientDescentOneIteration(VectorX& x);
	bool performNewtonsMethodOneIteration(VectorX& x);
	bool performLBFGSOneIteration(VectorX& x);// our method
	void LBFGSKernelLinearSolve(VectorX& r, VectorX gf_k, ScalarType scaled_identity_constant);

	// key initializations and constants computations
	void computeConstantVectorsYandZ();
	void updatePosAndVel(const VectorX& new_pos);

	// evaluate energy
	ScalarType evaluateEnergy(const VectorX& x);
	// evaluate gradient
	void evaluateGradient(const VectorX& x, VectorX& gradient, bool enable_omp = false);
	// evaluate gradient and energy
	ScalarType evaluateEnergyAndGradient(const VectorX& x, VectorX& gradient);
	// evaluate Hessian Matrix
	void evaluateHessian(const VectorX& x, SparseMatrix& hessian_matrix);
	void evaluateHessianSmart(const VectorX& x, SparseMatrix& hessian_matrix);
	// evaluate hessian
	void evaluateHessianForCG(const VectorX& x);
	// apply hessian
	void applyHessianForCG(const VectorX& x, VectorX & b);
	// evaluate Weighted Laplacian Matrix
	void evaluateLaplacian(SparseMatrix& laplacian_matrix);
	// evaluate Weighted Laplacian Matrix nxn
	void evaluateLaplacian1D(SparseMatrix& laplacian_matrix_1d);

	// accesser 
	// volume
	ScalarType getVolume(const VectorX& x);

	// testing
	// print volume of the elements
	void printVolumeTesting(const VectorX& x);
	
	// energy conservation
	ScalarType evaluatePotentialEnergy(const VectorX& x);
	ScalarType evaluateKineticEnergy(const VectorX& v);
	ScalarType evaluateTotalEnergy(const VectorX& x, const VectorX& v);

	// basic building blocks
	ScalarType evaluateEnergyPureConstraint(const VectorX& x, const VectorX& f_ext);
	void evaluateGradientPureConstraint(const VectorX& x, const VectorX& f_ext, VectorX& gradient);
	ScalarType evaluateEnergyAndGradientPureConstraint(const VectorX& x, const VectorX& f_ext, VectorX& gradient);
	void evaluateHessianPureConstraint(const VectorX& x, SparseMatrix& hessian_matrix);
	void evaluateHessianPureConstraintSmart(const VectorX& x, SparseMatrix& hessian_matrix);
	void evaluateLaplacianPureConstraint(SparseMatrix& laplacian_matrix);
	void evaluateLaplacianPureConstraint1D(SparseMatrix& laplacian_matrix_1d);
	void applyHessianForCGPureConstraint(const VectorX& x, VectorX& b); // b = H*x

	// collision
	ScalarType evaluateEnergyCollision(const VectorX& x);
	void evaluateGradientCollision(const VectorX& x, VectorX& gradient);
	ScalarType evaluateEnergyAndGradientCollision(const VectorX& x, VectorX& gradient);
	void evaluateHessianCollision(const VectorX& x, SparseMatrix& hessian_matrix);

	// line search
	ScalarType lineSearch(const VectorX& x, const VectorX& gradient_dir, const VectorX& descent_dir);
	ScalarType linesearchWithPrefetchedEnergyAndGradientComputing(const VectorX& x, const ScalarType current_energy, const VectorX& gradient_dir, const VectorX& descent_dir, ScalarType& next_energy, VectorX& next_gradient_dir);

	// matrices and prefactorizations
	void precomputeLaplacianWeights();
	void precomputeLaplacian();
	void setWeightedLaplacianMatrix();
	void setWeightedLaplacianMatrix1D();
	void prefactorize();

	// newton solver
	void analyzeNewtonSolverPattern(const SparseMatrix& A);
	void factorizeNewtonSolver(const SparseMatrix& A, char* warning_msg = "");


	// utility functions
	// linear solver
	ScalarType linearSolve(VectorX& x, const SparseMatrix& A, const VectorX& b, char* msg = "");
	// conjugate gradient solver
	ScalarType conjugateGradientWithInitialGuess(VectorX& x, const SparseMatrix& A, const VectorX& b, const unsigned int max_it = 200, const ScalarType tol = 1e-5);
	void factorizeDirectSolverLLT(const SparseMatrix& A, Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>& lltSolver, char* warning_msg = ""); // factorize matrix A using LLT decomposition

	void generateRandomVector(const unsigned int size, VectorX& x); // generate random vector varing from [-1 1].
};

#endif