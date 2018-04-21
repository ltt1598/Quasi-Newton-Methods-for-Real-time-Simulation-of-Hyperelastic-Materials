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

#pragma warning( disable : 4244 4267 4305 4996) 

#include <omp.h>
#include <exception>

#include <Eigen/Eigenvalues>

#include "simulation.h"
#include "timer_wrapper.h"

#ifdef ENABLE_MATLAB_DEBUGGING
#include "matlab_debugger.h"
extern MatlabDebugger *g_debugger;
#endif

//#define USE_STL_QUEUE_IMPLEMENTATION
//#define OUTPUT_LS_ITERATIONS
#ifdef OUTPUT_LS_ITERATIONS
#define OUTPUT_LS_ITERATIONS_EVERY_N_FRAMES 100
ScalarType g_total_ls_iterations;
int g_total_iterations;
#endif

TimerWrapper g_integration_timer;
TimerWrapper g_lbfgs_timer;

ScalarType rest_length_adjust = 1; // 1  = normal spring, 0 = zero length spring

// comparison function for sort
bool compareTriplet(SparseMatrixTriplet i, SparseMatrixTriplet j)
{ 
	return (abs(i.value())<abs(j.value())); 
}

void Vector3mx1ToMatrixmx3(const VectorX&x, EigenMatrixx3& m)
{
	for (unsigned int i = 0; i < m.rows(); i++)
	{
		m.block<1, 3>(i, 0) = x.block_vector(i).transpose();
	}
}
void Vector3mx1ToMatrixmx3(const VectorX&x, Matrix& m)
{
	for (unsigned int i = 0; i < m.rows(); i++)
	{
		m.block<1, 3>(i, 0) = x.block_vector(i).transpose();
	}
}

void Matrixmx3ToVector3mx1(const EigenMatrixx3& m, VectorX&x)
{
	for (unsigned int i = 0; i < m.rows(); i++)
	{
		x.block_vector(i) = m.block<1, 3>(i, 0).transpose();
	}
}
void Matrixmx3ToVector3mx1(const Matrix& m, VectorX&x)
{
	assert(m.cols() == 3);

	for (unsigned int i = 0; i < m.rows(); i++)
	{
		x.block_vector(i) = m.block<1, 3>(i, 0).transpose();
	}
}

void EigenSparseDiagonalToVector(VectorX& dst, const SparseMatrix& src)
{
	assert(src.rows() == src.cols());
	dst.resize(src.rows());

	for (unsigned int i = 0; i != src.rows(); ++i)
	{
		dst(i) = src.coeff(i, i);
	}
}

Simulation::Simulation()
{
	m_lbfgs_queue = NULL;

	m_processing_collision = true;

	m_verbose_show_converge = false;
	m_verbose_show_optimization_time = false;
	m_verbose_show_energy = false;
	m_verbose_show_factorization_warning = true;
}

Simulation::~Simulation()
{
	clearConstraints();
	m_handles.clear();
	m_handle_id.clear();

	DeleteVisualizationMesh();
}

void Simulation::Reset()
{	
	m_y.resize(m_mesh->m_system_dimension);
	m_external_force.resize(m_mesh->m_system_dimension);

	// handles
	if (!m_handles.empty())
		m_handles.clear();
	m_handle_id.resize(m_mesh->m_vertices_number);
	for (unsigned int i = 0; i != m_handle_id.size(); ++i)
	{
		m_handle_id[i] = -1;
	}
	m_selected_handle_id = -1;

	m_mesh->m_expanded_system_dimension = 0;
	m_mesh->m_expanded_system_dimension_1d = 0;

	setupConstraints();
	SetMaterialProperty();

	m_selected_attachment_constraint = NULL;
	m_step_mode = false;

	// lbfgs
	m_lbfgs_restart_every_frame = true;
	m_lbfgs_need_update_H0 = true;

	// solver type
	m_solver_type = SOLVER_TYPE_DIRECT_LLT;
	m_iterative_solver_max_iteration = 10;

	// volume
	m_restshape_volume = getVolume(m_mesh->m_current_positions);
	m_current_volume = m_restshape_volume;

	// animation
	m_keyframe_handle_unit_translation_total_segments = 0;
	m_keyframe_handle_unit_rotation_total_segments = 0;

	// partial material property editting
	m_selected_constraints.clear();

	// collision
	m_collision_constraints.clear();

#ifdef OUTPUT_LS_ITERATIONS
	g_total_ls_iterations = 0;
	g_total_iterations = 0;
#endif
}

void Simulation::UpdateAnimation(const int fn)
{
	if (m_animation_enable_swinging)
	{
		int swing_num = 0;
		ScalarType swing_step = m_animation_swing_amp / m_animation_swing_half_period;
		EigenVector3 swing_dir(m_animation_swing_dir[0], m_animation_swing_dir[1], m_animation_swing_dir[2]);
		int positive_direction = ((fn/m_animation_swing_half_period)%2)?-1:1;
		for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
		{
			AttachmentConstraint* ac;
			if (ac = dynamic_cast<AttachmentConstraint*>(*c)) // is attachment constraint
			{
				EigenVector3 new_fixed_point = ac->GetFixedPoint() + swing_dir*swing_step*positive_direction;
				ac->SetFixedPoint(new_fixed_point);
				if (++swing_num >= m_animation_swing_num)
				{
					break;
				}
			}
		}
	}
}

void Simulation::Update()
{
	// update external force
	calculateExternalForce();

	ScalarType old_h = m_h;
	m_h = m_h / m_sub_stepping;

	m_last_descent_dir.resize(m_mesh->m_system_dimension);
	m_last_descent_dir.setZero();

	for (unsigned int substepping_i = 0; substepping_i != m_sub_stepping; substepping_i ++)
	{
		// update inertia term
		computeConstantVectorsYandZ();

		// update
		switch (m_integration_method)
		{
		case INTEGRATION_QUASI_STATICS:
		case INTEGRATION_IMPLICIT_EULER:
		case INTEGRATION_IMPLICIT_BDF2:
		case INTEGRATION_IMPLICIT_MIDPOINT:
		case INTEGRATION_IMPLICIT_NEWMARK_BETA:
			integrateImplicitMethod();
			break;
		}

		// damping
		dampVelocity();
	}

	//// volume
	//m_current_volume = getVolume(m_mesh->m_current_positions);

	if (m_verbose_show_energy)
	{
		if (m_integration_method == INTEGRATION_QUASI_STATICS)
		{
			ScalarType W = evaluatePotentialEnergy(m_mesh->m_current_positions);
			std::cout << "Potential Energy = " << W << std::endl;
		}
		else
		{
			// output total energy;
			ScalarType K = evaluateKineticEnergy(m_mesh->m_current_velocities);
			ScalarType W = evaluatePotentialEnergy(m_mesh->m_current_positions);
			ScalarType K_plus_W = K + W;
			std::cout << "Kinetic Energy = " << K << std::endl;
			std::cout << "Potential Energy = " << W << std::endl;
			std::cout << "Total Energy = " << K_plus_W << std::endl;
		}
	}
	m_h = old_h;
}

void Simulation::Draw(const VBO& vbos)
{
	//// draw attachment constraints
	//for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
	//{
	//	(*it)->Draw(vbos);
	//}
	// draw handles
	for (std::vector<Handle>::iterator it = m_handles.begin(); it != m_handles.end(); ++it)
	{
		it->Draw(vbos, (it->ID() == m_selected_handle_id));
	}
}

void Simulation::GetOverlayChar(char* overlay, unsigned int size)
{
	if (m_mesh->m_mesh_type == MESH_TYPE_TET)
	{
		sprintf_s(overlay, size, "| #vertices: %d, #elements: %d | Restshape Volume: %.1lf, Current Volume: %.1lf (%.1lf%%)", m_mesh->m_vertices_number, m_constraints.size(), m_restshape_volume, m_current_volume, m_current_volume / m_restshape_volume * 100);
	}
	else
	{
		sprintf_s(overlay, size, "| #vertices: %d, #elements: %d", m_mesh->m_vertices_number, m_constraints.size());
	}
}

ScalarType Simulation::TryToSelectAttachmentConstraint(const EigenVector3& p0, const EigenVector3& dir)
{
	ScalarType ray_point_dist;
	ScalarType min_dist = 100.0;
	AttachmentConstraint* best_candidate = NULL;

	bool current_state_on = false;
	for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
	{
		AttachmentConstraint* ac;
		if (ac = dynamic_cast<AttachmentConstraint*>(*c)) // is attachment constraint
		{
			ray_point_dist = ((ac->GetFixedPoint()-p0).cross(dir)).norm();
			if (ray_point_dist < min_dist)
			{
				min_dist = ray_point_dist;
				best_candidate = ac;
			}
		}
	}
	// exit if no one fits
	if (min_dist > DEFAULT_SELECTION_RADIUS)
	{
		UnselectAttachmentConstraint();

		return -1;
	}
	else
	{
		SelectAtttachmentConstraint(best_candidate);
		EigenVector3 fixed_point_temp = m_mesh->m_current_positions.block_vector(m_selected_attachment_constraint->GetConstrainedVertexIndex());

		return (fixed_point_temp-p0).dot(dir); // this is m_cached_projection_plane_distance
	}
}

bool Simulation::TryToToggleAttachmentConstraint(const EigenVector3& p0, const EigenVector3& dir)
{
	EigenVector3 p1;

	ScalarType ray_point_dist;
	ScalarType min_dist = 100.0;
	unsigned int best_candidate = 0;
	// first pass: choose nearest point
	for (unsigned int i = 0; i != m_mesh->m_vertices_number; i++)
	{
		p1 = m_mesh->m_current_positions.block_vector(i);
		
		ray_point_dist = ((p1-p0).cross(dir)).norm();
		if (ray_point_dist < min_dist)
		{
			min_dist = ray_point_dist;
			best_candidate = i;
		}
	}
	for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
	{
		AttachmentConstraint* ac;
		if (ac = dynamic_cast<AttachmentConstraint*>(*c)) // is attachment constraint
		{
			ray_point_dist = ((ac->GetFixedPoint()-p0).cross(dir)).norm();
			if (ray_point_dist < min_dist)
			{
				min_dist = ray_point_dist;
				best_candidate = ac->GetConstrainedVertexIndex();
			}
		}
	}
	// exit if no one fits
	if (min_dist > DEFAULT_SELECTION_RADIUS)
	{
		return false;
	}
	// second pass: toggle that point's fixed position constraint
	bool current_state_on = false;
	for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
	{
		AttachmentConstraint* ac;
		if (ac = dynamic_cast<AttachmentConstraint*>(*c)) // is attachment constraint
		{
			if (ac->GetConstrainedVertexIndex() == best_candidate)
			{
				current_state_on = true;
				m_constraints.erase(c);
				delete ac;
				m_mesh->m_expanded_system_dimension-=3;
				m_mesh->m_expanded_system_dimension_1d-=1;
				break;
			}
		}
	}
	if (!current_state_on)
	{
		AddAttachmentConstraint(best_candidate);
	}

	return true;
}

void Simulation::SelectAtttachmentConstraint(AttachmentConstraint* ac)
{
	//m_selected_attachment_constraint = ac;
	//m_selected_attachment_constraint->Select();
}

void Simulation::UnselectAttachmentConstraint()
{
	//if (m_selected_attachment_constraint)
	//{
	//	m_selected_attachment_constraint->UnSelect();
	//}
	//m_selected_attachment_constraint = NULL;
}

AttachmentConstraint* Simulation::AddAttachmentConstraint(unsigned int vertex_index)
{
	AttachmentConstraint* ac = new AttachmentConstraint(vertex_index, m_mesh->m_current_positions.block_vector(vertex_index));
	ac->SetMaterialProperty(m_stiffness_attachment);
	m_constraints.push_back(ac);
	m_mesh->m_expanded_system_dimension+=3;
	m_mesh->m_expanded_system_dimension_1d+=1;

	return ac;
}

AttachmentConstraint* Simulation::AddAttachmentConstraint(unsigned int vertex_index, const EigenVector3& target)
{
	AttachmentConstraint* ac = new AttachmentConstraint(vertex_index, target);
	ac->SetMaterialProperty(m_stiffness_attachment);
	m_constraints.push_back(ac);
	m_mesh->m_expanded_system_dimension += 3;
	m_mesh->m_expanded_system_dimension_1d += 1;

	return ac;
}

void Simulation::MoveSelectedAttachmentConstraintTo(const EigenVector3& target)
{
	if (m_selected_attachment_constraint)
		m_selected_attachment_constraint->SetFixedPoint(target);
}

void Simulation::SaveAttachmentConstraint(const char* filename)
{
	std::ofstream outfile;
	outfile.open(filename, std::ifstream::out);
	if (outfile.is_open())
	{
		int existing_vertices = 0;
		for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
		{
			(*c)->WriteToFileOBJHead(outfile);
		}
		outfile << std::endl;
		for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
		{
			(*c)->WriteToFileOBJ(outfile, existing_vertices);
		}

		outfile.close();
	}
}

void Simulation::LoadAttachmentConstraint(const char* filename)
{
	// clear current attachement constraints
	for (std::vector<Constraint*>::iterator& c = m_constraints.begin(); c != m_constraints.end(); )
	{
		AttachmentConstraint* ac;
		if (ac = dynamic_cast<AttachmentConstraint*>(*c)) // is attachment constraint
		{
			c = m_constraints.erase(c);
			delete ac;
			m_mesh->m_expanded_system_dimension-=3;
			m_mesh->m_expanded_system_dimension_1d-=1;
		}
		else
		{
			c++;
		}
	}

	// read from file
	std::ifstream infile;
	infile.open(filename, std::ifstream::in);
	char ignore[256];
	if (infile.is_open())
	{
		while(!infile.eof())
		{
			int id;
			EigenVector3 p;
			if (infile >> ignore >> id >> p[0] >> p[1] >> p[2])
			{
				if (strcmp(ignore, "v") == 0)
					break;
				AttachmentConstraint* ac = new AttachmentConstraint(id, p);
				ac->SetMaterialProperty(m_stiffness_attachment);
				m_constraints.push_back(ac);
				m_mesh->m_expanded_system_dimension+=3;
				m_mesh->m_expanded_system_dimension_1d+=1;
			}
			else
				break;
		}

		infile.close();
	}

}

void Simulation::NewHandle(const std::vector<unsigned int>& indices, const glm::vec3 color)
{
	// check if any points has already been assigned as handle
	for (unsigned int i = 0; i != indices.size(); ++i)
	{
		if (m_handle_id[indices[i]] >= 0)
		{
			std::cerr << "Some of the vertices in the selection has already been assigned as handle. Please select again." << std::endl;
			return;
		}
	}

	// new handle
	VectorX vertices(3*indices.size());
	for (unsigned int i = 0; i != indices.size(); ++i)
	{
		vertices.block<3,1>(i*3, 0) = m_mesh->m_current_positions.block<3,1>(indices[i] * 3, 0);
	}

	int id = m_handles.size();
	Handle handle = Handle(indices, vertices, color, id);
	// update the handle id list
	for (unsigned int i = 0; i != indices.size(); ++i)
	{
		m_handle_id[indices[i]] = id;
	}

	handle.attachment_constraints.clear();
	// new attachment constraints according to this handle
	for (unsigned int i = 0; i != handle.Indices().size(); ++i)
	{
		handle.attachment_constraints.push_back(AddAttachmentConstraint(handle.Indices()[i]));
	}
	m_handles.push_back(handle);
}

void Simulation::DeleteHandle()
{
	if (m_selected_handle_id >= 0)
	{
		// remove attachment constraints
		Handle& handle = m_handles[m_selected_handle_id];

		// TODO: remove attachment constraits
		std::vector<Constraint*>::iterator it;
		bool find_ac = false;
		for (it = m_constraints.begin(); it != m_constraints.end(); ++it)
		{
			if ((*it) == handle.attachment_constraints[0])
			{
				find_ac = true;
				break;
			}
		}
		if (find_ac == true)
		{
			m_constraints.erase(it, it + handle.attachment_constraints.size());
		}
		else
		{
			assert(find_ac == true);
		}
		//AddAttachmentConstraint(handle.Indices()[i]);

		// delete handle
		m_handles.erase(m_handles.begin() + m_selected_handle_id);

		// reconstruct the id of the handles
		for (unsigned int i = m_selected_handle_id; i < m_handles.size(); ++i)
		{
			m_handles[i].ID()--;
		}

		// reconstruct the handle id list
		for (unsigned int i = 0; i != m_handle_id.size(); ++i)
		{
			if (m_handle_id[i] == m_selected_handle_id)
			{
				m_handle_id[i] = -1; // unselected
			}
			else if (m_handle_id[i] > m_selected_handle_id)
			{
				m_handle_id[i] --;
			}
		}
		m_selected_handle_id = -1;
	}
}

bool Simulation::SelectHandle(std::vector<glm::vec3> ray)
{
	unsigned int nearest_handle_id = -1;
	float dist_min = 1e10;
	float dist;

	for (unsigned int i = 0; i != m_handles.size(); ++i)
	{
		if (m_handles[i].Select(ray, dist))
		{
			if (dist_min > dist)
			{
				dist_min = dist;
				nearest_handle_id = i;
			}
		}
	}
	m_selected_handle_id = nearest_handle_id;
	return (m_selected_handle_id == -1) ? false : true;
}

void Simulation::MoveHandleTemporary(const glm::vec3& trans)
{
	if (m_selected_handle_id >= 0)
	{
		m_handles[m_selected_handle_id].MoveTemporary(trans);
		UpdateHandleInfoToConstraints(m_handles[m_selected_handle_id]);
	}
}
void Simulation::MoveHandleFinalize()
{
	if (m_selected_handle_id >= 0)
	{
		m_handles[m_selected_handle_id].MoveFinalize();
		UpdateHandleInfoToConstraints(m_handles[m_selected_handle_id]);
	}
}
void Simulation::RotateHandleToValue()
{
	if (m_selected_handle_id >= 0)
	{
		ScalarType theta;
		std::cout << "Assign rotation(degree) for handle [" << m_selected_handle_id << "]." << std::endl;
		std::cin >> theta;
		theta = theta / 180.0 * 3.1415926;
		m_handles[m_selected_handle_id].RotateToValue(theta);
		UpdateHandleInfoToConstraints(m_handles[m_selected_handle_id]);
	}
}
void Simulation::RotateHandleSetStepSize()
{
	if (m_selected_handle_id >= 0)
	{
		//m_keyframe_handle_id = m_selected_handle_id;
		//std::cout << "Assign rotation(degree) for handle [" << m_selected_handle_id << "]." << std::endl;
		//std::cin >> m_keyframe_handle_unit_rotation;
		//m_keyframe_handle_unit_rotation = m_keyframe_handle_unit_rotation / 180.0 * 3.1415926;
		//UpdateHandleInfoToConstraints(m_handles[m_selected_handle_id]);
	}
}
void Simulation::RotateHandleTemporary(const glm::vec3& axis, const float& theta)
{
	if (m_selected_handle_id >= 0)
	{
		m_handles[m_selected_handle_id].RotateTemporary(axis, theta);
		UpdateHandleInfoToConstraints(m_handles[m_selected_handle_id]);
	}
}
void Simulation::RotateHandleFinalize()
{
	if (m_selected_handle_id >= 0)
	{
		m_handles[m_selected_handle_id].RotateFinalize();
		UpdateHandleInfoToConstraints(m_handles[m_selected_handle_id]);
	}
}

void Simulation::UpdateHandleInfoToConstraints(Handle& selected_handle)
{
	for (unsigned int i = 0; i != selected_handle.attachment_constraints.size(); ++i)
	{
		selected_handle.attachment_constraints[i]->SetFixedPoint(selected_handle[i]);
	}
}

glm::vec3 Simulation::SelectedHandleLocalCoM()
{
	assert(m_selected_handle_id >= 0);
	assert(m_selected_handle_id < m_handles.size());

	return m_handles[m_selected_handle_id].GetLocalCoM();
}

glm::vec3 Simulation::SelectedHandleCoM()
{
	assert(m_selected_handle_id >= 0);
	assert(m_selected_handle_id < m_handles.size());

	return m_handles[m_selected_handle_id].GetCoM();
}

void Simulation::SetHandleTranslationAnimation()
{
	if (m_selected_handle_id >= 0)
	{
		m_keyframe_handle_id_translation = m_selected_handle_id;
		m_keyframe_handle_unit_translation_axis.clear();
		m_keyframe_handle_unit_translation_amount.clear();
		m_keyframe_handle_unit_translation_end_frames.clear();

		// report handle
		std::cout << "Assignment Translation Animation for handle [" << m_selected_handle_id << "]." << std::endl;

		// set number of segments
		std::cout << "How many segments of translation do you want?" << std::endl;
		std::cin >> m_keyframe_handle_unit_translation_total_segments;

		int last_end_frame = 0;
		for (int i = 0; i != m_keyframe_handle_unit_translation_total_segments; i++)
		{
			EigenVector3 axis;
			ScalarType t;
			int end_frame;
			std::cout << "Input Translation Segment[" << i + 1 << "]." << std::endl;
			std::cout << "Assign translation axis: (will be normalized afterwards)" << std::endl;
			std::cin >> axis[0] >> axis[1] >> axis[2];
			std::cout << "Assign translation amount:" << std::endl;
			std::cin >> t;
			std::cout << "Assign this segment duration: (#frames)" << std::endl;
			std::cin >> end_frame;
			end_frame += last_end_frame;
			last_end_frame = end_frame;
			m_keyframe_handle_unit_translation_axis.push_back(axis);
			m_keyframe_handle_unit_translation_amount.push_back(t);
			m_keyframe_handle_unit_translation_end_frames.push_back(end_frame);
		}
	}
}
void Simulation::SetHandleTranslation()
{
	if (m_selected_handle_id >= 0)
	{
		EigenVector3 axis;
		ScalarType t;
		std::cout << "Assignment Translation for handle [" << m_selected_handle_id << "]." << std::endl;
		std::cout << "Assign translation axis: (will be normalized afterwards)" << std::endl;
		std::cin >> axis[0] >> axis[1] >> axis[2];
		std::cout << "Assign translation amount:" << std::endl;
		std::cin >> t;

		m_handles[m_selected_handle_id].ChangeTranslation(axis, t);
		UpdateHandleInfoToConstraints(m_handles[m_selected_handle_id]);
	}
}
void Simulation::SetHandleRotationAnimation()
{
	if (m_selected_handle_id >= 0)
	{
		m_keyframe_handle_id_rotation = m_selected_handle_id;
		m_keyframe_handle_unit_rotation_axis.clear();
		m_keyframe_handle_unit_rotation_degree.clear();
		m_keyframe_handle_unit_rotation_end_frames.clear();

		// report handle
		std::cout << "Assignment Rotation Animation for handle [" << m_selected_handle_id << "]." << std::endl;

		// set number of segments
		std::cout << "How many segments of rotation do you want?" << std::endl;
		std::cin >> m_keyframe_handle_unit_rotation_total_segments;

		int last_end_frame = 0;
		for (int i = 0; i != m_keyframe_handle_unit_rotation_total_segments; i++)
		{
			EigenVector3 axis;
			ScalarType t;
			int end_frame;
			std::cout << "Input Rotation Segment[" << i + 1 << "]." << std::endl;
			std::cout << "Assign rotation axis: (will be normalized afterwards)" << std::endl;
			std::cin >> axis[0] >> axis[1] >> axis[2];
			std::cout << "Assign rotation degree:" << std::endl;
			std::cin >> t;
			std::cout << "Assign this segment duration: (#frames)" << std::endl;
			std::cin >> end_frame;
			end_frame += last_end_frame;
			last_end_frame = end_frame;
			m_keyframe_handle_unit_rotation_axis.push_back(axis);
			m_keyframe_handle_unit_rotation_degree.push_back(t);
			m_keyframe_handle_unit_rotation_end_frames.push_back(end_frame);
		}
	}
}
void Simulation::SetHandleRotation()
{
	if (m_selected_handle_id >= 0)
	{
		EigenVector3 axis;
		ScalarType t;
		std::cout << "Assignment Rotation for handle [" << m_selected_handle_id << "]." << std::endl;
		std::cout << "Assign rotation axis: (will be normalized afterwards)" << std::endl;
		std::cin >> axis[0] >> axis[1] >> axis[2];
		std::cout << "Assign rotation degree:" << std::endl;
		std::cin >> t;

		m_handles[m_selected_handle_id].ChangeRotation(axis, t);
		UpdateHandleInfoToConstraints(m_handles[m_selected_handle_id]);
	}
}
void Simulation::AnimateHandle(const int current_frame)
{
	int segment = 0;
	// translation
	// find first segment;
	for (segment = 0; segment < m_keyframe_handle_unit_translation_total_segments /*This end condition should never be hit*/; segment++)
	{
		if (m_keyframe_handle_unit_translation_end_frames[segment] > current_frame)
		{
			break;
		}
	}
	if (segment == m_keyframe_handle_unit_translation_total_segments)
	{
		// do nothing 
	}
	else
	{
		if (m_keyframe_handle_unit_translation_amount[segment] > EPSILON)
		{
			m_handles[m_keyframe_handle_id_translation].ChangeTranslation(m_keyframe_handle_unit_translation_axis[segment], m_keyframe_handle_unit_translation_amount[segment]);
			UpdateHandleInfoToConstraints(m_handles[m_keyframe_handle_id_translation]);
		}
	}

	// rotation
	// find first segment;
	for (segment = 0; segment < m_keyframe_handle_unit_rotation_total_segments /*This end condition should never be hit*/; segment++)
	{
		if (m_keyframe_handle_unit_rotation_end_frames[segment] > current_frame)
		{
			break;
		}
	}
	if (segment == m_keyframe_handle_unit_rotation_total_segments)
	{
		// do nothing 
	}
	else
	{
		if (m_keyframe_handle_unit_rotation_degree[segment] > EPSILON)
		{

			m_handles[m_keyframe_handle_id_rotation].ChangeRotation(m_keyframe_handle_unit_rotation_axis[segment], m_keyframe_handle_unit_rotation_degree[segment]);
			UpdateHandleInfoToConstraints(m_handles[m_keyframe_handle_id_rotation]);
		}
	}

}

void Simulation::SaveHandleAnimation(const char* filename)
{
	std::ofstream outfile;
	outfile.open(filename, std::ifstream::out);
	if (outfile.is_open())
	{
		// TODO: change it to memory dump.
		outfile << "KeyframedHandleIDT   " << m_keyframe_handle_id_translation << std::endl << std::endl;
		outfile << "AnimationSegmentsT   " << m_keyframe_handle_unit_translation_total_segments << std::endl << std::endl;

		for (int i = 0; i != m_keyframe_handle_unit_translation_total_segments; ++i)
		{
			outfile	<< m_keyframe_handle_unit_translation_axis[i].x() << " " \
				<< m_keyframe_handle_unit_translation_axis[i].y() << " " \
				<< m_keyframe_handle_unit_translation_axis[i].z() << " " \
				<< m_keyframe_handle_unit_translation_amount[i] << " " \
				<< m_keyframe_handle_unit_translation_end_frames[i] <<	std::endl << std::endl;
		}

		outfile << "KeyframedHandleIDR   " << m_keyframe_handle_id_rotation << std::endl << std::endl;
		outfile << "AnimationSegmentsR   " << m_keyframe_handle_unit_rotation_total_segments << std::endl << std::endl;

		for (int i = 0; i != m_keyframe_handle_unit_rotation_total_segments; ++i)
		{
			outfile << m_keyframe_handle_unit_rotation_axis[i].x() << " " \
				<< m_keyframe_handle_unit_rotation_axis[i].y() << " " \
				<< m_keyframe_handle_unit_rotation_axis[i].z() << " " \
				<< m_keyframe_handle_unit_rotation_degree[i] << " " \
				<< m_keyframe_handle_unit_rotation_end_frames[i] << std::endl << std::endl;
		}

		outfile.close();
	}
	else
	{
		std::cerr << "Warning: Can not write handle animation file. Settings not saved." << std::endl;
	}
}
void Simulation::LoadHandleAnimation(const char* filename)
{
	// clear current handle animation info
	m_keyframe_handle_unit_translation_axis.clear();
	m_keyframe_handle_unit_translation_amount.clear();
	m_keyframe_handle_unit_translation_end_frames.clear();

	bool successfulRead = false;
	// read file
	std::ifstream infile;
	infile.open(filename, std::ifstream::in);
	if (successfulRead = infile.is_open())
	{
		char ignoreToken[256];

		infile >> ignoreToken >> m_keyframe_handle_id_translation;
		infile >> ignoreToken >> m_keyframe_handle_unit_translation_total_segments;

		for (int i = 0; i != m_keyframe_handle_unit_translation_total_segments; ++i)
		{
			EigenVector3 translation_axis;
			ScalarType translation_amount;
			int end_frame;
			infile >> translation_axis[0] \
				   >> translation_axis[1] \
				   >> translation_axis[2] \
				   >> translation_amount \
				   >> end_frame;

			m_keyframe_handle_unit_translation_axis.push_back(translation_axis);
			m_keyframe_handle_unit_translation_amount.push_back(translation_amount);
			m_keyframe_handle_unit_translation_end_frames.push_back(end_frame);
		}

		infile >> ignoreToken >> m_keyframe_handle_id_rotation;
		infile >> ignoreToken >> m_keyframe_handle_unit_rotation_total_segments;

		for (int i = 0; i != m_keyframe_handle_unit_rotation_total_segments; ++i)
		{
			EigenVector3 rotation_axis;
			ScalarType rotation_amount;
			int end_frame;
			infile >> rotation_axis[0] \
				>> rotation_axis[1] \
				>> rotation_axis[2] \
				>> rotation_amount \
				>> end_frame;

			m_keyframe_handle_unit_rotation_axis.push_back(rotation_axis);
			m_keyframe_handle_unit_rotation_degree.push_back(rotation_amount);
			m_keyframe_handle_unit_rotation_end_frames.push_back(end_frame);
		}
	}
	if (!successfulRead)
	{
		std::cerr << "Waning: failed loading handles animation." << std::endl;
	}
}
void Simulation::SaveHandles(const char* filename)
{
	unsigned int handle_num = m_handles.size();

	//const std::vector<unsigned int>& indices, const glm::vec3 color;
	std::ofstream outfile;
	outfile.open(filename, std::ifstream::out);
	if (outfile.is_open())
	{
		// TODO: change it to memory dump.
		outfile << "HandleNum   " << m_handles.size() << std::endl << std::endl;

		for (unsigned int i = 0; i != m_handles.size(); ++i)
		{
			outfile << "Color       "
				<< m_handles[i].Color().x << " "
				<< m_handles[i].Color().y << " "
				<< m_handles[i].Color().z << std::endl;
			//outfile << "Rotation2D  " << m_handles[i].RotationAngle2d() << std::endl;
			outfile << "RotationCenter "
				<< m_handles[i].CoM()[0] << " "
				<< m_handles[i].CoM()[1] << " " 
				<< m_handles[i].CoM()[2] << std::endl;
			outfile << "Translation "
				<< m_handles[i].Translation()[0] << " "
				<< m_handles[i].Translation()[1] << " " 
				<< m_handles[i].Translation()[2] << std::endl;
			EigenAngleAxis aa(m_handles[i].Rotation());
			outfile << "Rotation "
				<< aa.axis()[0] << " "
				<< aa.axis()[1] << " "
				<< aa.axis()[2] << " "
				<< aa.angle() << std::endl;
			outfile << "VerticesNum " << m_handles[i].Indices().size() << std::endl;
			for (unsigned int j = 0; j != m_handles[i].Indices().size(); ++j)
			{
				outfile << m_handles[i].Indices()[j] << " ";
			}
			outfile << std::endl << std::endl;
		}

		outfile.close();
	}
	else
	{
		std::cerr << "Warning: Can not write handle file. Settings not saved." << std::endl;
	}
}

void Simulation::LoadHandles(const char* filename)
{
	// clear current handles and attachment constraints
	for (int i = m_handles.size() - 1; i >= 0; --i)
	{
		m_selected_handle_id = i;
		DeleteHandle();
	}
	m_handles.clear();
	for (unsigned int i = 0; i != m_handle_id.size(); ++i)
	{
		m_handle_id[i] = -1;
	}

	bool successfulRead = false;
	// read file
	std::ifstream infile;
	infile.open(filename, std::ifstream::in);
	if (successfulRead = infile.is_open())
	{
		char ignoreToken[256];

		unsigned int handle_num;
		infile >> ignoreToken >> handle_num;

		unsigned int handle_size;
		glm::vec3 color;
		std::vector<unsigned int> ids;

		EigenVector3 translation;
		EigenVector3 com;
		EigenVector3 rotation_axis;
		ScalarType rotation_angle;
		for (unsigned int hi = 0; hi != handle_num; ++hi)
		{
			infile >> ignoreToken >> color[0] >> color[1] >> color[2];
			//infile >> ignoreToken >> rotation2d;
			infile >> ignoreToken >> com[0] >> com[1] >> com[2];
			infile >> ignoreToken >> translation[0] >> translation[1] >> translation[2];
			infile >> ignoreToken >> rotation_axis[0] >> rotation_axis[1] >> rotation_axis[2] >> rotation_angle;
			infile >> ignoreToken >> handle_size;
			ids.resize(handle_size);
			for (unsigned int j = 0; j != handle_size; ++j)
			{
				infile >> ids[j];
			}
			VectorX vertices(3*ids.size());
			for (unsigned int i = 0; i != ids.size(); ++i)
			{
				vertices.block<3,1>(i*3, 0) = m_mesh->m_restpose_positions.block<3,1>(3*ids[i], 0).transpose();
			}
			Handle h(ids, vertices, color, hi);
			//h.RotationAngle2d() = rotation2d;
			h.Translation() = translation;
			h.Rotation() = EigenAngleAxis(rotation_angle, rotation_axis).matrix();
			h.CoM() = com;
			h.Update();

			h.attachment_constraints.clear();
			// new attachment constraints according to this handle
			for (unsigned int i = 0; i != h.Indices().size(); ++i)
			{
				h.attachment_constraints.push_back(AddAttachmentConstraint(h.Indices()[i], h[i]));
			}

			m_handles.push_back(h);

			// update handle id
			for (unsigned int i = 0; i != h.Indices().size(); ++i)
			{
				m_handle_id[h.Indices()[i]] = h.ID();
			}
		}
	}
	if (!successfulRead)
	{
		std::cerr << "Waning: failed loading handles." << std::endl;
	}
}

void Simulation::ResetHandles()
{
	for (unsigned int i = 0; i != m_handles.size(); i++)
	{
		m_handles[i].Reset();
		UpdateHandleInfoToConstraints(m_handles[i]);
	}
}

void Simulation::SaveSparseMatrix(const SparseMatrix& A, const char* filename)
{
	std::ofstream outfile;
	outfile.open(filename, std::ifstream::out);
	if (outfile.is_open())
	{
		std::vector<SparseMatrixTriplet> A_triplets;
		EigenSparseMatrixToTriplets(A, A_triplets);

		// my format
		outfile << A.rows() << " " << A.cols() << " " << A_triplets.size() << std::endl;
		for (unsigned int i = 0; i != A_triplets.size(); i++)
		{
			SparseMatrixTriplet& Ai = A_triplets[i];
			outfile << Ai.row() << " " << Ai.col() << " " << Ai.value() << std::endl;
		}

		//// cuSolver format
		//outfile << "%%MatrixMarket matrix coordinate real symmetric\n% Generated 20 - Nov - 2014" << std::endl;
		//outfile << m_mass_plus_h2_weighted_laplacian.rows() << " " << m_mass_plus_h2_weighted_laplacian.cols() << " " << (L_triplets.size()+m_mass_plus_h2_weighted_laplacian.rows())/2 << std::endl;
		//for (unsigned int i = 0; i != L_triplets.size(); i++)
		//{
		//	SparseMatrixTriplet& Li = L_triplets[i];
		//	if (Li.row() >= Li.col())
		//	{
		//		outfile << Li.row() + 1 << " " << Li.col() + 1 << " " << Li.value() << std::endl;
		//	}
		//}

		outfile.close();
	}
}

void Simulation::SaveLaplacianMatrix(const char * filename)
{
	SaveSparseMatrix(m_weighted_laplacian, filename);
	//std::ofstream outfile;
	//outfile.open(filename, std::ifstream::out);
	//if (outfile.is_open())
	//{
	//	std::vector<SparseMatrixTriplet> L_triplets;
	//	EigenSparseMatrixToTriplets(m_weighted_laplacian, L_triplets);

	//	// my format
	//	outfile << m_weighted_laplacian.rows() << " " << m_weighted_laplacian.cols() << " " << L_triplets.size() << std::endl;
	//	for (unsigned int i = 0; i != L_triplets.size(); i++)
	//	{
	//		SparseMatrixTriplet& Li = L_triplets[i];
	//		outfile << Li.row() << " " << Li.col() << " " << Li.value() << std::endl;
	//	}

	//	outfile.close();
	//}
}

void Simulation::SetConvergedEnergy()
{
#ifdef ENABLE_MATLAB_DEBUGGING
	ScalarType energy = evaluateEnergy(m_mesh->m_current_positions);
	g_debugger->SetConvergedEnergy(energy);
#endif // ENABLE_MATLAB_DEBUGGING
}

void Simulation::RandomizePoints()
{
	VectorX x;
	generateRandomVector(m_mesh->m_system_dimension, x);

	m_mesh->m_current_positions = x;
}

// set material property for selected elements
void Simulation::SetMaterialProperty(std::vector<Constraint*>& constraints)
{
	for (std::vector<Constraint*>::iterator it = constraints.begin(); it != constraints.end(); ++it)
	{
		switch ((*it)->Type())
		{
		case CONSTRAINT_TYPE_TET:
			(*it)->SetMaterialProperty(m_material_type, m_stiffness_stretch, m_stiffness_bending, m_stiffness_kappa, m_stiffness_laplacian);
			break;
		case CONSTRAINT_TYPE_SPRING:
			(*it)->SetMaterialProperty(m_stiffness_stretch);
			break;
		case CONSTRAINT_TYPE_SPRING_BENDING:
			(*it)->SetMaterialProperty(m_stiffness_bending);
			break;
		case CONSTRAINT_TYPE_ATTACHMENT:
			(*it)->SetMaterialProperty(m_stiffness_attachment);
			break;
		}
	}
	SetReprefactorFlag();
}
void Simulation::SetMaterialProperty(std::vector<Constraint*>& constraints, MaterialType type, ScalarType stretch, ScalarType bending, ScalarType kappa, ScalarType laplacian_coeff)
{
	for (std::vector<Constraint*>::iterator it = constraints.begin(); it != constraints.end(); ++it)
	{
		switch ((*it)->Type())
		{
		case CONSTRAINT_TYPE_TET:
			(*it)->SetMaterialProperty(type, stretch, bending, kappa, laplacian_coeff);
			break;
		case CONSTRAINT_TYPE_SPRING:
			(*it)->SetMaterialProperty(stretch);
			break;
		}
	}
	SetReprefactorFlag();
}

// set material property for all elements
void Simulation::SetMaterialProperty()
{
	SetMaterialProperty(m_constraints);
}

void Simulation::GetPartialMaterialProperty()
{
	if (!m_selected_constraints.empty())
	{
		Constraint* c = m_selected_constraints[0];
		if (c->Type() == CONSTRAINT_TYPE_SPRING)
		{
			c->GetMaterialProperty(m_partial_stiffness_stretch);
		}
		else if (c->Type() == CONSTRAINT_TYPE_TET)
		{
			c->GetMaterialProperty(m_partial_material_type, m_partial_stiffness_stretch, m_partial_stiffness_bending, m_partial_stiffness_kappa);
		}
	}
}
void Simulation::SetPartialMaterialProperty()
{
	if (!m_selected_constraints.empty())
	{
		SetMaterialProperty(m_selected_constraints, m_partial_material_type, m_partial_stiffness_stretch, m_partial_stiffness_bending, m_partial_stiffness_kappa, 2*m_partial_stiffness_stretch+m_partial_stiffness_bending);
	}

}
void Simulation::SavePerConstraintMaterialProperties(const char* filename)
{
	std::ofstream outfile;
	outfile.open(filename, std::ifstream::out);
	if (outfile.is_open())
	{
		MaterialType material_type;
		ScalarType stiffness, mu, lambda, kappa;
		for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
		{
			if ((*c)->Type() == CONSTRAINT_TYPE_TET)
			{
				(*c)->GetMaterialProperty(material_type, mu, lambda, kappa);
				outfile << material_type << " " << mu << " " << lambda << " " << kappa << std::endl;
			}
			else
			{
				(*c)->GetMaterialProperty(stiffness);
				outfile << stiffness << std::endl;
			}
		}

		outfile.close();
	}

}
void Simulation::LoadPerConstraintMaterialProperties(const char* filename)
{
	std::ifstream infile;
	infile.open(filename, std::ifstream::in);
	char ignore[256];
	if (infile.is_open())
	{
		MaterialType material_type;
		ScalarType stiffness, mu, lambda, kappa;
		int temp_enum;
		for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
		{
			if ((*c)->Type() == CONSTRAINT_TYPE_TET)
			{
				infile >> temp_enum; material_type = MaterialType(temp_enum);
				infile >> mu;
				infile >> lambda;
				infile >> kappa;
				(*c)->SetMaterialProperty(material_type, mu, lambda, kappa, 2*mu+lambda);
			}
			else
			{
				infile >> stiffness;
				(*c)->SetMaterialProperty(stiffness);
			}
		}

		infile.close();
	}
}
void Simulation::SelectTetConstraints(const std::vector<unsigned int>& indices)
{
	m_selected_constraints.clear();
	for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); it++)
	{
		if ((*it)->Type() == CONSTRAINT_TYPE_SPRING || (*it)->Type() == CONSTRAINT_TYPE_TET)
		{
			for (unsigned int i = 0; i != indices.size(); i++)
			{
				if ((*it)->VertexIncluded(indices[i]))
				{
					m_selected_constraints.push_back(*it);
					break;
				}
			}
		}
	}
}

// eigen value visualization mesh
void Simulation::NewVisualizationMesh()
{
	if (m_mesh->GetMeshType() == MESH_TYPE_CLOTH)
	{
		m_eigenvector_vis_mesh = new ClothMesh();
	}
	else
	{
		m_eigenvector_vis_mesh = new TetMesh();
	}
}
void Simulation::DeleteVisualizationMesh()
{
	if (m_eigenvector_vis_mesh != NULL)
	{
		delete m_eigenvector_vis_mesh;
	}
}
void Simulation::ResetVisualizationMesh()
{
	DeleteVisualizationMesh();
	NewVisualizationMesh();
}
void Simulation::SetVisualizationMesh()
{
	m_eigenvector_vis_mesh->CopyFromClothMesh(m_mesh);
}
void Simulation::ResetVisualizationMeshHeight()
{
	if (m_eigenvector_vis_mesh->GetMeshType() == MESH_TYPE_CLOTH)
	{
		VectorX& x = m_eigenvector_vis_mesh->m_current_positions;

		for (unsigned int i = 0; 3 * i != x.size(); i++)
		{
			// set y component of x to 0
			x(3 * i + 1) = 0;
		}
		m_eigenvector_vis_mesh->Update();
	}
}

void Simulation::clearConstraints()
{
	for (unsigned int i = 0; i < m_constraints.size(); ++i)
	{
		delete m_constraints[i];
	}
	m_constraints.clear();
}

void Simulation::setupConstraints()
{
	clearConstraints();

	m_stiffness_high = 1e5;

	switch(m_mesh->m_mesh_type)
	{
	case MESH_TYPE_CLOTH:
		// procedurally generate constraints including to attachment constraints
		{
			// generate stretch constraints. assign a stretch constraint for each edge.
			EigenVector3 p1, p2;
			for(std::vector<Edge>::iterator e = m_mesh->m_edge_list.begin(); e != m_mesh->m_edge_list.end(); ++e)
			{
				p1 = m_mesh->m_current_positions.block_vector(e->m_v1);
				p2 = m_mesh->m_current_positions.block_vector(e->m_v2);
				SpringConstraint* c;
				//if (e - m_mesh->m_edge_list.begin() < 100)
				//{
				//	c = new SpringConstraint(&m_stiffness_high, e->m_v1, e->m_v2, (p1 - p2).norm());
				//}
				//else
				{
					ScalarType rest_length = (p1 - p2).norm();
					rest_length *= rest_length_adjust;
					c = new SpringConstraint(e->m_v1, e->m_v2, rest_length);
				}
				m_constraints.push_back(c);
				m_mesh->m_expanded_system_dimension+=6;
				m_mesh->m_expanded_system_dimension_1d+=2;
			}

			// generate bending constraints. naive
			unsigned int i, k;
			for(i = 0; i < m_mesh->m_dim[0]; ++i)
			{
				for(k = 0; k < m_mesh->m_dim[1]; ++k)
				{
					unsigned int index_self = m_mesh->m_dim[1] * i + k;
					p1 = m_mesh->m_current_positions.block_vector(index_self);
					if (i+2 < m_mesh->m_dim[0])
					{
						unsigned int index_row_1 = m_mesh->m_dim[1] * (i + 2) + k;
						p2 = m_mesh->m_current_positions.block_vector(index_row_1);
						ScalarType rest_length = (p1 - p2).norm();
						rest_length *= rest_length_adjust;
						SpringConstraint* c = new SpringConstraint(CONSTRAINT_TYPE_SPRING_BENDING, index_self, index_row_1, rest_length);
						m_constraints.push_back(c);
						m_mesh->m_expanded_system_dimension+=6;
						m_mesh->m_expanded_system_dimension_1d+=2;
					}
					if (k+2 < m_mesh->m_dim[1])
					{
						unsigned int index_column_1 = m_mesh->m_dim[1] * i + k + 2;
						p2 = m_mesh->m_current_positions.block_vector(index_column_1);
						ScalarType rest_length = (p1 - p2).norm();
						rest_length *= rest_length_adjust;
						SpringConstraint* c = new SpringConstraint(CONSTRAINT_TYPE_SPRING_BENDING, index_self, index_column_1, rest_length);
						m_constraints.push_back(c);
						m_mesh->m_expanded_system_dimension+=6;
						m_mesh->m_expanded_system_dimension_1d+=2;
					}
				}
			}

			// generating attachment constraints.
			//std::vector<unsigned int> handle_1_indices; handle_1_indices.clear(); handle_1_indices.push_back(0);
			//std::vector<unsigned int> handle_2_indices; handle_2_indices.clear(); handle_2_indices.push_back(m_mesh->m_dim[1] * (m_mesh->m_dim[0] - 1));
			NewHandle({ 0 }, glm::vec3(1.0, 0.0, 0.0));
			NewHandle({ m_mesh->m_dim[1] * (m_mesh->m_dim[0] - 1) }, glm::vec3(1.0, 0.0, 0.0));
			//AddAttachmentConstraint(0);
			//AddAttachmentConstraint(m_mesh->m_dim[1]*(m_mesh->m_dim[0]-1));
		}
		break;
	case MESH_TYPE_TET:
		{
			//// generate stretch constraints. assign a stretch constraint for each edge.
			//EigenVector3 p1, p2;
			//for(std::vector<Edge>::iterator e = m_mesh->m_edge_list.begin(); e != m_mesh->m_edge_list.end(); ++e)
			//{
			//	p1 = m_mesh->m_current_positions.block_vector(e->m_v1);
			//	p2 = m_mesh->m_current_positions.block_vector(e->m_v2);
			//	SpringConstraint *c = new SpringConstraint(&m_stiffness_stretch, e->m_v1, e->m_v2, (p1-p2).norm());
			//	m_constraints.push_back(c);
			//}

			// reset mass matrix for tet simulation:
			ScalarType total_volume = 0;
			std::vector<SparseMatrixTriplet> mass_triplets;
			std::vector<SparseMatrixTriplet> mass_1d_triplets;
			mass_triplets.clear();
			mass_1d_triplets.clear();

			VectorX& x = m_mesh->m_current_positions;
			TetMesh* tet_mesh = dynamic_cast<TetMesh*>(m_mesh);
			for (unsigned int i = 0; i < tet_mesh->m_loaded_mesh->m_tets.size(); ++i)
			{
				MeshLoader::Tet& tet = tet_mesh->m_loaded_mesh->m_tets[i];
				TetConstraint *c = new TetConstraint(tet.id1, tet.id2, tet.id3, tet.id4, x);
				m_constraints.push_back(c);

				total_volume += c->SetMassMatrix(mass_triplets, mass_1d_triplets);

				m_mesh->m_expanded_system_dimension+=9;
				m_mesh->m_expanded_system_dimension_1d+=3;
			}

			m_mesh->m_mass_matrix.setFromTriplets(mass_triplets.begin(), mass_triplets.end());
			m_mesh->m_mass_matrix_1d.setFromTriplets(mass_1d_triplets.begin(), mass_1d_triplets.end());

			m_mesh->m_mass_matrix = m_mesh->m_mass_matrix * (m_mesh->m_total_mass / total_volume);
			m_mesh->m_mass_matrix_1d = m_mesh->m_mass_matrix_1d * (m_mesh->m_total_mass / total_volume);

			std::vector<SparseMatrixTriplet> mass_inv_triplets;
			mass_inv_triplets.clear();
			std::vector<SparseMatrixTriplet> mass_inv_1d_triplets;
			mass_inv_1d_triplets.clear();
			for (unsigned int i = 0; i != m_mesh->m_mass_matrix.rows(); i++)
			{
				ScalarType mi = m_mesh->m_mass_matrix.coeff(i, i);
				ScalarType mi_inv;
				if (std::abs(mi) > 1e-12)
				{
					mi_inv = 1.0 / mi;
				}
				else
				{
					// ugly ugly!
					m_mesh->m_mass_matrix.coeffRef(i, i) = 1e-12;
					mi_inv = 1e12;
				}
				mass_inv_triplets.push_back(SparseMatrixTriplet(i, i, mi_inv));
			}
			for (unsigned int i = 0; i != m_mesh->m_mass_matrix_1d.rows(); i++)
			{
				ScalarType mi = m_mesh->m_mass_matrix_1d.coeff(i, i);
				ScalarType mi_inv;
				if (std::abs(mi) > 1e-12)
				{
					mi_inv = 1.0 / mi;
				}
				else
				{
					// ugly ugly!
					m_mesh->m_mass_matrix_1d.coeffRef(i, i) = 1e-12;
					mi_inv = 1e12;
				}
				mass_inv_1d_triplets.push_back(SparseMatrixTriplet(i, i, mi_inv));
			}

			m_mesh->m_inv_mass_matrix.setFromTriplets(mass_inv_triplets.begin(), mass_inv_triplets.end());
			m_mesh->m_inv_mass_matrix_1d.setFromTriplets(mass_inv_1d_triplets.begin(), mass_inv_1d_triplets.end());

#ifdef ENABLE_MATLAB_DEBUGGING
			g_debugger->SendSparseMatrix(m_mesh->m_mass_matrix, "M");
			g_debugger->SendSparseMatrix(m_mesh->m_inv_mass_matrix, "M_inv");
			g_debugger->SendSparseMatrix(m_mesh->m_mass_matrix_1d, "M_1d");
#endif
	}
		break;
	}
}

void Simulation::dampVelocity()
{
	if (std::abs(m_damping_coefficient) < EPSILON)
		return;

	m_mesh->m_current_velocities *= 1-m_damping_coefficient;

	//// post-processing damping
	//EigenVector3 pos_mc(0.0, 0.0, 0.0), vel_mc(0.0, 0.0, 0.0);
	//unsigned int i, size;
	//ScalarType denominator(0.0), mass(0.0);
	//size = m_mesh->m_vertices_number;
	//for(i = 0; i < size; ++i)
	//{
	//	mass = m_mesh->m_mass_matrix.coeff(i*3, i*3);

	//	pos_mc += mass * m_mesh->m_current_positions.block_vector(i);
	//	vel_mc += mass * m_mesh->m_current_velocities.block_vector(i);
	//	denominator += mass;
	//}
	//assert(denominator != 0.0);
	//pos_mc /= denominator;
	//vel_mc /= denominator;

	//EigenVector3 angular_momentum(0.0, 0.0, 0.0), r(0.0, 0.0, 0.0);
	//EigenMatrix3 inertia, r_mat;
	//inertia.setZero(); r_mat.setZero();

	//for(i = 0; i < size; ++i)
	//{
	//	mass = m_mesh->m_mass_matrix.coeff(i*3, i*3);

	//	r = m_mesh->m_current_positions.block_vector(i) - pos_mc;
	//	angular_momentum += r.cross(mass * m_mesh->m_current_velocities.block_vector(i));

	//	//r_mat = EigenMatrix3(0.0,  r.z, -r.y,
	//	//					-r.z, 0.0,  r.x,
	//	//					r.y, -r.x, 0.0);

	//	r_mat.coeffRef(0, 1) = r[2];
	//	r_mat.coeffRef(0, 2) = -r[1];
	//	r_mat.coeffRef(1, 0) = -r[2];
	//	r_mat.coeffRef(1, 2) = r[0];
	//	r_mat.coeffRef(2, 0) = r[1];
	//	r_mat.coeffRef(2, 1) = -r[0];

	//	inertia += r_mat * r_mat.transpose() * mass;
	//}
	//EigenVector3 angular_vel = inertia.inverse() * angular_momentum;

	//EigenVector3 delta_v(0.0, 0.0, 0.0);
	//for(i = 0; i < size; ++i)
	//{
	//	r = m_mesh->m_current_positions.block_vector(i) - pos_mc;
	//	delta_v = vel_mc + angular_vel.cross(r) - m_mesh->m_current_velocities.block_vector(i);     
	//	m_mesh->m_current_velocities.block_vector(i) += m_damping_coefficient * delta_v;
	//}
}

void Simulation::calculateExternalForce()
{
	m_external_force.resize(m_mesh->m_system_dimension);
	m_external_force.setZero();

	// gravity
	for (unsigned int i = 0; i < m_mesh->m_vertices_number; ++i)
	{
		m_external_force[3*i+1] += -m_gravity_constant;
	}

#ifdef ENABLE_MATLAB_DEBUGGING
	g_debugger->SendSparseMatrix(m_mesh->m_mass_matrix, "M");
#endif
	m_external_force = m_mesh->m_mass_matrix * m_external_force;
}

VectorX Simulation::collisionDetectionPostProcessing(const VectorX& x)
{
	// Naive implementation of collision detection
	VectorX penetration(m_mesh->m_system_dimension);
	penetration.setZero();
	EigenVector3 normal;
	ScalarType dist;

	for (unsigned int i = 0; i != m_mesh->m_vertices_number; ++i)
	{
		EigenVector3 xi = x.block_vector(i);

		if (m_scene->StaticIntersectionTest(xi, normal, dist))
		{
			penetration.block_vector(i) += (dist) * normal;
		}
	}

	return penetration;
}

void Simulation::collisionDetection(const VectorX& x)
{
	if (!m_scene->IsEmpty())
	{
		m_collision_constraints.clear();

		EigenVector3 surface_point;
		EigenVector3 normal;
		ScalarType dist;

		for (unsigned int i = 0; i != m_mesh->m_vertices_number; ++i)
		{
			EigenVector3 xi = x.block_vector(i);

			if (m_scene->StaticIntersectionTest(xi, normal, dist))
			{
				surface_point = xi - normal*dist; // dist is negative...
				m_collision_constraints.push_back(CollisionSpringConstraint(1e3, i, surface_point, normal));
			}
		}
	}
}

void Simulation::collisionResolution(const VectorX& penetration, VectorX& x, VectorX& v)
{
	EigenVector3 xi, vi, pi, ni;
	EigenVector3 vin, vit;
	for (unsigned int i = 0; i != m_mesh->m_vertices_number; ++i)
	{
		xi = x.block_vector(i);
		vi = v.block_vector(i);
		pi = penetration.block_vector(i);

		ScalarType dist = pi.norm();
		if (dist > EPSILON) // there is collision
		{
			ni = -pi / dist; // normalize
			xi -= pi;
			vin = vi.dot(ni)*ni;
			vit = vi - vin;
			vi = -(m_restitution_coefficient)*vin + (1-m_friction_coefficient) * vit;
			x.block_vector(i) = xi;
			v.block_vector(i) = vi;
		}
	}
}

void Simulation::integrateImplicitMethod()
{
	// take a initial guess
	VectorX x = m_y;
	//VectorX x = m_mesh->m_current_positions;

	// init method specific constants
	// for l-bfgs only
	if (m_lbfgs_restart_every_frame == true)
	{
		m_lbfgs_need_update_H0 = true;
	}
	EigenMatrixx3 x_nx3(x.size()/3, 3);

	ScalarType total_time = 1e-5;
	if (m_step_mode)
	{
#ifdef ENABLE_MATLAB_DEBUGGING
		ScalarType energy = evaluateEnergy(x);
		VectorX gradient;
		evaluateGradient(x, gradient);
		ScalarType gradient_norm = gradient.norm();
		g_debugger->SendData(x, energy, gradient_norm, 0, total_time);
#endif // ENABLE_MATLAB_DEBUGGING
	}

	TimerWrapper t_optimization;
	t_optimization.Tic();
	g_lbfgs_timer.Tic();
	g_lbfgs_timer.Pause();
	// while loop until converge or exceeds maximum iterations
	bool converge = false;
	m_ls_is_first_iteration = true;
	for (m_current_iteration = 0; !converge && m_current_iteration < m_iterations_per_frame; ++m_current_iteration)
	{
		if (m_processing_collision)
		{
			// Collision Detection every iteration
			collisionDetection(x);
		}

		g_integration_timer.Tic();
		switch (m_optimization_method)
		{
		case OPTIMIZATION_METHOD_GRADIENT_DESCENT:
			converge = performGradientDescentOneIteration(x);
			break;
		case OPTIMIZATION_METHOD_NEWTON:
			converge = performNewtonsMethodOneIteration(x);
			break;
		case OPTIMIZATION_METHOD_LBFGS:
			converge = performLBFGSOneIteration(x);
			break;
		default:
			break;
		}
		m_ls_is_first_iteration = false;
		g_integration_timer.Toc();

		if (m_verbose_show_converge)
		{
			if (converge && m_current_iteration != 0)
			{
				std::cout << "Optimization Converged in iteration #" << m_current_iteration << std::endl;
			}
		}

		if (m_step_mode)
		{
#ifdef ENABLE_MATLAB_DEBUGGING
			ScalarType energy = evaluateEnergy(x);
			VectorX gradient;
			evaluateGradient(x, gradient);
			ScalarType gradient_norm = gradient.norm();
			total_time += g_integration_timer.DurationInSeconds();
			g_debugger->SendData(x, energy, gradient_norm, m_current_iteration + 1, total_time);
#endif // ENABLE_MATLAB_DEBUGGING
		}
	}

	t_optimization.Toc();
	t_optimization.Report("Optimization", m_verbose_show_optimization_time);
	g_lbfgs_timer.Resume();
	g_lbfgs_timer.TocAndReport("L-BFGS overhead", m_verbose_show_converge, TIMER_OUTPUT_MILLISECONDS);

	// update constants
	updatePosAndVel(x);
}

bool Simulation::performGradientDescentOneIteration(VectorX& x)
{
	// evaluate gradient direction
	VectorX gradient;
	evaluateGradient(x, gradient);

#ifdef ENABLE_MATLAB_DEBUGGING
	g_debugger->SendVector(gradient, "g");
#endif

	if (gradient.norm() < EPSILON)
		return true;

	// assign descent direction
	//VectorX descent_dir = -m_mesh->m_inv_mass_matrix*gradient;
	VectorX descent_dir = -gradient;

	// line search
	ScalarType step_size = lineSearch(x, gradient, descent_dir);

	// update x
	x = x + descent_dir * step_size;

	// report convergence
	if (step_size < EPSILON)
		return true;
	else
		return false;
}

bool Simulation::performNewtonsMethodOneIteration(VectorX& x)
{
	TimerWrapper timer; timer.Tic();
	// evaluate gradient direction
	VectorX gradient;
	evaluateGradient(x, gradient, true);
	//QSEvaluateGradient(x, gradient, m_ss->m_quasi_static);
#ifdef ENABLE_MATLAB_DEBUGGING
	g_debugger->SendVector(gradient, "g");
#endif

	timer.TocAndReport("evaluate gradient", m_verbose_show_converge);
	timer.Tic();

	// evaluate hessian matrix
	SparseMatrix hessian_1;
	evaluateHessian(x, hessian_1);
	//SparseMatrix hessian_2;
	//evaluateHessianSmart(x, hessian_2);

	SparseMatrix& hessian = hessian_1;

#ifdef ENABLE_MATLAB_DEBUGGING
	g_debugger->SendSparseMatrix(hessian_1, "H");
	//g_debugger->SendSparseMatrix(hessian_2, "H2");
#endif

	timer.TocAndReport("evaluate hessian", m_verbose_show_converge);
	timer.Tic();
	VectorX descent_dir;

	linearSolve(descent_dir, hessian, gradient);
	descent_dir = -descent_dir;

	timer.TocAndReport("solve time", m_verbose_show_converge);
	timer.Tic();

	// line search
	ScalarType step_size = lineSearch(x, gradient, descent_dir);
	//if (step_size < EPSILON)
	//{
	//	std::cout << "correct step size to 1" << std::endl;
	//	step_size = 1;
	//}
	// update x
	x = x + descent_dir * step_size;

	//if (step_size < EPSILON)
	//{
	//	printVolumeTesting(x);
	//}

	timer.TocAndReport("line search", m_verbose_show_converge);
	//timer.Toc();
	//std::cout << "newton: " << timer.Duration() << std::endl;

	if (-descent_dir.dot(gradient) < EPSILON_SQUARE)
		return true;
	else
		return false;
}

bool Simulation::performLBFGSOneIteration(VectorX& x)
{
	bool converged = false;
	ScalarType current_energy;
	VectorX gf_k;

	// set xk and gfk
	if (m_ls_is_first_iteration || !m_enable_line_search)
	{
		current_energy = evaluateEnergyAndGradient(x, gf_k);
	}
	else
	{
		current_energy = m_ls_prefetched_energy;
		gf_k = m_ls_prefetched_gradient;
	}
	//current_energy = evaluateEnergyAndGradient(x, gf_k);

	if (m_lbfgs_need_update_H0) // first iteration
	{
		// clear sk and yk and alpha_k
#ifdef USE_STL_QUEUE_IMPLEMENTATION
		// stl implementation
		m_lbfgs_y_queue.clear();
		m_lbfgs_s_queue.clear();
#else
		// my implementation
		delete m_lbfgs_queue;
		m_lbfgs_queue = new QueueLBFGS(x.size(), m_lbfgs_m);
#endif

		// decide H0 and it's factorization precomputation
		switch (m_lbfgs_H0_type)
		{
		case LBFGS_H0_LAPLACIAN:
			prefactorize();
			break;
		default:
			//prefactorize();
			break;
		}

		g_lbfgs_timer.Resume();
		// store them before wipeout
		m_lbfgs_last_x = x;
		m_lbfgs_last_gradient = gf_k;

		g_lbfgs_timer.Pause();
		// first iteration
		VectorX r;
		LBFGSKernelLinearSolve(r, gf_k, 1);
		g_lbfgs_timer.Resume();

		// update
		VectorX p_k = -r;
		g_lbfgs_timer.Pause();

		if (-p_k.dot(gf_k) < EPSILON_SQUARE || p_k.norm() / x.norm() < LARGER_EPSILON)
		{
			converged = true;
		}

		ScalarType alpha_k = linesearchWithPrefetchedEnergyAndGradientComputing(x, current_energy, gf_k, p_k, m_ls_prefetched_energy, m_ls_prefetched_gradient);
		x += alpha_k * p_k;

		// final touch
		m_lbfgs_need_update_H0 = false;
	}
	else // otherwise
	{
		TimerWrapper t_local;
		TimerWrapper t_global;
		TimerWrapper t_linesearch;
		TimerWrapper t_other;
		bool verbose = false;

		t_other.Tic();
		g_lbfgs_timer.Resume();
		// enqueue stuff
		VectorX s_k = x - m_lbfgs_last_x;
		VectorX y_k = gf_k - m_lbfgs_last_gradient;

#ifdef USE_STL_QUEUE_IMPLEMENTATION
		//stl implementation
		if (m_lbfgs_s_queue.size() > m_lbfgs_m)
		{
			m_lbfgs_s_queue.pop_back();
			m_lbfgs_y_queue.pop_back();
		}
		// enqueue stuff
		m_lbfgs_s_queue.push_front(s_k);
		m_lbfgs_y_queue.push_front(y_k);

		int m_queue_size = m_lbfgs_s_queue.size();
#else
		// my implementation
		if (m_lbfgs_queue->isFull())
		{
			m_lbfgs_queue->dequeue();
		}
		m_lbfgs_queue->enqueue(s_k, y_k);

		int m_queue_size = m_lbfgs_queue->size();
#endif

		// store them before wipeout
		m_lbfgs_last_x = x;
		m_lbfgs_last_gradient = gf_k;
		VectorX q = gf_k;

		// loop 1 of l-BFGS
		std::vector<ScalarType> rho;
		rho.clear();
		std::vector<ScalarType> alpha;
		alpha.clear();
		int m_queue_visit_upper_bound = (m_lbfgs_m < m_queue_size) ? m_lbfgs_m : m_queue_size;
		ScalarType* s_i = NULL;
		ScalarType* y_i = NULL;
		for (int i = 0; i != m_queue_visit_upper_bound; i++)
		{
#ifdef USE_STL_QUEUE_IMPLEMENTATION
			// stl implementation
			ScalarType yi_dot_si = m_lbfgs_y_queue[i].dot(m_lbfgs_s_queue[i]);
			if (yi_dot_si < EPSILON_SQUARE)
			{
				return true;
			}
			ScalarType rho_i = 1.0 / yi_dot_si;
			rho.push_back(rho_i);
			alpha.push_back(rho[i]*m_lbfgs_s_queue[i].dot(q));
			q = q - alpha[i] * m_lbfgs_y_queue[i];
#else
			// my implementation
			m_lbfgs_queue->visitSandY(&s_i, &y_i, i);
			Eigen::Map<const VectorX> s_i_eigen(s_i, x.size());
			Eigen::Map<const VectorX> y_i_eigen(y_i, x.size());
			ScalarType yi_dot_si = (y_i_eigen.dot(s_i_eigen));
			if (yi_dot_si < EPSILON_SQUARE)
			{
				return true;
			}
			ScalarType rho_i = 1.0 / yi_dot_si;
			rho.push_back(rho_i);
			ScalarType alpha_i = rho_i * s_i_eigen.dot(q);
			alpha.push_back(alpha_i);
			q -= alpha_i * y_i_eigen;
#endif
		}
		// compute H0 * q
		g_lbfgs_timer.Pause();
		t_other.Pause();
		t_global.Tic();
		VectorX r;
		// compute the scaling parameter on the fly
		ScalarType scaling_parameter = (s_k.transpose()*y_k).trace() / (y_k.transpose()*y_k).trace();
		if (scaling_parameter < EPSILON) // should not be negative
		{
			scaling_parameter = EPSILON;
		}
		LBFGSKernelLinearSolve(r, q, scaling_parameter);
		t_global.Toc();
		t_other.Resume();
		g_lbfgs_timer.Resume();
		// loop 2 of l-BFGS
		for (int i = m_queue_visit_upper_bound - 1; i >= 0; i--)
		{
#ifdef USE_STL_QUEUE_IMPLEMENTATION
			// stl implementation
			ScalarType beta = rho[i] * m_lbfgs_y_queue[i].dot(r);
			r = r + m_lbfgs_s_queue[i] * (alpha[i] - beta);
#else
			// my implementation
			m_lbfgs_queue->visitSandY(&s_i, &y_i, i);
			Eigen::Map<const VectorX> s_i_eigen(s_i, x.size());
			Eigen::Map<const VectorX> y_i_eigen(y_i, x.size());
			ScalarType beta = rho[i] * y_i_eigen.dot(r);
			r += s_i_eigen * (alpha[i] - beta);
#endif
		}
		// update
		VectorX p_k = -r;
		if (-p_k.dot(gf_k) < EPSILON_SQUARE || p_k.squaredNorm() < EPSILON_SQUARE)
		{
			converged = true;
		}
		g_lbfgs_timer.Pause();
		t_other.Toc();

		t_linesearch.Tic();
		//ScalarType alpha_k = lineSearch(x, gf_k, p_k);
		ScalarType alpha_k = linesearchWithPrefetchedEnergyAndGradientComputing(x, current_energy, gf_k, p_k, m_ls_prefetched_energy, m_ls_prefetched_gradient);
		t_linesearch.Toc();

		x += alpha_k * p_k;

		t_global.Report("Forward Backward Substitution", verbose, TIMER_OUTPUT_MICROSECONDS);
		t_other.Report("Two loop overhead", verbose, TIMER_OUTPUT_MICROSECONDS);
		t_linesearch.Report("Linesearch", verbose, TIMER_OUTPUT_MICROSECONDS);
	}

	return converged;
}

void Simulation::LBFGSKernelLinearSolve(VectorX & r, VectorX rhs, ScalarType scaled_identity_constant) // Ar = rhs
{
	r.resize(rhs.size());
	switch (m_lbfgs_H0_type)
	{
	case LBFGS_H0_IDENTITY:
		r = rhs / scaled_identity_constant;
		break;
	case LBFGS_H0_LAPLACIAN: // h^2*laplacian+mass
	{
		// solve the linear system in reduced dimension because of the pattern of the Laplacian matrix
		// convert to nx3 space
		EigenMatrixx3 rhs_n3(rhs.size()/3, 3);
		Vector3mx1ToMatrixmx3(rhs, rhs_n3);
		// solve using the nxn laplacian
		EigenMatrixx3 r_n3;
		if (m_solver_type == SOLVER_TYPE_CG)
		{
			m_preloaded_cg_solver_1D.setMaxIterations(m_iterative_solver_max_iteration);
			r_n3 = m_preloaded_cg_solver_1D.solve(rhs_n3);
		}
		else
		{
			r_n3 = m_prefactored_solver_1D.solve(rhs_n3);
		}
		// convert the result back
		Matrixmx3ToVector3mx1(r_n3, r);


		////// conventional solve using 3nx3n system
		//if (m_solver_type == SOLVER_TYPE_CG)
		//{
		//	m_preloaded_cg_solver.setMaxIterations(m_iterative_solver_max_iteration);
		//	r = m_preloaded_cg_solver.solve(rhs);
		//}
		//else
		//{
		//	r = m_prefactored_solver.solve(rhs);
		//}
	}
	break;
	default:
		break;
	}
}

void Simulation::computeConstantVectorsYandZ()
{
	switch (m_integration_method)
	{
	case INTEGRATION_QUASI_STATICS:
		m_y = m_mesh->m_current_positions;
		break;
	case INTEGRATION_IMPLICIT_EULER:
		m_y = m_mesh->m_current_positions + m_mesh->m_current_velocities * m_h + m_h * m_h * m_mesh->m_inv_mass_matrix*m_external_force;
		break;
	case INTEGRATION_IMPLICIT_BDF2:
		m_y = (4 * m_mesh->m_current_positions - m_mesh->m_previous_positions) / 3 + (4 * m_mesh->m_current_velocities - m_mesh->m_previous_velocities + 2*m_h*m_mesh->m_inv_mass_matrix*m_external_force) * m_h * 2.0 / 9.0;
		break;
	case INTEGRATION_IMPLICIT_MIDPOINT:
		m_y = m_mesh->m_current_positions + m_mesh->m_current_velocities * m_h + 0.5 * m_h * m_h * m_mesh->m_inv_mass_matrix*m_external_force;
		break;
	case INTEGRATION_IMPLICIT_NEWMARK_BETA:
		m_y = m_mesh->m_current_positions + m_mesh->m_current_velocities * m_h + 0.5 * m_h * m_h * m_mesh->m_inv_mass_matrix*m_external_force;
		evaluateGradientPureConstraint(m_mesh->m_current_positions, m_external_force, m_z);
		break;
	default:
		break;
	}
}

void Simulation::updatePosAndVel(const VectorX& new_pos)
{
	switch (m_integration_method)
	{
	case INTEGRATION_QUASI_STATICS:
		m_mesh->m_previous_positions = m_mesh->m_current_positions;
		m_mesh->m_current_positions = new_pos;
		break;
	case INTEGRATION_IMPLICIT_EULER:
		m_mesh->m_previous_velocities = m_mesh->m_current_velocities;
		m_mesh->m_previous_positions = m_mesh->m_current_positions;
		m_mesh->m_current_velocities = (new_pos - m_mesh->m_current_positions) / m_h;
		m_mesh->m_current_positions = new_pos;
		break;
	case INTEGRATION_IMPLICIT_BDF2:
	{
		m_mesh->m_previous_velocities = m_mesh->m_current_velocities;
		m_mesh->m_current_velocities = 1.5 * (new_pos - (4 * m_mesh->m_current_positions - m_mesh->m_previous_positions) / 3) / m_h;;
		m_mesh->m_previous_positions = m_mesh->m_current_positions;
		m_mesh->m_current_positions = new_pos;
	}
		break;
	case INTEGRATION_IMPLICIT_MIDPOINT:
		m_mesh->m_previous_velocities = m_mesh->m_current_velocities;
		m_mesh->m_previous_positions = m_mesh->m_current_positions;
		m_mesh->m_current_velocities = 2 * (new_pos - m_mesh->m_current_positions) / m_h - m_mesh->m_current_velocities;
		m_mesh->m_current_positions = new_pos;
		break;
	case INTEGRATION_IMPLICIT_NEWMARK_BETA:
		m_mesh->m_previous_velocities = m_mesh->m_current_velocities;
		m_mesh->m_previous_positions = m_mesh->m_current_positions;
		m_mesh->m_current_velocities = 2 * (new_pos - m_mesh->m_current_positions) / m_h - m_mesh->m_current_velocities;
		m_mesh->m_current_positions = new_pos;
		break;
	default:
		break;
	}
}

ScalarType Simulation::evaluateEnergy(const VectorX& x)
{
	ScalarType energy_pure_constraints, energy;

	ScalarType inertia_term = 0.5 * (x - m_y).transpose() * m_mesh->m_mass_matrix * (x - m_y);
	ScalarType h_square = m_h*m_h;
	switch (m_integration_method)
	{
	case INTEGRATION_QUASI_STATICS:
		energy = evaluateEnergyPureConstraint(x, m_external_force);
		energy -= m_external_force.dot(x);
		break;
	case INTEGRATION_IMPLICIT_EULER:
		energy_pure_constraints = evaluateEnergyPureConstraint(x, m_external_force);
		energy = inertia_term + h_square*energy_pure_constraints;
		break;
	case INTEGRATION_IMPLICIT_BDF2:
		energy_pure_constraints = evaluateEnergyPureConstraint(x, m_external_force);
		energy = inertia_term + h_square*4.0 / 9.0*energy_pure_constraints;
		break;
	case INTEGRATION_IMPLICIT_MIDPOINT:
		energy_pure_constraints = evaluateEnergyPureConstraint((x+m_mesh->m_current_positions)/2, m_external_force);
		energy = inertia_term + h_square * (energy_pure_constraints);
		break;
	case INTEGRATION_IMPLICIT_NEWMARK_BETA:
		energy_pure_constraints = evaluateEnergyPureConstraint(x, m_external_force);
		energy = inertia_term + h_square / 4 * (energy_pure_constraints + m_z.dot(x));
		break;
	}

	return energy;
}

void Simulation::evaluateGradient(const VectorX& x, VectorX& gradient, bool enable_omp)
{
	ScalarType h_square = m_h*m_h;
	switch (m_integration_method)
	{
	case INTEGRATION_QUASI_STATICS:
		evaluateGradientPureConstraint(x, m_external_force, gradient);
		gradient -= m_external_force;
		break;//DO NOTHING
	case INTEGRATION_IMPLICIT_EULER:
		evaluateGradientPureConstraint(x, m_external_force, gradient);
		gradient = m_mesh->m_mass_matrix * (x - m_y) + h_square*gradient;
		break;
	case INTEGRATION_IMPLICIT_BDF2:
		evaluateGradientPureConstraint(x, m_external_force, gradient);
		gradient = m_mesh->m_mass_matrix * (x - m_y) + (h_square*4.0 / 9.0)*gradient;
		break;
	case INTEGRATION_IMPLICIT_MIDPOINT:
		evaluateGradientPureConstraint((x + m_mesh->m_current_positions) / 2, m_external_force, gradient);
		gradient = m_mesh->m_mass_matrix * (x - m_y) + h_square / 2 * (gradient);
		break;
	case INTEGRATION_IMPLICIT_NEWMARK_BETA:
		evaluateGradientPureConstraint(x, m_external_force, gradient);
		gradient = m_mesh->m_mass_matrix * (x - m_y) + h_square / 4 * (gradient + m_z);
		break;
	}
}

ScalarType Simulation::evaluateEnergyAndGradient(const VectorX& x, VectorX& gradient)
{
	ScalarType h_square = m_h*m_h;
	ScalarType energy_pure_constraints, energy;
	ScalarType inertia_term = 0.5 * (x - m_y).transpose() * m_mesh->m_mass_matrix * (x - m_y);

	switch (m_integration_method)
	{
	case INTEGRATION_QUASI_STATICS:
		energy = evaluateEnergyAndGradientPureConstraint(x, m_external_force, gradient);
		energy -= m_external_force.dot(x);
		gradient -= m_external_force;
		break;//DO NOTHING
	case INTEGRATION_IMPLICIT_EULER:
		energy_pure_constraints = evaluateEnergyAndGradientPureConstraint(x, m_external_force, gradient);
		energy = inertia_term + h_square*energy_pure_constraints;
		gradient = m_mesh->m_mass_matrix * (x - m_y) + h_square*gradient;
		break;
	case INTEGRATION_IMPLICIT_BDF2:
		energy_pure_constraints = evaluateEnergyAndGradientPureConstraint(x, m_external_force, gradient);
		energy = inertia_term + h_square*4.0 / 9.0*energy_pure_constraints;
		gradient = m_mesh->m_mass_matrix * (x - m_y) + (h_square*4.0 / 9.0)*gradient;
		break;
	case INTEGRATION_IMPLICIT_MIDPOINT:
		energy_pure_constraints = evaluateEnergyAndGradientPureConstraint((x + m_mesh->m_current_positions)/2, m_external_force, gradient);
		energy = inertia_term + h_square * (energy_pure_constraints);
		gradient = m_mesh->m_mass_matrix * (x - m_y) + h_square / 2 * (gradient);
		break;
	case INTEGRATION_IMPLICIT_NEWMARK_BETA:
		energy_pure_constraints = evaluateEnergyAndGradientPureConstraint(x, m_external_force, gradient);
		energy = inertia_term + h_square / 4 * (energy_pure_constraints + m_z.dot(x));
		gradient = m_mesh->m_mass_matrix * (x - m_y) + h_square / 4 * (gradient + m_z);
		break;
	}

	return energy;
}

void Simulation::evaluateHessian(const VectorX& x, SparseMatrix& hessian_matrix)
{
	ScalarType h_square = m_h*m_h;
	switch (m_integration_method)
	{
	case INTEGRATION_QUASI_STATICS:
		evaluateHessianPureConstraint(x, hessian_matrix);
		break;//DO NOTHING
	case INTEGRATION_IMPLICIT_EULER:
		evaluateHessianPureConstraint(x, hessian_matrix);
		hessian_matrix = m_mesh->m_mass_matrix + h_square*hessian_matrix;
		break;
	case INTEGRATION_IMPLICIT_BDF2:
		evaluateHessianPureConstraint(x, hessian_matrix);
		hessian_matrix = m_mesh->m_mass_matrix + h_square*4.0 / 9.0*hessian_matrix;
		break;
	case INTEGRATION_IMPLICIT_MIDPOINT:
		evaluateHessianPureConstraint((x + m_mesh->m_current_positions) / 2, hessian_matrix);
		hessian_matrix = m_mesh->m_mass_matrix + h_square / 4 * hessian_matrix;
		break;
	case INTEGRATION_IMPLICIT_NEWMARK_BETA:
		evaluateHessianPureConstraint(x, hessian_matrix);
		hessian_matrix = m_mesh->m_mass_matrix + h_square / 4 * hessian_matrix;
		break;
	}
}

void Simulation::evaluateHessianSmart(const VectorX& x, SparseMatrix& hessian_matrix)
{
	ScalarType h_square = m_h*m_h;
	switch (m_integration_method)
	{
	case INTEGRATION_QUASI_STATICS:
		evaluateHessianPureConstraintSmart(x, hessian_matrix);
		break;//DO NOTHING
	case INTEGRATION_IMPLICIT_EULER:
		evaluateHessianPureConstraintSmart(x, hessian_matrix);
		hessian_matrix = m_mesh->m_mass_matrix + h_square*hessian_matrix;
		break;
	case INTEGRATION_IMPLICIT_BDF2:
		evaluateHessianPureConstraintSmart(x, hessian_matrix);
		hessian_matrix = m_mesh->m_mass_matrix + h_square*4.0 / 9.0*hessian_matrix;
		break;
	case INTEGRATION_IMPLICIT_MIDPOINT:
		evaluateHessianPureConstraintSmart((x + m_mesh->m_current_positions) / 2, hessian_matrix);
		hessian_matrix = m_mesh->m_mass_matrix + h_square / 4 * hessian_matrix;
		break;
	case INTEGRATION_IMPLICIT_NEWMARK_BETA:
		evaluateHessianPureConstraintSmart(x, hessian_matrix);
		hessian_matrix = m_mesh->m_mass_matrix + h_square / 4 * hessian_matrix;
		break;
	}
}

// evaluate hessian
void Simulation::evaluateHessianForCG(const VectorX& x)
{
	VectorX x_evaluated_point;
	switch (m_integration_method)
	{
	case INTEGRATION_QUASI_STATICS:
	case INTEGRATION_IMPLICIT_EULER:
	case INTEGRATION_IMPLICIT_BDF2:
	case INTEGRATION_IMPLICIT_NEWMARK_BETA:
		x_evaluated_point = x;
		break;
	case INTEGRATION_IMPLICIT_MIDPOINT:
		x_evaluated_point = (x + m_mesh->m_current_positions) / 2;
		break;
	}

	if (!m_enable_openmp)
	{
		for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
		{
			(*it)->EvaluateHessian(x_evaluated_point, m_definiteness_fix);
		}

		if (m_processing_collision)
		{
			for (std::vector<CollisionSpringConstraint>::iterator it = m_collision_constraints.begin(); it != m_collision_constraints.end(); ++it)
			{
				it->EvaluateHessian(x_evaluated_point, m_definiteness_fix);
			}
		}
	}
	else
	{
		int i;
#pragma omp parallel
		{
#pragma omp for
			for (i = 0; i < m_constraints.size(); i++)
			{
				m_constraints[i]->EvaluateHessian(x_evaluated_point, m_definiteness_fix);
			}
#pragma omp for
			for (i = 0; i < m_collision_constraints.size(); i++)
			{
				m_collision_constraints[i].EvaluateHessian(x_evaluated_point, m_definiteness_fix);
			}
		}
	}
}
// apply hessian
void Simulation::applyHessianForCG(const VectorX& x, VectorX & b)
{
	ScalarType h_square = m_h*m_h;
	switch (m_integration_method)
	{
	case INTEGRATION_QUASI_STATICS:
		applyHessianForCGPureConstraint(x, b);
		break;//DO NOTHING
	case INTEGRATION_IMPLICIT_EULER:
		applyHessianForCGPureConstraint(x, b);
		b = m_mesh->m_mass_matrix * x + h_square*b;
		break;
	case INTEGRATION_IMPLICIT_BDF2:
		applyHessianForCGPureConstraint(x, b);
		b = m_mesh->m_mass_matrix * x + h_square*4.0 / 9.0*b;
		break;
	case INTEGRATION_IMPLICIT_MIDPOINT:
		applyHessianForCGPureConstraint(x, b);
		b = m_mesh->m_mass_matrix * x + h_square / 4 * b;
		break;
	case INTEGRATION_IMPLICIT_NEWMARK_BETA:
		applyHessianForCGPureConstraint(x, b);
		b = m_mesh->m_mass_matrix * x + h_square/4*b;
		break;
	}

}

void Simulation::evaluateLaplacian(SparseMatrix& laplacian_matrix)
{
	evaluateLaplacianPureConstraint(laplacian_matrix);

#ifdef ENABLE_MATLAB_DEBUGGING
	g_debugger->SendSparseMatrix(m_weighted_laplacian, "L");
#endif

	ScalarType h_square = m_h*m_h;
	switch (m_integration_method)
	{
	case INTEGRATION_QUASI_STATICS:
		break;//DO NOTHING
	case INTEGRATION_IMPLICIT_EULER:
		laplacian_matrix = m_mesh->m_mass_matrix + h_square*laplacian_matrix;
		break;
	case INTEGRATION_IMPLICIT_BDF2:
		laplacian_matrix = m_mesh->m_mass_matrix + h_square*4.0 / 9.0*laplacian_matrix;
		break;
	case INTEGRATION_IMPLICIT_MIDPOINT:
		laplacian_matrix = m_mesh->m_mass_matrix + h_square / 4 * laplacian_matrix;
		break;
	case INTEGRATION_IMPLICIT_NEWMARK_BETA:
		laplacian_matrix = m_mesh->m_mass_matrix + h_square / 4 * laplacian_matrix;
		break;
	}

#ifdef ENABLE_MATLAB_DEBUGGING
	g_debugger->SendSparseMatrix(laplacian_matrix, "A");
#endif

}

void Simulation::evaluateLaplacian1D(SparseMatrix & laplacian_matrix_1d)
{
	evaluateLaplacianPureConstraint1D(laplacian_matrix_1d);

	ScalarType h_square = m_h*m_h;
	switch (m_integration_method)
	{
	case INTEGRATION_QUASI_STATICS:
		break;//DO NOTHING
	case INTEGRATION_IMPLICIT_EULER:
		laplacian_matrix_1d = m_mesh->m_mass_matrix_1d + h_square*laplacian_matrix_1d;
		break;
	case INTEGRATION_IMPLICIT_BDF2:
		laplacian_matrix_1d = m_mesh->m_mass_matrix_1d + h_square*4.0 / 9.0*laplacian_matrix_1d;
		break;
	case INTEGRATION_IMPLICIT_MIDPOINT:
		laplacian_matrix_1d = m_mesh->m_mass_matrix_1d + h_square / 4 * laplacian_matrix_1d;
		break;
	case INTEGRATION_IMPLICIT_NEWMARK_BETA:
		laplacian_matrix_1d = m_mesh->m_mass_matrix_1d + h_square / 4 * laplacian_matrix_1d;
		break;
	}
}

ScalarType Simulation::getVolume(const VectorX& x)
{
	ScalarType volume = 0.0;
	for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
	{
		volume += (*it)->GetVolume(x);
	}

	return volume;
}

void Simulation::printVolumeTesting(const VectorX& x)
{
	unsigned int index = 0;
	for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
	{
		if ((*it)->Type() == CONSTRAINT_TYPE_TET)
		{
			TetConstraint* tc = dynamic_cast<TetConstraint*> (*it);
			ScalarType rest_vol = tc->GetVolume(m_mesh->m_restpose_positions);
			ScalarType vol = tc->GetVolume(x);

			if (vol / rest_vol < 0.1)
			{
				std::cout << "Volume of element: " << index << std::endl;
				std::cout << "rest pose vol: " << rest_vol << std::endl;
				std::cout << "current pose vol: " << vol << std::endl << std::endl;
			}

			index++;
		}
	}
}

ScalarType Simulation::evaluateEnergyPureConstraint(const VectorX& x, const VectorX& f_ext)
{
	ScalarType energy = 0.0;

	if (!m_enable_openmp)
	{
		for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
		{
			energy += (*it)->EvaluateEnergy(x);
		}
	}
	else
	{
		// openmp get all energy
		int i;
#pragma omp parallel
		{
#pragma omp for
			for (i = 0; i < m_constraints.size(); i++)
			{
				m_constraints[i]->EvaluateEnergy(x);
			}
		}

		// reduction
		for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
		{
			energy += (*it)->GetEnergy();
		}
	}

	//energy -= f_ext.dot(x);

	//// collision
	//for (unsigned int i = 1; i * 3 < x.size(); i++)
	//{
	//	EigenVector3 xi = x.block_vector(i);
	//	EigenVector3 n;
	//	ScalarType d;
	//	if (m_scene->StaticIntersectionTest(xi, n, d))
	//	{

	//	}
	//}

	// hardcoded collision plane
	if (m_processing_collision)
	{
		energy += evaluateEnergyCollision(x);
	}

	return energy;
}
void Simulation::evaluateGradientPureConstraint(const VectorX& x, const VectorX& f_ext, VectorX& gradient)
{
	gradient.resize(m_mesh->m_system_dimension);
	gradient.setZero();

	if (!m_enable_openmp)
	{
		// constraints single thread
		for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
		{
			(*it)->EvaluateGradient(x, gradient);
		}
	}
	else
	{
		// constraints omp
		int i;
#pragma omp parallel
		{
#pragma omp for
			for (i = 0; i < m_constraints.size(); i++)
			{
				m_constraints[i]->EvaluateGradient(x);
			}
		}

		for (i = 0; i < m_constraints.size(); i++)
		{
			m_constraints[i]->GetGradient(gradient);
		}
	}

	// hardcoded collision plane
	if (m_processing_collision)
	{
		VectorX gc;

		evaluateGradientCollision(x, gc);

		gradient += gc;
	}
}
ScalarType Simulation::evaluateEnergyAndGradientPureConstraint(const VectorX& x, const VectorX& f_ext, VectorX& gradient)
{
	ScalarType energy = 0.0;
	gradient.resize(m_mesh->m_system_dimension);
	gradient.setZero();

	if (!m_enable_openmp)
	{
		// constraints single thread
		for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
		{
			energy += (*it)->EvaluateEnergyAndGradient(x, gradient);
		}
	}
	else
	{
		// constraints omp
		int i;
#pragma omp parallel
		{
#pragma omp for
			for (i = 0; i < m_constraints.size(); i++)
			{
				m_constraints[i]->EvaluateEnergyAndGradient(x);
			}
		}

		// collect the results in a sequential way
		for (i = 0; i < m_constraints.size(); i++)
		{
			energy += m_constraints[i]->GetEnergyAndGradient(gradient);
		}
	}

	// hardcoded collision plane
	if (m_processing_collision)
	{
		VectorX gc;

		energy += evaluateEnergyAndGradientCollision(x, gc);

		gradient += gc;
	}

	return energy;
}

void Simulation::evaluateHessianPureConstraint(const VectorX& x, SparseMatrix& hessian_matrix)
{
	hessian_matrix.resize(m_mesh->m_system_dimension, m_mesh->m_system_dimension);
	std::vector<SparseMatrixTriplet> h_triplets;
	h_triplets.clear();

	for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
	{
		(*it)->EvaluateHessian(x, h_triplets, m_definiteness_fix);
	}

	hessian_matrix.setFromTriplets(h_triplets.begin(), h_triplets.end());

	if (m_processing_collision)
	{
		SparseMatrix HC;
		evaluateHessianCollision(x, HC);
		hessian_matrix += HC;
	}
}

void Simulation::evaluateHessianPureConstraintSmart(const VectorX& x, SparseMatrix& hessian_matrix)
{
	hessian_matrix.resize(m_mesh->m_system_dimension, m_mesh->m_system_dimension);
	std::vector<SparseMatrixTriplet> h_triplets;
	h_triplets.clear();

	for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
	{
		(*it)->EvaluateHessian(x, h_triplets, m_definiteness_fix);
	}

	// sort triplets using ascent order of ||triplet.value()||
	std::sort(h_triplets.begin(), h_triplets.end(), compareTriplet);

	hessian_matrix.setFromTriplets(h_triplets.begin(), h_triplets.end());

	if (m_processing_collision)
	{
		SparseMatrix HC;
		evaluateHessianCollision(x, HC);
		hessian_matrix += HC;
	}
}

void Simulation::evaluateLaplacianPureConstraint(SparseMatrix& laplacian_matrix)
{
	laplacian_matrix.resize(m_mesh->m_system_dimension, m_mesh->m_system_dimension);
	std::vector<SparseMatrixTriplet> l_triplets;
	l_triplets.clear();

	for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
	{
		(*it)->EvaluateWeightedLaplacian(l_triplets);
	}

	laplacian_matrix.setFromTriplets(l_triplets.begin(), l_triplets.end());
}

void Simulation::evaluateLaplacianPureConstraint1D(SparseMatrix & laplacian_matrix_1d)
{
	laplacian_matrix_1d.resize(m_mesh->m_vertices_number, m_mesh->m_vertices_number);
	std::vector<SparseMatrixTriplet> l_1d_triplets;
	l_1d_triplets.clear();

	for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
	{
		(*it)->EvaluateWeightedLaplacian1D(l_1d_triplets);
	}

	laplacian_matrix_1d.setFromTriplets(l_1d_triplets.begin(), l_1d_triplets.end());
}

void Simulation::applyHessianForCGPureConstraint(const VectorX& x, VectorX& b)
{
	b.resize(x.size());
	b.setZero();
	for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
	{
		(*it)->ApplyHessian(x, b);
	}
}

ScalarType Simulation::evaluateEnergyCollision(const VectorX& x)
{
	ScalarType energy = 0.0;

	if (!m_enable_openmp)
	{
		for (std::vector<CollisionSpringConstraint>::iterator it = m_collision_constraints.begin(); it != m_collision_constraints.end(); ++it)
		{
			energy += it->EvaluateEnergy(x);
		}
	}
	else
	{
		// openmp get all energy
		int i;
#pragma omp parallel
		{
#pragma omp for
			for (i = 0; i < m_collision_constraints.size(); i++)
			{
				m_collision_constraints[i].EvaluateEnergy(x);
			}
		}

		// reduction
		for (std::vector<CollisionSpringConstraint>::iterator it = m_collision_constraints.begin(); it != m_collision_constraints.end(); ++it)
		{
			energy += it->GetEnergy();
		}
	}

	return energy;
}
void Simulation::evaluateGradientCollision(const VectorX& x, VectorX& gradient)
{
	gradient.resize(m_mesh->m_system_dimension);
	gradient.setZero();

	if (!m_enable_openmp)
	{
		// constraints single thread
		for (std::vector<CollisionSpringConstraint>::iterator it = m_collision_constraints.begin(); it != m_collision_constraints.end(); ++it)
		{
			it->EvaluateGradient(x, gradient);
		}
	}
	else
	{
		// constraints omp
		int i;
#pragma omp parallel
		{
#pragma omp for
			for (i = 0; i < m_collision_constraints.size(); i++)
			{
				m_collision_constraints[i].EvaluateGradient(x);
			}
		}

		for (i = 0; i < m_collision_constraints.size(); i++)
		{
			m_collision_constraints[i].GetGradient(gradient);
		}
	}
}

ScalarType Simulation::evaluateEnergyAndGradientCollision(const VectorX& x, VectorX& gradient)
{
	ScalarType energy = 0.0;
	gradient.resize(m_mesh->m_system_dimension);
	gradient.setZero();

	if (!m_enable_openmp)
	{
		// constraints single thread
		for (std::vector<CollisionSpringConstraint>::iterator it = m_collision_constraints.begin(); it != m_collision_constraints.end(); ++it)
		{
			energy += it->EvaluateEnergyAndGradient(x, gradient);
		}
	}
	else
	{
		// constraints omp
		int i;
#pragma omp parallel
		{
#pragma omp for
			for (i = 0; i < m_collision_constraints.size(); i++)
			{
				m_collision_constraints[i].EvaluateEnergyAndGradient(x);
			}
		}

		// collect the results in a sequential way
		for (i = 0; i < m_collision_constraints.size(); i++)
		{
			energy += m_collision_constraints[i].GetEnergyAndGradient(gradient);
		}
	}

	return energy;
}
void Simulation::evaluateHessianCollision(const VectorX& x, SparseMatrix& hessian_matrix)
{
	hessian_matrix.resize(m_mesh->m_system_dimension, m_mesh->m_system_dimension);
	std::vector<SparseMatrixTriplet> h_triplets;
	h_triplets.clear();

	for (std::vector<CollisionSpringConstraint>::iterator it = m_collision_constraints.begin(); it != m_collision_constraints.end(); ++it)
	{
		it->EvaluateHessian(x, h_triplets, m_definiteness_fix);
	}

	hessian_matrix.setFromTriplets(h_triplets.begin(), h_triplets.end());
}

ScalarType Simulation::lineSearch(const VectorX& x, const VectorX& gradient_dir, const VectorX& descent_dir)
{
	if (m_enable_line_search)
	{
		VectorX x_plus_tdx(m_mesh->m_system_dimension);
		ScalarType t = 1.0 / m_ls_beta;
		//ScalarType t = m_ls_step_size/m_ls_beta;
		ScalarType lhs, rhs;

		ScalarType currentObjectiveValue;
		try
		{
			currentObjectiveValue = evaluateEnergy(x);
		}
		catch (const std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
		do
		{
#ifdef OUTPUT_LS_ITERATIONS
			g_total_ls_iterations++;
#endif
			t *= m_ls_beta;
			x_plus_tdx = x + t*descent_dir;

			lhs = 1e15;
			rhs = 0;
			try
			{
				lhs = evaluateEnergy(x_plus_tdx);
			}
			catch (const std::exception&)
			{
				continue;
			}
			rhs = currentObjectiveValue + m_ls_alpha * t * (gradient_dir.transpose() * descent_dir)(0);
			if (lhs >= rhs)
			{
				continue; // keep looping
			}

			break; // exit looping

		} while (t > 1e-5);

		if (t < 1e-5)
		{
			t = 0.0;
		}
		m_ls_step_size = t;

		if (m_verbose_show_converge)
		{
			std::cout << "Linesearch Stepsize = " << t << std::endl;
			std::cout << "lhs (current energy) = " << lhs << std::endl;
			std::cout << "previous energy = " << currentObjectiveValue << std::endl;
			std::cout << "rhs (previous energy + alpha * t * gradient.dot(descet_dir)) = " << rhs << std::endl;
		}

#ifdef OUTPUT_LS_ITERATIONS
		g_total_iterations++;
		if (g_total_iterations % OUTPUT_LS_ITERATIONS_EVERY_N_FRAMES == 0)
		{
			std::cout << "Avg LS Iterations = " << g_total_ls_iterations / g_total_iterations << std::endl;
			g_total_ls_iterations = 0;
			g_total_iterations = 0;
		}
#endif
		return t;
	}
	else
	{
		return m_ls_step_size;
	}
}

ScalarType Simulation::linesearchWithPrefetchedEnergyAndGradientComputing(const VectorX& x, const ScalarType current_energy, const VectorX& gradient_dir, const VectorX& descent_dir, ScalarType& next_energy, VectorX& next_gradient_dir)
{
	if (m_enable_line_search)
	{
		VectorX x_plus_tdx(m_mesh->m_system_dimension);
		ScalarType t = 1.0 / m_ls_beta;
		ScalarType lhs, rhs;

		ScalarType currentObjectiveValue = current_energy;

		do
		{
#ifdef OUTPUT_LS_ITERATIONS
			g_total_ls_iterations++;
#endif

			t *= m_ls_beta;
			x_plus_tdx = x + t*descent_dir;

			lhs = 1e15;
			rhs = 0;
			try
			{
				lhs = evaluateEnergyAndGradient(x_plus_tdx, next_gradient_dir);
			}
			catch (const std::exception&)
			{
				continue;
			}
			rhs = currentObjectiveValue + m_ls_alpha * t * (gradient_dir.transpose() * descent_dir)(0);
			if (lhs >= rhs)
			{
				continue; // keep looping
			}

			next_energy = lhs;
			break; // exit looping

		} while (t > 1e-5);

		if (t < 1e-5)
		{
			t = 0.0;
			next_energy = current_energy;
			next_gradient_dir = gradient_dir;
		}
		m_ls_step_size = t;

		if (m_verbose_show_converge)
		{
			std::cout << "Linesearch Stepsize = " << t << std::endl;
			std::cout << "lhs (current energy) = " << lhs << std::endl;
			std::cout << "previous energy = " << currentObjectiveValue << std::endl;
			std::cout << "rhs (previous energy + alpha * t * gradient.dot(descet_dir)) = " << rhs << std::endl;
		}

#ifdef OUTPUT_LS_ITERATIONS
		g_total_iterations++;
		if (g_total_iterations % OUTPUT_LS_ITERATIONS_EVERY_N_FRAMES == 0)
		{
			std::cout << "Avg LS Iterations = " << g_total_ls_iterations / g_total_iterations << std::endl;
			g_total_ls_iterations = 0;
			g_total_iterations = 0;
		}
#endif

		return t;
	}
	else
	{
		return m_ls_step_size;
	}
}

ScalarType Simulation::evaluatePotentialEnergy(const VectorX& x)
{
	ScalarType energy = evaluateEnergyPureConstraint(x, m_external_force);
	energy -= m_external_force.dot(x);

	return energy;
}
ScalarType Simulation::evaluateKineticEnergy(const VectorX& v)
{
	return (0.5*v.transpose()*m_mesh->m_mass_matrix*v);
}
ScalarType Simulation::evaluateTotalEnergy(const VectorX& x, const VectorX& v)
{
	return (evaluatePotentialEnergy(x) + evaluateKineticEnergy(v));
}

#pragma region matrices and prefactorization
void Simulation::setWeightedLaplacianMatrix()
{
	evaluateLaplacian(m_weighted_laplacian);
}

void Simulation::setWeightedLaplacianMatrix1D()
{
	evaluateLaplacian1D(m_weighted_laplacian_1D);
}

void Simulation::precomputeLaplacianWeights()
{
	for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
	{
		if ((*it)->Type() == CONSTRAINT_TYPE_TET)
		{
			m_stiffness_laplacian = (*it)->ComputeLaplacianWeight();
		}
	}
	SetReprefactorFlag();
}

void Simulation::precomputeLaplacian()
{
	if (m_precomputing_flag == false)
	{
		setWeightedLaplacianMatrix();

		m_precomputing_flag = true;
	}
}

void Simulation::prefactorize()
{
	if (m_prefactorization_flag == false)
	{
		// update laplacian coefficients
		if (m_stiffness_auto_laplacian_stiffness)
		{
			precomputeLaplacianWeights();
		}
		else
		{
			SetMaterialProperty();
		}

		// full space laplacian 3n x 3n
		setWeightedLaplacianMatrix();
		factorizeDirectSolverLLT(m_weighted_laplacian, m_prefactored_solver, "Our Method");		// prefactorization of laplacian
		m_preloaded_cg_solver.compute(m_weighted_laplacian);		// load the cg solver

		// reduced dim space laplacian nxn
		setWeightedLaplacianMatrix1D();
		factorizeDirectSolverLLT(m_weighted_laplacian_1D, m_prefactored_solver_1D, "Our Method Reduced Space");
		m_preloaded_cg_solver_1D.compute(m_weighted_laplacian_1D);

#ifdef ENABLE_MATLAB_DEBUGGING
		g_debugger->SendSparseMatrix(m_weighted_laplacian, "L");
		g_debugger->SendSparseMatrix(m_weighted_laplacian_1D, "L1");
#endif
		m_prefactorization_flag = true;
	}
}
#pragma endregion

#pragma region newton_solver
void Simulation::analyzeNewtonSolverPattern(const SparseMatrix& A)
{
	if (!m_prefactorization_flag_newton)
	{
		m_newton_solver.analyzePattern(A);
		m_prefactorization_flag_newton = true;
	}
}

void Simulation::factorizeNewtonSolver(const SparseMatrix& A, char* warning_msg)
{
	SparseMatrix A_prime = A;
	m_newton_solver.factorize(A_prime);
	ScalarType Regularization = 1e-10;
	bool success = true;
	SparseMatrix I;
	while (m_newton_solver.info() != Eigen::Success)
	{
		if (success == true) // first time here
		{
			EigenMakeSparseIdentityMatrix(A.rows(), A.cols(), I);
		}
		Regularization *= 10;
		A_prime = A_prime + Regularization*I;
		m_newton_solver.factorize(A_prime);
		success = false;
	}
	if (!success && m_verbose_show_factorization_warning)
		std::cout << "Warning: " << warning_msg <<  " adding "<< Regularization <<" identites.(llt solver)" << std::endl;
}
#pragma endregion

#pragma region utilities

ScalarType Simulation::linearSolve(VectorX& x, const SparseMatrix& A, const VectorX& b, char* msg)
{
	ScalarType residual = 0;

	switch (m_solver_type)
	{
	case SOLVER_TYPE_DIRECT_LLT:
	{
#ifdef PARDISO_SUPPORT
		Eigen::PardisoLLT<SparseMatrix, Eigen::Upper> A_solver;
#else
		Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper> A_solver;
#endif
		factorizeDirectSolverLLT(A, A_solver, msg);
		x = A_solver.solve(b);
	}
		break;
	case SOLVER_TYPE_CG:
	{
		x.resize(b.size());
		x.setZero();
		residual = conjugateGradientWithInitialGuess(x, A, b, m_iterative_solver_max_iteration);
	}
		break;
	default:
		break;
	}

	return residual;
}

ScalarType Simulation::conjugateGradientWithInitialGuess(VectorX& x, const SparseMatrix& A, const VectorX& b, const unsigned int max_it /* = 200 */, const ScalarType tol /* = 1e-5 */)
{
	VectorX r = b - A*x;
	VectorX p = r;
	ScalarType rsold = r.dot(r);
	ScalarType rsnew;

	VectorX Ap;
	Ap.resize(x.size());
	ScalarType alpha;

	for (unsigned int i = 1; i != max_it; ++i)
	{
		Ap = A*p;
		alpha = rsold / p.dot(Ap);
		x = x + alpha * p;

		r = r - alpha*Ap;
		rsnew = r.dot(r);
		if (sqrt(rsnew) < tol)
		{
			break;
		}
		p = r + (rsnew / rsold)*p;
		rsold = rsnew;
	}

	return sqrt(rsnew);
}

void Simulation::factorizeDirectSolverLLT(const SparseMatrix& A, Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>& lltSolver, char* warning_msg)
{
	SparseMatrix A_prime = A;
	lltSolver.analyzePattern(A_prime);
	lltSolver.factorize(A_prime);
	ScalarType Regularization = 1e-10;
	bool success = true;
	SparseMatrix I;
	while (lltSolver.info() != Eigen::Success)
	{
		if (success == true) // first time factorization failed
		{
			EigenMakeSparseIdentityMatrix(A.rows(), A.cols(), I);
		}
		Regularization *= 10;
		A_prime = A_prime + Regularization*I;
		lltSolver.factorize(A_prime);
		success = false;
	}
	if (!success && m_verbose_show_factorization_warning)
		std::cout << "Warning: " << warning_msg <<  " adding "<< Regularization <<" identites.(llt solver)" << std::endl;
}

#ifdef PARDISO_SUPPORT
void Simulation::factorizeDirectSolverLLT(const SparseMatrix& A, Eigen::PardisoLLT<SparseMatrix, Eigen::Upper>& lltSolver, char* warning_msg)
{
	SparseMatrix A_prime = A;
	lltSolver.analyzePattern(A_prime);
	lltSolver.factorize(A_prime);
	ScalarType Regularization = 1e-10;
	bool success = true;
	SparseMatrix I;
	while (lltSolver.info() != Eigen::Success)
	{
		if (success == true) // first time factorization failed
		{
			EigenMakeSparseIdentityMatrix(A.rows(), A.cols(), I);
		}
		Regularization *= 10;
		A_prime = A_prime + Regularization*I;
		lltSolver.factorize(A_prime);
		success = false;
	}
	if (!success && m_verbose_show_factorization_warning)
		std::cout << "Warning: " << warning_msg << " adding " << Regularization << " identites.(llt solver)" << std::endl;
}
#endif

void Simulation::generateRandomVector(const unsigned int size, VectorX& x)
{
	x.resize(size);
	for (unsigned int i = 0; i < size; i++)
	{
		x(i) = ((ScalarType)(rand()) / (ScalarType)(RAND_MAX + 1) - 0.5) * 2;
	}

	//x.resize(size);
	//unsigned int dim = 0;
	//ScalarType scale[3];
	//scale[0] = scale[2] = 1;
	//scale[1] = 1e6;
	//for (unsigned int i = 0; i < size; i++)
	//{
	//	x(i) = ((ScalarType)(rand()) / (ScalarType)(RAND_MAX + 1) - 0.5) * 2 * scale[dim];
	//	dim = (dim + 1) % 3;
	//}
}

#pragma endregion
