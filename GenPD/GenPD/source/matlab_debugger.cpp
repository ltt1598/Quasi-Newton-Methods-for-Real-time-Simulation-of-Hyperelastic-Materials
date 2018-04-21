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


#include "matlab_debugger.h"
//#include "timer_wrapper.h"

#include <iostream>

#ifdef ENABLE_MATLAB_DEBUGGING
MatlabDebugger::MatlabDebugger() :
m_engine(NULL)
{
	m_enable = false;
	m_plot_type = PLOT_TYPE_SEMILOG_Y;
	m_plot_data_type = PLOT_DATA_TYPE_GRADIENT_NORM;
}

MatlabDebugger::~MatlabDebugger()
{
	if (m_engine != NULL)
	{
		engClose(m_engine);
	}
}

void MatlabDebugger::Init()
{
	m_enable = true;

	m_engine = engOpen(NULL);
	if (m_engine == NULL)
	{
		fprintf(stderr, "ERROR: Can not initialize matlab engine.\n");
		exit(EXIT_FAILURE);
	}

	m_double1x1_array = mxCreateDoubleMatrix(1, 1, mxREAL);
	m_double1x1 = mxGetPr(m_double1x1_array);

	m_plot_x_axis_plot_iterations = true;

	Reset();
}

void MatlabDebugger::ResetCurrent()
{
	if (!m_enable)
		return;

	engEvalString(m_engine, "it = []");
	engEvalString(m_engine, "time = []");
	engEvalString(m_engine, "X = []");
	engEvalString(m_engine, "gradient_norm = []");
	engEvalString(m_engine, "energy = []");
	engEvalString(m_engine, "diff = []");

	m_data_filled = false;
}

void MatlabDebugger::Reset()
{
	if (!m_enable)
		return;

	engEvalString(m_engine, "close");
	engEvalString(m_engine, "clear");
	engEvalString(m_engine, "it = []");
	engEvalString(m_engine, "time = []");
	engEvalString(m_engine, "X = []");
	engEvalString(m_engine, "gradient_norm = []");
	engEvalString(m_engine, "energy = []");
	engEvalString(m_engine, "diff = []");
	engEvalString(m_engine, "Hessian = cell(0)");
	engEvalString(m_engine, "Gradient = cell(0)");

	char command[200];
	sprintf_s(command, sizeof(char) * 200, "plot_capacity = %d", m_plot_capacity);
	engEvalString(m_engine, command);

	engEvalString(m_engine, "it_cell = cell(1, plot_capacity);");
	engEvalString(m_engine, "time_cell = cell(1, plot_capacity);");
	engEvalString(m_engine, "dist_cell = cell(1, plot_capacity);");
	engEvalString(m_engine, "energy_cell = cell(1, plot_capacity);");
	engEvalString(m_engine, "gradient_norm_cell = cell(1, plot_capacity);");
	engEvalString(m_engine, "lgd_cell = cell(1, plot_capacity);");

	m_legend_string[0] = '\0';
	m_plot_index = 0;

	m_vis_name[0] = '\0';
	m_vis_name_current[0] = '\0';
	m_vis_ready = false;
	m_vis_amplify = 100;

	m_data_filled = false;
	m_converged_energy_set = false;
}

void MatlabDebugger::Export()
{
	engEvalString(m_engine, "save('D:/Desktop/data.mat')");
}

void MatlabDebugger::Import()
{
	engEvalString(m_engine, "load('D:/Desktop/data.mat')");

	m_double1x1_array = engGetVariable(m_engine, "plot_index");
	m_plot_index = m_double1x1[0] - 1;
}

void MatlabDebugger::SaveCurrentData(const char* filename)
{
	if (!m_enable)
		return;

	//std::string data_path_file = m_gs->m_exe_path+std::string("/matlab/")+std::string(filename);
	//std::string command = std::string("save('")+data_path_file+std::string("');");

	//engEvalString(m_engine, command.c_str());
}

void MatlabDebugger::SaveCurrentFigure(const char* filename)
{
	if (!m_enable)
		return;

	//std::string figure_path_file = m_gs->m_exe_path+std::string("/matlab/")+std::string(filename);
	//std::string command = std::string("saveas(handle, '")+figure_path_file+std::string("', 'fig');");

	//char handle_command[255];
	//sprintf(handle_command, "handle = figure(%d);", m_gs->m_choose_figure_to_save);
	//engEvalString(m_engine,handle_command);
	//engEvalString(m_engine, command.c_str());
}
void MatlabDebugger::SendCommand(char* command)
{
	engEvalString(m_engine, command);
}
void MatlabDebugger::SendVector(const std::vector<ScalarType> &x, char* name)
{
	VectorX x_eigen(x.size());
	for (unsigned int i = 0; i != x.size(); i++)
	{
		x_eigen(i) = x[i];
	}
	SendVector(x_eigen, name);
}
void MatlabDebugger::SendVector(const VectorX &x, char* name)
{
	SendVector(x, name, x.size());
}

void MatlabDebugger::SendVector(const VectorX &x, char* name, int size)
{
	if (!m_enable)
		return;

	m_vector_array = mxCreateDoubleMatrix(size, 1, mxREAL);
	m_vector = mxGetPr(m_vector_array);

	char command[200];
	sprintf_s(command, sizeof(char)*200, "%s = []", name);

	engEvalString(m_engine, command);

	for (int i = 0; i < size; i++)
	{
		m_vector[i] = x(i);
	}

	engPutVariable(m_engine, name, m_vector_array);

	mxDestroyArray(m_vector_array);
}

void MatlabDebugger::SendSparseMatrix(const SparseMatrix& A, char* name)
{
	int d1 = A.rows();
	int d2 = A.cols();

	std::vector<SparseMatrixTriplet> triplets;
	EigenSparseMatrixToTriplets(A, triplets);

	SendSparseMatrixUsingTriplets(triplets, name, d1, d2);
}

void MatlabDebugger::SendSparseMatrixUsingTriplets(std::vector<SparseMatrixTriplet> &triplets, char* name, int d1, int d2)
{
	if (!m_enable)
		return;

	int size = triplets.size();

	m_triplet_array = mxCreateDoubleMatrix(size, 3, mxREAL);
	m_triplet = mxGetPr(m_triplet_array);

	engEvalString(m_engine, "triplets = [];");

	int i = 0;
	for (std::vector<SparseMatrixTriplet>::iterator t = triplets.begin(); t != triplets.end(); ++t)
	{
		m_triplet[i + 0] = t->row() + 1; //matlab starts index with 1
		m_triplet[i + 1 * size] = t->col() + 1; //matlab starts index with 1
		m_triplet[i + 2 * size] = t->value();
		i++;
	}

	engPutVariable(m_engine, "triplets", m_triplet_array);
	char command[255];
	sprintf_s(command, sizeof(char)*255, "%s = sparse(triplets(:,1), triplets(:,2), triplets(:,3), %d, %d);", name, d1, d2);
	engEvalString(m_engine, command);

	mxDestroyArray(m_triplet_array);
}

void MatlabDebugger::SendPositionIteration(VectorX &x, int size, double iteration)
{
	if (!m_enable)
		return;

	m_vector_array = mxCreateDoubleMatrix(size, 1, mxREAL);
	m_vector = mxGetPr(m_vector_array);

	char* name = "x";
	char command[200];
	sprintf_s(command, sizeof(char)*200, "%s = []", name);

	engEvalString(m_engine, command);

	for (int i = 0; i < size; i++)
	{
		m_vector[i] = x(i);
	}

	engPutVariable(m_engine, name, m_vector_array);

	engEvalString(m_engine, "X = [X x];");

	std::string iteration_name;
	iteration_name = std::string("it");
	m_double1x1[0] = iteration;
	engPutVariable(m_engine, "temp", m_double1x1_array);
	std::string iteration_commond;
	iteration_commond = iteration_name + std::string(" = [") + iteration_name + std::string(" temp];");
	engEvalString(m_engine, iteration_commond.c_str());
}

void MatlabDebugger::SendPositionIterationTime(VectorX &x, int size, double iteration, double time)
{
	if (!m_enable)
		return;

	m_vector_array = mxCreateDoubleMatrix(size, 1, mxREAL);
	m_vector = mxGetPr(m_vector_array);

	char* name = "x";
	char command[200];
	sprintf_s(command, sizeof(char) * 200, "%s = []", name);

	engEvalString(m_engine, command);

	for (int i = 0; i < size; i++)
	{
		m_vector[i] = x(i);
	}

	engPutVariable(m_engine, name, m_vector_array);

	engEvalString(m_engine, "X = [X x];");

	// iteration
	std::string iteration_name;
	iteration_name = std::string("it");
	m_double1x1[0] = iteration;
	engPutVariable(m_engine, "temp", m_double1x1_array);
	std::string iteration_commond;
	iteration_commond = iteration_name + std::string(" = [") + iteration_name + std::string(" temp];");
	engEvalString(m_engine, iteration_commond.c_str());


	// time
	std::string time_name;
	time_name = std::string("time");
	m_double1x1[0] = time;
	engPutVariable(m_engine, "temp", m_double1x1_array);
	std::string time_commond;
	time_commond = time_name + std::string(" = [") + time_name + std::string(" temp];");
	engEvalString(m_engine, time_commond.c_str());
}

void MatlabDebugger::SendData(VectorX &position, const double& energy, const double& gradient_norm, double iteration, double time)
{
	if (!m_enable)
		return;

	int size = position.size();

	m_vector_array = mxCreateDoubleMatrix(size, 1, mxREAL);
	m_vector = mxGetPr(m_vector_array);

	// position
	char* name = "x";
	char command[200];
	sprintf_s(command, sizeof(char) * 200, "%s = []", name);
	engEvalString(m_engine, command);
	for (int i = 0; i < size; i++)
	{
		m_vector[i] = position(i);
	}
	engPutVariable(m_engine, name, m_vector_array);
	engEvalString(m_engine, "X = [X x];");

	// energy
	std::string energy_name;
	energy_name = std::string("energy");
	m_double1x1[0] = energy;
	engPutVariable(m_engine, "temp", m_double1x1_array);
	std::string energy_commond;
	energy_commond = energy_name + std::string(" = [") + energy_name + std::string(" temp];");
	engEvalString(m_engine, energy_commond.c_str());

	// gradient norm
	std::string gradient_norm_name;
	gradient_norm_name = std::string("gradient_norm");
	m_double1x1[0] = gradient_norm;
	engPutVariable(m_engine, "temp", m_double1x1_array);
	std::string gradient_norm_commond;
	gradient_norm_commond = gradient_norm_name + std::string(" = [") + gradient_norm_name + std::string(" temp];");
	engEvalString(m_engine, gradient_norm_commond.c_str());

	// iteration
	std::string iteration_name;
	iteration_name = std::string("it");
	m_double1x1[0] = iteration;
	engPutVariable(m_engine, "temp", m_double1x1_array);
	std::string iteration_commond;
	iteration_commond = iteration_name + std::string(" = [") + iteration_name + std::string(" temp];");
	engEvalString(m_engine, iteration_commond.c_str());

	// time
	std::string time_name;
	time_name = std::string("time");
	m_double1x1[0] = time;
	engPutVariable(m_engine, "temp", m_double1x1_array);
	std::string time_commond;
	time_commond = time_name + std::string(" = [") + time_name + std::string(" temp];");
	engEvalString(m_engine, time_commond.c_str());

	m_data_filled = true;
}

void MatlabDebugger::SendEnergyIteration(double energy, double iteration)
{
	if (!m_enable)
		return;

	std::string energy_name;
	std::string iteration_name;
	energy_name = std::string("energy");
	iteration_name = std::string("it");

	std::string energy_commond;
	std::string iteration_commond;

	m_double1x1[0] = energy;
	engPutVariable(m_engine, "temp", m_double1x1_array);
	energy_commond = energy_name + std::string(" = [") + energy_name + std::string(" temp];");
	engEvalString(m_engine, energy_commond.c_str());

	m_double1x1[0] = iteration;
	engPutVariable(m_engine, "temp", m_double1x1_array);
	iteration_commond = iteration_name + std::string(" = [") + iteration_name + std::string(" temp];");
	engEvalString(m_engine, iteration_commond.c_str());
}

void MatlabDebugger::SetLastMatrixToLog(char* name, double iteration)
{
	if (!m_enable)
		return;

	if (iteration > 0)
	{
		char command[200];
		sprintf_s(command, sizeof(char) * 200, "Hessian{%lf} = %s;", iteration, name);

		engEvalString(m_engine, command);
	}
}

void MatlabDebugger::SetLastVectorToLog(char* name, double iteration)
{
	if (!m_enable)
		return;

	if (iteration > 0)
	{
		char command[200];
		sprintf_s(command, sizeof(char) * 200, "Gradient{%lf} = %s;", iteration, name);

		engEvalString(m_engine, command);
	}
}

void MatlabDebugger::SetConvergedEnergy(ScalarType energy)
{
	if (!m_enable)
		return;

	m_converged_energy_set = true;
	m_converged_energy = energy;

	m_double1x1[0] = m_converged_energy;
	engPutVariable(m_engine, "energy_converged", m_double1x1_array);
}

void MatlabDebugger::AddNewPlotData()
{
	if (m_legend_string[0] == '\0')
	{
		std::cout << "Please specify the data name." << std::endl;
		return;
	}
	if (m_plot_index >= m_plot_capacity)
	{
		std::cout << "Plot full." << std::endl;
		return;
	}
	if (!m_data_filled)
	{ 
		std::cout << "Data not filled yet." << std::endl;
		return;
	}
	char command[200];
	sprintf_s(command, sizeof(char) * 200, "plot_index = %d;", m_plot_index + 1);
	engEvalString(m_engine, command);

	engEvalString(m_engine, "diff =  X - repmat(X(:, it(end)), 1, size(it, 2));");
	engEvalString(m_engine, "dist = sqrt(sum(diff.^2));");

	engEvalString(m_engine, "dist_cell{plot_index} = dist");
	engEvalString(m_engine, "energy_cell{plot_index} = energy");
	engEvalString(m_engine, "gradient_norm_cell{plot_index} = gradient_norm");
	engEvalString(m_engine, "it_cell{plot_index} = it");
	engEvalString(m_engine, "time_cell{plot_index} = time");
	sprintf_s(command, sizeof(char) * 200, "lgd_cell{plot_index} = '%s';", m_legend_string);
	engEvalString(m_engine, command);

	m_legend_string[0] = '\0';
	m_plot_index++;
}

void MatlabDebugger::RemoveLastData()
{
	if (m_plot_index > 0)
	{
		m_plot_index--;
	}
	else
	{
		std::cout << "No data to remove." << std::endl;
	}
}

void MatlabDebugger::PlotAll()
{
	std::string plot_type;
	switch (m_plot_type)
	{
	case PLOT_TYPE_PLOT:
		plot_type = std::string("plot(");
		break;
	case PLOT_TYPE_SEMILOG_X:
		plot_type = std::string("semilogx(");
		break;
	case PLOT_TYPE_SEMILOG_Y:
		plot_type = std::string("semilogy(");
		break;
	case PLOT_TYPE_LOGLOG:
		plot_type = std::string("loglog(");
		break;
	}

	std::string lgd_command = std::string("legend(");
	char lgd[200];
	std::string y_axis_label;
	for (unsigned int i = 0; i != m_plot_index; i++)
	{
		char command[200];
		sprintf_s(command, sizeof(char) * 200, "plot_index = %d;", i + 1);
		engEvalString(m_engine, command);

		std::string x_axis_var;
		if (m_plot_x_axis_plot_iterations)
		{
			x_axis_var = std::string("it_cell{plot_index}(1:end-1)");
		}
		else
		{
			x_axis_var = std::string("time_cell{plot_index}(1:end-1)");
		}
		std::string y_axis_var;
		switch (m_plot_data_type)
		{
		case PLOT_DATA_TYPE_DISTANCE_TO_SOLUTION:
			if (m_converged_energy_set)
			{
				y_axis_var = std::string("(energy_cell{plot_index}(1:end-1)-energy_converged)/(energy_cell{plot_index}(1)-energy_converged)");
			}
			else
			{
				y_axis_var = std::string("(energy_cell{plot_index}(1:end-1)-energy_cell{plot_index}(end))/(energy_cell{plot_index}(1)-energy_cell{plot_index}(end))");
			}
			y_axis_label = std::string("ylabel('relative error');");
			break;
		case PLOT_DATA_TYPE_ENERGY:
			y_axis_var = std::string("energy_cell{plot_index}(1:end-1)");
			y_axis_label = std::string("ylabel('f(x)');");
			break;
		case PLOT_DATA_TYPE_GRADIENT_NORM:
			y_axis_var = std::string("gradient_norm_cell{plot_index}(1:end-1)");
			y_axis_label = std::string("ylabel('||f''(x)||');");
			break;
		default:
			break;
		}

		//std::string line_parameter = m_plot_style[i];
		sprintf_s(lgd, sizeof(char) * 200, "lgd_cell{%d}", i + 1);
		lgd_command = lgd_command + std::string(lgd);
		if (i < m_plot_index - 1)
		{
			lgd_command = lgd_command + std::string(", ");
		}

		std::string plot_command = plot_type\
			+ x_axis_var\
			+ std::string(", ")\
			+ y_axis_var\
			//+ std::string(", ")\
			//+ line_parameter
			+ std::string(");");

		engEvalString(m_engine, plot_command.c_str());

		if (i == 0)
		{
			engEvalString(m_engine, "hold on;");
		}
	}
	engEvalString(m_engine, "hold off;");

	// plot legend
	lgd_command = lgd_command + std::string(");");
	engEvalString(m_engine, lgd_command.c_str());

	// plot title
	engEvalString(m_engine, "title('Convergence Rate');");
	// plot xlabel and scale it
	if (m_plot_x_axis_plot_iterations)
	{
		engEvalString(m_engine, "xlabel('number of iterations');");
		engEvalString(m_engine, "xlim([0 50]);");
	}
	else
	{
		engEvalString(m_engine, "xlabel('time (seconds)');");
		engEvalString(m_engine, "xlim([0 0.3]);");
	}
	// plot ylabel
	engEvalString(m_engine, y_axis_label.c_str());
	engEvalString(m_engine, "ylim([1e-7 1]);");
}

void MatlabDebugger::Plot()
{
	switch(m_plot_data_type)
	{
	case PLOT_DATA_TYPE_DISTANCE_TO_SOLUTION:
		PlotDistToSolution();
		break;
	case PLOT_DATA_TYPE_ENERGY:
		PlotEnergy();
		break;
	case PLOT_DATA_TYPE_GRADIENT_NORM:
		PlotResidue();
		break;
	}
}

void MatlabDebugger::PlotEnergy()
{
	std::string x_axis_var = std::string("it");
	std::string y_axis_var = std::string("energy");
	std::string line_parameter = std::string("'-b'");

	std::string plot_type;
	switch (m_plot_type)
	{
	case PLOT_TYPE_PLOT:
		plot_type = std::string("plot(");
		break;
	case PLOT_TYPE_SEMILOG_X:
		plot_type = std::string("semilogx(");
		break;
	case PLOT_TYPE_SEMILOG_Y:
		plot_type = std::string("semilogy(");
		break;
	case PLOT_TYPE_LOGLOG:
		plot_type = std::string("loglog(");
		break;
	}

	std::string plot_command = plot_type\
		+ x_axis_var\
		+ std::string(", ")\
		+ y_axis_var\
		+ std::string(", ")\
		+ line_parameter\
		+ std::string(");");

	engEvalString(m_engine, plot_command.c_str());
}

void MatlabDebugger::PlotDistToSolution()
{
	engEvalString(m_engine, "diff =  X - repmat(X(:, it(end)), 1, size(it, 2));");
	engEvalString(m_engine, "dist = sqrt(sum(diff.^2));");

	std::string x_axis_var;
	if (m_plot_x_axis_plot_iterations)
	{
		x_axis_var = std::string("it(1:end-1)");
	}
	else
	{
		x_axis_var = std::string("time(1:end-1)");
	}
	std::string y_axis_var;// = std::string("(energy_cell{plot_index}(1:end-1)-energy_cell{plot_index}(end))/((energy_cell{plot_index}(1)-energy_cell{plot_index}(end)))");
	if (m_converged_energy_set)
	{
		y_axis_var = std::string("(energy(1:end-1)-energy_converged)/(energy(1)-energy_converged)");
	}
	else
	{
		y_axis_var = std::string("(energy(1:end-1)-energy(end))/(energy(1)-energy(end))");
	}
	std::string line_parameter = std::string("'-b*'");

	std::string plot_type;
	switch (m_plot_type)
	{
	case PLOT_TYPE_PLOT:
		plot_type = std::string("plot(");
		break;
	case PLOT_TYPE_SEMILOG_X:
		plot_type = std::string("semilogx(");
		break;
	case PLOT_TYPE_SEMILOG_Y:
		plot_type = std::string("semilogy(");
		break;
	case PLOT_TYPE_LOGLOG:
		plot_type = std::string("loglog(");
		break;
	}

	std::string plot_command = plot_type\
		+ x_axis_var\
		+ std::string(", ")\
		+ y_axis_var\
		+ std::string(", ")\
		+ line_parameter\
		+ std::string(");");

	engEvalString(m_engine, plot_command.c_str());
}

void MatlabDebugger::PlotResidue()
{
	std::string x_axis_var = std::string("it");
	std::string y_axis_var = std::string("gradient_norm");
	std::string line_parameter = std::string("'-b'");

	std::string plot_type;
	switch (m_plot_type)
	{
	case PLOT_TYPE_PLOT:
		plot_type = std::string("plot(");
		break;
	case PLOT_TYPE_SEMILOG_X:
		plot_type = std::string("semilogx(");
		break;
	case PLOT_TYPE_SEMILOG_Y:
		plot_type = std::string("semilogy(");
		break;
	case PLOT_TYPE_LOGLOG:
		plot_type = std::string("loglog(");
		break;
	}

	std::string plot_command = plot_type\
		+ x_axis_var\
		+ std::string(", ")\
		+ y_axis_var\
		+ std::string(", ")\
		+ line_parameter\
		+ std::string(");");

	engEvalString(m_engine, plot_command.c_str());
}

void MatlabDebugger::SetVisualizationVectorFromMatlab()
{
	if (!m_enable)
		return;

	mxArray *res = NULL;
	double *data = NULL;
	res = engGetVariable(m_engine, m_vis_name_current);
	if (res != NULL)
	{
		data = mxGetPr(res);
		//memcpy((char *)m_vis.data(), (char *)data, m_vis.size()*sizeof(double));
		for (unsigned int i = 0; i != m_vis.size(); ++i)
		{
			m_vis(i) = data[i];
		}
		m_vis_ready = true;
	}
	else
	{
		m_vis_ready = false;
	}

	mxDestroyArray(res);
}

#endif