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


#ifndef _MATLAB_DEBUGGER_H_
#define _MATLAB_DEBUGGER_H_

#include "global_headers.h"

#define ANTTWEAKBAR_SUPPORT

#ifdef ENABLE_MATLAB_DEBUGGING

#include <string>
#include "math_headers.h"
#ifdef ANTTWEAKBAR_SUPPORT
#include "anttweakbar_wrapper.h"
#endif

#include "engine.h"

typedef enum 
{
	PLOT_TYPE_PLOT,
	PLOT_TYPE_SEMILOG_X,
	PLOT_TYPE_SEMILOG_Y,
	PLOT_TYPE_LOGLOG,
	PLOT_TYPE_TOTAL_NUM

} PlotType;

typedef enum
{
	PLOT_DATA_TYPE_DISTANCE_TO_SOLUTION,
	PLOT_DATA_TYPE_ENERGY,
	PLOT_DATA_TYPE_GRADIENT_NORM,
	PLOT_DATA_TYPE_TOTAL_NUM
} PlotDataType;

#ifdef  ANTTWEAKBAR_SUPPORT
// forward declaration
class AntTweakBarWrapper;
#endif //  ANTTWEAKBAR_SUPPORT

class MatlabDebugger
{
#ifdef  ANTTWEAKBAR_SUPPORT
	friend class AntTweakBarWrapper;
#endif //  ANTTWEAKBAR_SUPPORT
public:
	MatlabDebugger();
	~MatlabDebugger();

	void Init();
	void ResetCurrent();
	void Reset();
	void Export();
	void Import();

	void SaveCurrentData(const char* filename);
	void SaveCurrentFigure(const char* filename);

	void SendCommand(char* command);

	void SendVector(const std::vector<ScalarType> &x, char* name);
	void SendVector(const VectorX &x, char* name);
	void SendVector(const VectorX &x, char* name, int size);
	void SendSparseMatrix(const SparseMatrix& A, char* name);
	void SendSparseMatrixUsingTriplets(std::vector<SparseMatrixTriplet> &triplets, char* name, int d1, int d2);

	void SendPositionIteration(VectorX &x, int size, double iteration);
	void SendPositionIterationTime(VectorX &x, int size, double iteration, double time);
	void SendData(VectorX &position, const double& energy, const double& gradient_norm, double iteration, double time);
	void SendEnergyIteration(double energy, double iteration);
	void SetLastMatrixToLog(char* name, double iteration);
	void SetLastVectorToLog(char* name, double iteration);
	void SetConvergedEnergy(ScalarType energy);

	void AddNewPlotData();
	void RemoveLastData();
	void PlotAll();
	void Plot();

	void PlotEnergy();
	void PlotDistToSolution();
	void PlotResidue();

	// specific for this program
	// add your own code
	inline void SetVisualizationVariableName() { strcpy_s(m_vis_name_current, m_vis_name); m_vis_name[0] = '\0'; }
	void SetVisualizationVectorFromMatlab();
	inline void InitVisualizationVector(unsigned int size) { m_vis.resize(size); }
	inline const VectorX& GetVisualizationVector() { return m_vis; }
	inline bool ReadyToVisualize() { return m_vis_ready; }
	inline ScalarType VisualizationAmplifer() { return m_vis_amplify; }

	// accesser
	bool IsEnabled() { return m_enable; };
	Engine* GetEngine() { return m_engine; }

protected:

	// settings for debugger
	bool m_enable;

	// engine related 
	Engine* m_engine;

	// data related
	mxArray *m_double1x1_array;
	mxArray *m_triplet_array;
	mxArray *m_vector_array;
	double *m_double1x1;
	double *m_triplet;
	double *m_vector;
	bool m_data_filled;

	bool m_converged_energy_set;
	double m_converged_energy;

	// plot related
	PlotType m_plot_type;
	PlotDataType m_plot_data_type;
	char m_legend_string[255];
	const int m_plot_capacity = 20;
	//std::vector<std::string> m_plot_style;
	bool m_plot_x_axis_plot_iterations; // true = iterations, false = time 
	int m_plot_index;

	// specific for this program
	// add your own code
	char m_vis_name_current[255];
	char m_vis_name[255];
	bool m_vis_ready;
	ScalarType m_vis_amplify;
	VectorX m_vis;

};

#endif

#endif