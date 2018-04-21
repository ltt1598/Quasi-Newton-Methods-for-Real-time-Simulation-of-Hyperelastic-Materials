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

#ifndef _ANTTWEAKBAR_WRAPPER_H_
#define _ANTTWEAKBAR_WRAPPER_H_

#include "AntTweakBar.h"
#include "global_headers.h"

typedef enum
{
	ATB_RESHAPE_WINDOW = 0x1,
	ATB_CHANGE_TIME_STEP = 0x2,
	ATB_CHANGE_MATERIAL_PROPERTY = 0x4,
	ATB_CHANGE_INTEGRATION = 0x8,
	ATB_INIT_MATLAB = 0x10,
	ATB_CHANGE_PARTIAL_MATERIAL_PROPERTY = 0x20,
	ATB_DEFAULT = 0x0

} ATBFeedBack;

class AntTweakBarWrapper
{
public:
	AntTweakBarWrapper();
	virtual ~AntTweakBarWrapper();

	// init / cleanup / reset / save / load / default / hide / show / update / Draw / change window size
	void Init();
	void Cleanup();
	void Reset();
	void SaveSettings();
	void LoadSettings();
	void DefaultSettings();
	void Hide();
	void Show();
	int Update();
	inline void Draw() {TwDraw();}
	inline void ChangeTwBarWindowSize(int width, int height) {TwWindowSize(width, height);}

	static void TW_CALL SetDefaultSettings(void*);
	static void TW_CALL SaveSettings(void*);
	static void TW_CALL LoadSettings(void*);

protected:

	// key component: bars
	TwBar *m_control_panel_bar; // Control Panel
	TwBar *m_mesh_bar;   // Mesh Settings
	TwBar *m_sim_bar;    // Simulation Settings
};

#endif