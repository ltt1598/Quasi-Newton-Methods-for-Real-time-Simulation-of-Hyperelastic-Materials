Quasi-Newton Methods for Real-time Simulation of Hyperelastic Materials.

1. The source code is provided AS IS. The author is not responsible for any compilation issues.
2. The code is not fully optimized due to the author's ability of coding. Feel free to accelerate it :)
3. The project containing is source code is currently only a Visual Studio project.
4. All the 3-rd party library dependencies in the Libs folder. The default path for all the dependencies is D:\Libs in the visual studio project.
5. Basic Keyboard shortcuts:
	a.) Camera Rotation: Alt+LeftMouseButton; Camera Panning: Alt+MiddleMouseButton; Camera Zooming: Alt:RightMouseButton
	b.) Select Vertices: Hold LeftMouseButton and drag; Remove Selected Vertices: Hold RightMouseButton and drag; Appending Vertices to Selection: Hold MiddleMouseButton and drag
	c.) Mouse Mode Selection: 'q'; Mouse Mode Translating: 'w'; Mouse Mode Rotation: 'r'
	d.) Set Selected Vertices as a Handle: 'h'
	e.) Run/Pause Simulation: 'space'
	f.) Step Simulation One Frame Forward: 'p'
	g.) Save Current State: 's'; Load the Last Saved State: 'l';
	h.) See more keyboard shortcuts in "void key_press(unsigned char key, int x, int y)" in main.cpp
6. Look for "void Simulation::Update()" in Simulation.cpp for the basic workflow of our method.

Thank you for being interested in our work.

Sincerely,
Tiantian Liu

08/14/2017