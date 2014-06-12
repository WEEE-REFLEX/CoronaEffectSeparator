///////////////////////////////////////////////////
//
//   Corona Electrostatic Separator
//
//   This program is based on the following
//   libraries:
//   - ChronoEngine
//   - Irrlicht
// 
//  
// ------------------------------------------------ 
///////////////////////////////////////////////////
 

#include "SimulatorCES.h"

// Use the namespace of Chrono

using namespace chrono;
using namespace postprocess;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace core;
using namespace scene; 
using namespace video;
using namespace io; 
using namespace gui;
using namespace std;

// Static values valid through the entire program (bad
// programming practice, but enough for quick tests)



int main(int argc, char* argv[])
{
	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	DLL_CreateGlobals();


	// Create a simulator object
	SimulatorCES mysimulator;

	// Load settings from file, if any
	mysimulator.ParseSettings("../CAD_conveyor/settings.ces");

	// Initialize and execute the simulation
	mysimulator.simulate();


	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}