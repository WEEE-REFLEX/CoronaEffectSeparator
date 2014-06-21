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
	// If the .exe is launched normally, by default it will parse the settings-file below, 
	// otherwise the user can launch it by command-line by passing the filename as argument.
	std::string ces_settings_filename("../CAD_conveyor/settings.ces");
	if (argc==2) 
		ces_settings_filename = argv[1];

	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	DLL_CreateGlobals();

	try
	{

		// Create a simulator object
		SimulatorCES mysimulator;

		// Load settings from file, if any
		mysimulator.ParseSettings(ces_settings_filename.c_str());

		// Initialize and execute the simulation
		mysimulator.simulate();


	}catch (ChException me)
	{
		GetLog() << "\n\n Program aborted.\n\n";
		system ("pause");
	}


	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}