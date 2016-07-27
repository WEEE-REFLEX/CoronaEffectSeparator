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


#include "ElectrostaticCoronaSeparator.h"

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





int main(int argc, char* argv[])
{
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);

    GetLog() << "Executing simulator \n";
    // If the .exe is launched normally, by default it will parse the settings-file below, 
    // otherwise the user can launch it by command-line by passing the filename as argument.
    //std::string ces_settings_filename("../CAD_conveyor/settings.ces");

    //if (argc == 2)
    //    ces_settings_filename = argv[1];

    // Create the Irrlicht visualization (open the Irrlicht device, 
    // bind a simple user interface, etc. etc.)
    ChSystem mphysicalSystem;
    irrlicht::ChIrrApp application(&mphysicalSystem, L"Conveyor belt", irr::core::dimension2d<irr::u32>(800, 600), false);

    bool irr_cast_shadows = true;

    // Change default font to something better
    //application.SetFonts("../objects/fonts/arial8.xml");

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(1.5f, 0.4f, -1.0f), irr::core::vector3df(0.5f, 0.f, 0.f));
    if (irr_cast_shadows)
        application.AddLightWithShadow(irr::core::vector3df(-4.5f, 5.5f, 4.5f), irr::core::vector3df(0.f, 0.f, 0.f), 10, 1.2, 10.2, 30, 512, irr::video::SColorf(1.f, 0.9f, 0.9f));

    application.SetStepManage(true);
    application.SetTimestep(0.001);

    application.GetSystem()->SetIntegrationType(ChSystem::INT_ANITESCU);
    application.GetSystem()->SetSolverType(ChSystem::SOLVER_SOR_MULTITHREAD);// SOLVER_SOR_MULTITHREAD or SOLVER_BARZILAIBORWEIN for max precision


    try
    {
        // Create a simulator object
        ElectrostaticCoronaSeparator separator(*application.GetSystem());

        //// Load settings from file, if any
        //separator.ParseSettings(ces_settings_filename.c_str());
        separator.Setup(*application.GetSystem(), &application);
        separator.LoadParticleScan("C:\\workspace\\ecs_particlescan\\input\\particlescan\\geometric_list-CU.txt");
        separator.LoadParticleScan("C:\\workspace\\ecs_particlescan\\input\\particlescan\\geometric_list-PCB.txt");
        // Initialize and execute the simulation
        separator.RunSimulation(application);

    }
    catch (ChException me)
    {
        GetLog() << "\n\n Program aborted.\n\n";
        system("pause");
    }



    return 0;
}