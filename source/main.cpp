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
#include "SerialComm.h"

using namespace chrono;


int main(int argc, char* argv[])
{
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);

    GetLog() << "Executing simulator \n";

    ChSystem mphysicalSystem;
    irrlicht::ChIrrApp application(&mphysicalSystem, L"Conveyor belt", irr::core::dimension2d<irr::u32>(800, 600), false);

    auto irr_cast_shadows = true;

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    //application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(0.5f, 0.25f, -0.5f), irr::core::vector3df(0.1f, 0.f, 0.f));
    if (irr_cast_shadows)
        application.AddLightWithShadow(irr::core::vector3df(+4.5f, 5.f, -4.f), irr::core::vector3df(0.f, 0.f, 0.f), 12.5, 1.2, 10.2, 30, 512, irr::video::SColorf(1.f, 1.f, 1.f));

    application.SetStepManage(true);
    application.SetTimestep(0.001);

    application.GetSystem()->SetIntegrationType(ChSystem::INT_ANITESCU);
    application.GetSystem()->SetSolverType(ChSystem::SOLVER_SOR_MULTITHREAD);// SOLVER_SOR_MULTITHREAD or SOLVER_BARZILAIBORWEIN for max precision

    //try{
    //    CallbackAsyncSerial prova("COM15", 115200);
    //    char* ciao = "ciao ins\n";
    //    prova.write(ciao, 9);
    //}
    //catch(...)
    //{
    //    std::cout << "bad done" << std::endl;
    //}

    //CallbackAsyncSerial prova("COM15", 115200);
    //char* ciao = "ciao out\n";
    //prova.write(ciao, 9);
    //prova.close();

    

    try
    {
        // Create a simulator object
        ElectrostaticCoronaSeparator separator(*application.GetSystem());
        EcsSerialCommunicator serial("COM15",115200);
        separator.BindToEcsSerialCommunicator(serial);

        //// Load settings from file, if any
        //separator.ParseSettings(ces_settings_filename.c_str());
        separator.Setup(*application.GetSystem(), &application);
        separator.LoadParticleScan("C:\\workspace\\ces\\input\\particlescan\\geometric_list-CU.txt");
        separator.LoadParticleScan("C:\\workspace\\ces\\input\\particlescan\\geometric_list-PCB.txt");
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