// Define a UserInterfaceEventReceiver class which will be used to manage input
// from the GUI graphical user interface

#include "UserInterfaceEventReceiver.h" 
#include "ElectrostaticCoronaSeparator.h" 


using namespace irr;
using namespace core;
using namespace scene; 
using namespace video;
using namespace io; 
using namespace gui;
using namespace chrono::irrlicht;


UserInterfaceEventReceiver::UserInterfaceEventReceiver(ChIrrAppInterface* myapp, ElectrostaticCoronaSeparator* mysimulator)
{
	// store pointer applicaiton
	application = myapp;
	simulator   = mysimulator;

	// ..add a GUI slider to control particles flow
	scrollbar_flow = application->GetIGUIEnvironment()->addScrollBar(
					true, rect<s32>(560, 15, 700, 15+20), nullptr, 101);
    scrollbar_flow->setMax(5000);
	//scrollbar_flow->setPos(mysimulator->emitter.ParticlesPerSecond());
	scrollbar_flow->setPos(simulator->particle_flow);
	text_flow = application->GetIGUIEnvironment()->addStaticText(
				L"Flow [particles/s]", rect<s32>(710,15,800,15+20), false);

	// ..add GUI slider to control the speed
	scrollbar_speed = application->GetIGUIEnvironment()->addScrollBar(
					true, rect<s32>(560, 40, 700, 40+20), nullptr, 102);
	scrollbar_speed->setMax(100); 
    scrollbar_speed->setPos(simulator->GetDrumSpeed()/simulator->drumspeed_rpm_max*100);
	text_speed = application->GetIGUIEnvironment()->addStaticText(
					L"Conv.vel. [m/s]:", rect<s32>(710,40,800,40+20), false);

	// ..add GUI checkmark to enable plotting forces
	checkbox_plotECSforces = application->GetIGUIEnvironment()->addCheckBox(false,core::rect<s32>(560,65, 560+150,65+20),
					0, 105, L"Plot applied CES forces");

	// ..add GUI checkmark to enable plotting forces
	checkbox_plottrajectories = application->GetIGUIEnvironment()->addCheckBox(false,core::rect<s32>(560,90, 560+150,90+20),
					0, 106, L"Plot trajectories");

    //editbox_ECSforces_scalefactor = application->GetIGUIEnvironment()->addEditBox(L"scale", core::rect<s32>(560, 115, 560 + 50, 115 + 20), true, nullptr, 103);
    //text_ECSforces = application->GetIGUIEnvironment()->addStaticText(
    //    L"ECS forces scalefactor", rect<s32>(560 + 70, 115, 560 + 70+ 90, 15 + 20), false);
	

}

bool UserInterfaceEventReceiver::OnEvent(const SEvent& event)
{

	// check if user moved the sliders with mouse..
	if (event.EventType == EET_GUI_EVENT)
	{
		s32 id = event.GUIEvent.Caller->getID();

		//IGUIEnvironment* env = application->GetIGUIEnvironment();
		switch(event.GUIEvent.EventType)
		{
		case EGET_SCROLL_BAR_CHANGED:
				if (id == 101) // id of 'flow' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					//simulator->emitter.ParticlesPerSecond() = 1000* ((double)pos/25);
					simulator->particle_flow = pos;
					//char message[50]; sprintf(message,"Flow %d [particl/s]", (int)simulator->emitter.ParticlesPerSecond());
					char message[50]; sprintf(message,"Flow %d [p/s]", static_cast<int>(simulator->particle_flow));
					text_flow->setText(core::stringw(message).c_str());
				}
				if (id == 102) // id of 'speed' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                    simulator->SetDrumSpeed(pos * simulator->drumspeed_rpm_max/100);
                    std::cout << "Drum speed: " << simulator->GetDrumSpeed() << " rpm" << std::endl;
					char message[50]; sprintf(message,"Drum speed %2.2f [rpm]", simulator->drumspeed_rpm);
					text_speed->setText(core::stringw(message).c_str());
				}
		break;

        //case EGET_EDITBOX_ENTER:
        //    if (id == 103)
        //    {
        //        simulator->ECSforces_scalefactor = atof(irr::core::stringc(((irr::gui::IGUIEditBox*)event.GUIEvent.Caller)->getText()).c_str());
        //        break;
        //    }
        //    break;

        default:
            break;
		}

        
	} 




	// check if user presses keys
	if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown)
	{
		irr::core::matrix4 matrix;
		switch (event.KeyInput.Key)
		{
		case irr::KEY_F1:	// camera will point to drum
			application->GetSceneManager()->getActiveCamera()->setPosition( vector3dfCH( simulator->drum_csys.pos + ChVector<>(0,0.4,0.4) ) );
			application->GetSceneManager()->getActiveCamera()->setTarget  ( vector3dfCH( simulator->drum_csys.pos ) );
			application->GetSceneManager()->getActiveCamera()->setFOV((float)(36*chrono::CH_C_DEG_TO_RAD));
			return true;
		case irr::KEY_F2:	// camera will point to nozzle
			application->GetSceneManager()->getActiveCamera()->setPosition( vector3dfCH( simulator->nozzle_csys.pos + ChVector<>(0.2,0.3,0.4) ) );
			application->GetSceneManager()->getActiveCamera()->setTarget  ( vector3dfCH( simulator->nozzle_csys.pos ) );
			application->GetSceneManager()->getActiveCamera()->setFOV((float)(36*chrono::CH_C_DEG_TO_RAD));
			return true;
		case irr::KEY_F3:	// camera will point to bins
			application->GetSceneManager()->getActiveCamera()->setPosition( vector3dfCH( simulator->drum_csys.pos + ChVector<>(0.8, 0.2, 0.9) ) );
			application->GetSceneManager()->getActiveCamera()->setTarget  ( vector3dfCH( simulator->drum_csys.pos + ChVector<>(0.2f,-0.8f, 0.f  ) ) );
			application->GetSceneManager()->getActiveCamera()->setFOV((float)(36*chrono::CH_C_DEG_TO_RAD));
			return true;
		case irr::KEY_F4:	// camera will point to drum in orthographic projection
			application->GetSceneManager()->getActiveCamera()->setPosition( vector3dfCH( simulator->drum_csys.pos + ChVector<>(0.0,0.0, 4) ) );
			application->GetSceneManager()->getActiveCamera()->setTarget  ( vector3dfCH( simulator->drum_csys.pos ) );	
			matrix.buildProjectionMatrixOrthoLH(0.4f, 0.3f, 0.3f, 100.f);
			application->GetSceneManager()->getActiveCamera()->setProjectionMatrix(matrix,true);
			return true;
		case irr::KEY_F5:	// camera will point to drum in orthographic projection (closeup)
			application->GetSceneManager()->getActiveCamera()->setPosition( vector3dfCH( simulator->nozzle_csys.pos + ChVector<>(0.0,0.0, 4) ) );
			application->GetSceneManager()->getActiveCamera()->setTarget  ( vector3dfCH( simulator->nozzle_csys.pos ) );	
			matrix.buildProjectionMatrixOrthoLH(0.2f*0.4f, 0.2f*0.3f, 0.2f, 100.f);
			application->GetSceneManager()->getActiveCamera()->setProjectionMatrix(matrix,true);
			return true;

		default:
			break;
		}
	}


	return false;
}

