// Define a UserInterfaceEventReceiver class which will be used to manage input
// from the GUI graphical user interface

#include "UserInterfaceEventReceiver.h" 
#include "SimulatorCES.h" 


using namespace irr;
using namespace core;
using namespace scene; 
using namespace video;
using namespace io; 
using namespace gui;



UserInterfaceEventReceiver::UserInterfaceEventReceiver(ChIrrAppInterface* myapp, SimulatorCES* mysimulator)
{
	// store pointer applicaiton
	application = myapp;
	simulator   = mysimulator;

	// ..add a GUI slider to control particles flow
	scrollbar_flow = application->GetIGUIEnvironment()->addScrollBar(
					true, rect<s32>(560, 15, 700, 15+20), 0, 101);
	scrollbar_flow->setMax(100);
	scrollbar_flow->setPos(((int)((mysimulator->emitter.ParticlesPerSecond()/1000.0 )*25)));
	text_flow = application->GetIGUIEnvironment()->addStaticText(
				L"Flow [particles/s]", rect<s32>(710,15,800,15+20), false);

	// ..add GUI slider to control the speed
	scrollbar_speed = application->GetIGUIEnvironment()->addScrollBar(
					true, rect<s32>(560, 40, 700, 40+20), 0, 102);
	scrollbar_speed->setMax(100); 
    scrollbar_speed->setPos(((int)((mysimulator->drumspeed_rpm/3.0 )*100)));
	text_speed = application->GetIGUIEnvironment()->addStaticText(
					L"Conv.vel. [m/s]:", rect<s32>(710,40,800,40+20), false);

	// ..add GUI checkmark to enable plotting forces
	checkbox_plotforces = application->GetIGUIEnvironment()->addCheckBox(false,core::rect<s32>(560,65, 560+150,65+20),
					0, 105, L"Plot applied CES forces");

	// ..add GUI checkmark to enable plotting forces
	checkbox_plottrajectories = application->GetIGUIEnvironment()->addCheckBox(false,core::rect<s32>(560,90, 560+150,90+20),
					0, 106, L"Plot trajectories");
	

}

bool UserInterfaceEventReceiver::OnEvent(const SEvent& event)
{

	// check if user moved the sliders with mouse..
	if (event.EventType == EET_GUI_EVENT)
	{
		s32 id = event.GUIEvent.Caller->getID();
		IGUIEnvironment* env = application->GetIGUIEnvironment();
		switch(event.GUIEvent.EventType)
		{
		case EGET_SCROLL_BAR_CHANGED:
				if (id == 101) // id of 'flow' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					simulator->emitter.ParticlesPerSecond() = 1000* ((double)pos/25);
					char message[50]; sprintf(message,"Flow %d [particl/s]", (int)simulator->emitter.ParticlesPerSecond());
					text_flow->setText(core::stringw(message).c_str());
				}
				if (id == 102) // id of 'speed' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					simulator->drumspeed_rpm = 3.0* (((double)pos)/100);
					simulator->drumspeed_radss = simulator->drumspeed_rpm*((2.0*CH_C_PI)/60.0); //[rad/s]
					char message[50]; sprintf(message,"Drum rpm %2.2f [m/s]", simulator->drumspeed_rpm);
					text_speed->setText(core::stringw(message).c_str());
				}
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

