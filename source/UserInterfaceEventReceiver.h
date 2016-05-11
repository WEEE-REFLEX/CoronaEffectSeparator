#ifndef USERINTERFACEEVENTRECEIVER_H
#define USERINTERFACEEVENTRECEIVER_H

// Define a UserInterfaceEventReceiver class which will be used to manage input
// from the GUI graphical user interface

#include "chrono_irrlicht/CHirrApp.h"



// Forward declaration
class SimulatorCES;



class UserInterfaceEventReceiver : public irr::IEventReceiver
{
public:

	UserInterfaceEventReceiver(chrono::irrlicht::ChIrrAppInterface* myapp, SimulatorCES* mysimulator);

	bool OnEvent(const irr::SEvent& event);

private: 
	chrono::irrlicht::ChIrrAppInterface* application;
	SimulatorCES* simulator;

public:
	irr::gui::IGUIScrollBar*  scrollbar_flow;
	irr::gui::IGUIStaticText* text_flow;
	irr::gui::IGUIScrollBar*  scrollbar_speed;
	irr::gui::IGUIStaticText* text_speed;
	irr::gui::IGUICheckBox*	checkbox_plotforces;
	irr::gui::IGUICheckBox*	checkbox_plottrajectories;
};



#endif
