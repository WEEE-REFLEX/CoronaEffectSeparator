#ifndef USERINTERFACEEVENTRECEIVER_H
#define USERINTERFACEEVENTRECEIVER_H

// Define a UserInterfaceEventReceiver class which will be used to manage input
// from the GUI graphical user interface

#include "chrono_irrlicht/CHirrApp.h"
#include <array>


// Forward declaration
class ElectrostaticCoronaSeparator;



class UserInterfaceEventReceiver : public irr::IEventReceiver
{
public:

	UserInterfaceEventReceiver(chrono::irrlicht::ChIrrAppInterface* myapp, ElectrostaticCoronaSeparator* mysimulator);

	bool OnEvent(const irr::SEvent& event) override;

private: 
	chrono::irrlicht::ChIrrAppInterface* application;
    ElectrostaticCoronaSeparator* simulator;

	std::array<int, 2> screen_dim = {1200, 900};

public:
	irr::gui::IGUIScrollBar*  scrollbar_flow;
	irr::gui::IGUIStaticText* text_flow;
	irr::gui::IGUIScrollBar*  scrollbar_speed;
	irr::gui::IGUIStaticText* text_speed;
	irr::gui::IGUICheckBox*	checkbox_plotECSforces;
	irr::gui::IGUICheckBox*	checkbox_plottrajectories;
    //irr::gui::IGUIEditBox* editbox_ECSforces_scalefactor;
    //irr::gui::IGUIStaticText* text_camera_pos;
};



#endif
