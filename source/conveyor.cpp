///////////////////////////////////////////////////
//
//   Eddy Current Separator
//
//   This program is based on the following
//   libraries:
//   - ChronoEngine
//   - Irrlicht
// 
//  
// ------------------------------------------------ 
///////////////////////////////////////////////////
 
    
#include "physics/CHapidll.h" 
#include "physics/CHsystem.h"
#include "physics/CHconveyor.h"
#include "irrlicht_interface/CHbodySceneNode.h"
#include "irrlicht_interface/CHbodySceneNodeTools.h" 
#include "irrlicht_interface/CHirrApp.h"
#include "core/CHrealtimeStep.h"
#include "core/CHmath.h"
#include <irrlicht.h>
#include <fstream>
#include "unit_PYTHON/ChPython.h"


// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;
         
using namespace core;
using namespace scene; 
using namespace video;
using namespace io; 
using namespace gui;
//
using namespace std;

// Static values valid through the entire program (bad
// programming practice, but enough for quick tests)

double STATIC_flow = 1; 
double STATIC_speed = 0.87; //[m/s]

const double mu0 = 0.0000012566; //vacuum permability [Tm/A]
const double drumspeed = 261; //[rad/s]
const double numberofpoles = 9;
const double intensity = 0.32; 
const double drumdiameter = 0.30;
// conveyor constant
const double conveyor_length=2;
const double conveyor_width=0.6;
const double conv_thick = 0.00005;

const double ro=1.225;  //fluid density (air) [Kg/m^3]

// fence constant
const double fence_width = 0.02;
const double fence_height=0.2;
// bin constant 
const double y_posbin=-0.815;
const double binbase=0.02; // base thickness of the bin
const double bin_length=3; 
const double bin_width=1.5;
const double bin_height=0.2;
const double n=0; // number of bins (values from 2 to 4)
// splitter constant
const double x_splitter1=0;
const double x_splitter2=0;
const double x_splitter3=0;
const double splitter_width=0.01;
const double splitter_height=0.4;
// hopper constant
const double xnozzlesize = 0.2;
const double znozzlesize = 0.3;
const double ynozzlesize = 0.5;
const double ynozzle = 0.6;
const double xnozzle = -conveyor_length/2+xnozzlesize/2+fence_width;

// set as true for saving log files each n frames
bool savefile = false;
int saveEachNframes = 10;

int totframes = 0;
	


// If one needs to add special data to a particle/object, the proper way to
// do this is by inheriting a custom class from the base ChAsset. 
// An asset of this class can then be created and attached to the
// particle (see below).  ***ALEX

class ElectricParticleProperty : public ChAsset
{ 
public:

	// data for this type of asset 
	ChVector<> Cdim;
	double conductivity;
	double birthdate;
	enum 
	{
		e_fraction_box,
		e_fraction_cylinder,
		e_fraction_sphere,
		e_fraction_others
	} fraction;
	enum 
	{
		e_mat_plastic,
		e_mat_metal,
		e_mat_other
	} material_type;

	// default constructor with initialization
	ElectricParticleProperty()
	{
		Cdim = ChVector<>(1,1,1);
		conductivity = 0;
		birthdate = 0;
		fraction = ElectricParticleProperty::e_fraction_others;
		material_type = ElectricParticleProperty::e_mat_other;
	}
	
};




// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface

class MyEventReceiver : public IEventReceiver
{
public:

	MyEventReceiver(ChIrrAppInterface* myapp)
			{
				// store pointer applicaiton
				application = myapp;

				// ..add a GUI slider to control particles flow
				scrollbar_flow = application->GetIGUIEnvironment()->addScrollBar(
								true, rect<s32>(510, 85, 650, 100), 0, 101);
				scrollbar_flow->setMax(300); 
				scrollbar_flow->setPos(150);
				text_flow = application->GetIGUIEnvironment()->addStaticText(
							L"Flow [particles/s]", rect<s32>(650,85,750,100), false);

				// ..add GUI slider to control the speed
				scrollbar_speed = application->GetIGUIEnvironment()->addScrollBar(
								true, rect<s32>(510, 125, 650, 140), 0, 102);
				scrollbar_speed->setMax(100); 
				scrollbar_speed->setPos(100);
				text_speed = application->GetIGUIEnvironment()->addStaticText(
								L"Conveyor speed [m/s]:", rect<s32>(650,125,750,140), false);
			}

	bool OnEvent(const SEvent& event)
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
								STATIC_flow = (double)pos;
							}
							if (id == 102) // id of 'speed' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								STATIC_speed = (((double)pos)/100)*2;
							}
					break;
					}
					
				} 

				return false;
			}

private: 
	ChIrrAppInterface* application;

	IGUIScrollBar*  scrollbar_flow;
	IGUIStaticText* text_flow;
	IGUIScrollBar*  scrollbar_speed;
	IGUIStaticText* text_speed;
};




// Function that creates debris that fall on the conveyor belt, to be called at each dt

void create_debris(double dt, double particles_second, 
				   ChSystem& mysystem, 
				   ChIrrApp* irr_application)
{

	double sph_fraction = 0;//0.3; // 30% cubes
	double box_fraction = 0;//0.4; // 40% cylinders
	double cyl_fraction = 1-box_fraction-sph_fraction;

	double density = 1820;
	double sphrad = 0.01;
	double cylhei = 0.035;
	double cylmass = 0.0095; //CH_C_PI*pow(sphrad,2)*cylhei*density;
	double sphmass = 0.0095;//1.3333*CH_C_PI*pow(sphrad,3)*density;
	double sphinertia = 0.4*pow(sphrad,2)*sphmass;
	double cylinertia = 0.0833*(pow(cylhei,2)+3*pow(sphrad,2))*cylmass;//0.0833*(pow(cylhei,2)+3*pow(sphrad,2))*cylmass;
	double cylinertia2 = 0.5*pow(sphrad,2)*cylmass; //0.5*pow(sphrad,2)*cylmass;
	
	float conductivity;

	double exact_particles_dt = dt * particles_second;
	double particles_dt = floor(exact_particles_dt);
	double remaind = exact_particles_dt - particles_dt;
	
	if (remaind > ChRandom()) particles_dt +=1;

	for (int i = 0; i < particles_dt; i++)
	{
		ChSharedPtr<ChBody> created_body;

		double rand_fract = ChRandom();

		double rand_mat = ChRandom();

		double plastic_fract=0.3;
	
		if (rand_mat < plastic_fract)
		{
			conductivity = 0;
		}
		if (rand_mat > plastic_fract)
		{
			conductivity = 6670000;
		}
		if (rand_fract < sph_fraction)
		{
			// Create a body
			ChSharedPtr<ChBody> mrigidBody(new ChBody);

			mrigidBody->SetPos(ChVector<>(-0.5*xnozzlesize+ChRandom()*xnozzlesize+xnozzle, conv_thick/2+sphrad, -0.5*znozzlesize+ChRandom()*znozzlesize));
			mrigidBody->SetMass(sphmass);
			mrigidBody->SetInertiaXX(ChVector<>(sphinertia,sphinertia,sphinertia));
			mrigidBody->SetFriction(0.2f);
			mrigidBody->SetImpactC(0.8f); 
			//mrigidBody->GetCollisionModel()->SetFamily(5);
			//mrigidBody->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);

			// Define a collision shape 
			mrigidBody->GetCollisionModel()->ClearModel();
			mrigidBody->GetCollisionModel()->AddSphere(sphrad);
			mrigidBody->GetCollisionModel()->BuildModel();
			mrigidBody->SetCollide(true);

			// Attach a visualization shape asset. 
			ChSharedPtr<ChSphereShape> msphere(new ChSphereShape);
			msphere->GetSphereGeometry().rad = sphrad;
			msphere->GetSphereGeometry().center = ChVector<>(0,0,0);
			mrigidBody->AddAsset(msphere);
			// Attach a visualization texture
			ChSharedPtr<ChTexture> mtexture(new ChTexture);
			mtexture->SetTextureFilename("../objects/bluwhite.png");
			mrigidBody->AddAsset(mtexture);

			// Attach a custom asset. Look how to create and add a custom asset to the object! ***ALEX
			ChSharedPtr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty); 
			electric_asset->Cdim         = ChVector<>(2*sphrad,2*sphrad,2*sphrad);
			electric_asset->conductivity = conductivity;
			electric_asset->fraction	 = ElectricParticleProperty::e_fraction_sphere;
			electric_asset->birthdate	 = mysystem.GetChTime();
			mrigidBody->AddAsset(electric_asset);
			
			// Finally, do not forget to add the body to the system:
			mysystem.Add(mrigidBody);

			// If Irrlicht is used, setup also the visualization proxy:
			if (irr_application)
			{
				irr_application->AssetBind(mrigidBody);
				irr_application->AssetUpdate(mrigidBody);
			}
			created_body = mrigidBody;
		}

		if ((rand_fract > sph_fraction) && 
			(rand_fract < box_fraction+sph_fraction))
		{
			double xscale = 1.3*(1-0.8*ChRandom()); // for oddly-shaped boxes..
			double yscale = 1.3*(1-0.8*ChRandom());
			double zscale = 1.3*(1-0.8*ChRandom());

			//	ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
			//	ChCollisionModel::SetDefaultSuggestedMargin  (0.002);

			// Create a body
			ChSharedPtr<ChBody> mrigidBody(new ChBody);

			mrigidBody->SetPos(ChVector<>(-0.5*xnozzlesize+ChRandom()*xnozzlesize+xnozzle, ynozzle+i*0.005, -0.5*znozzlesize+ChRandom()*znozzlesize));
			mrigidBody->SetMass(sphmass);
			mrigidBody->SetInertiaXX(ChVector<>(sphinertia,sphinertia,sphinertia));
			mrigidBody->SetFriction(0.4f);
			mrigidBody->SetImpactC(0.0f); 

			// Define a collision shape 
			mrigidBody->GetCollisionModel()->ClearModel();
			mrigidBody->GetCollisionModel()->AddBox(sphrad*2*xscale, sphrad*2*yscale, sphrad*2*yscale);
			mrigidBody->GetCollisionModel()->BuildModel();
			mrigidBody->SetCollide(true);

			// Attach a visualization shape asset. 
			ChSharedPtr<ChBoxShape> mbox(new ChBoxShape);
			mbox->GetBoxGeometry().SetLenghts(ChVector<>(sphrad*2*xscale, sphrad*2*yscale, sphrad*2*yscale));
			mrigidBody->AddAsset(mbox);
			// Attach a visualization texture
			ChSharedPtr<ChTexture> mtexture(new ChTexture);
			mtexture->SetTextureFilename("../objects/bluwhite.png");
			mrigidBody->AddAsset(mtexture);

			// Attach a custom asset. Look how to create and add a custom asset to the object! ***ALEX
			ChSharedPtr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty); 
			electric_asset->Cdim         = ChVector<>(2*sphrad,2*sphrad,2*sphrad);
			electric_asset->conductivity = conductivity;
			electric_asset->fraction	 = ElectricParticleProperty::e_fraction_box;
			electric_asset->birthdate	 = mysystem.GetChTime();
			mrigidBody->AddAsset(electric_asset);
			
			// Finally, do not forget to add the body to the system:
			mysystem.Add(mrigidBody);

			// If Irrlicht is used, setup also the visualization proxy:
			if (irr_application)
			{
				irr_application->AssetBind(mrigidBody);
				irr_application->AssetUpdate(mrigidBody);
			}
			created_body = mrigidBody;
		}

		if (rand_fract > box_fraction+sph_fraction)
		{

			//	ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
			//	ChCollisionModel::SetDefaultSuggestedMargin  (0.002);

			// Create a body
			ChSharedPtr<ChBody> mrigidBody(new ChBody);

			mrigidBody->SetPos(ChVector<>(-0.5*xnozzlesize+ChRandom()*xnozzlesize+xnozzle, ynozzle+i*0.005, -0.5*znozzlesize+ChRandom()*znozzlesize));
			mrigidBody->SetMass(cylmass);
			mrigidBody->SetInertiaXX(ChVector<>(cylinertia,cylinertia2,cylinertia));
			mrigidBody->SetFriction(0.4f);
			mrigidBody->SetImpactC(0.0f); 

			// Define a collision shape 
			mrigidBody->GetCollisionModel()->ClearModel();
			mrigidBody->GetCollisionModel()->AddCylinder(sphrad,sphrad,cylhei);
			mrigidBody->GetCollisionModel()->BuildModel();
			mrigidBody->SetCollide(true);

			// Attach a visualization shape asset. 
			ChSharedPtr<ChCylinderShape> mcyl(new ChCylinderShape);
			mcyl->GetCylinderGeometry().rad = sphrad;
			mcyl->GetCylinderGeometry().p1 = ChVector<>(0, cylhei/2,0);
			mcyl->GetCylinderGeometry().p2 = ChVector<>(0,-cylhei/2,0);
			mrigidBody->AddAsset(mcyl);
			// Attach a visualization texture
			ChSharedPtr<ChTexture> mtexture(new ChTexture);
			mtexture->SetTextureFilename("../objects/bluwhite.png");
			mrigidBody->AddAsset(mtexture);

			// Attach a custom asset. Look how to create and add a custom asset to the object! ***ALEX
			ChSharedPtr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty); 
			electric_asset->Cdim         = ChVector<>(sphrad*2,cylhei,sphrad*2);
			electric_asset->conductivity = conductivity;
			electric_asset->fraction	 = ElectricParticleProperty::e_fraction_cylinder;
			electric_asset->birthdate	 = mysystem.GetChTime();
			mrigidBody->AddAsset(electric_asset);
			
			// Finally, do not forget to add the body to the system:
			mysystem.Add(mrigidBody);

			// If Irrlicht is used, setup also the visualization proxy:
			if (irr_application)
			{
				irr_application->AssetBind(mrigidBody);
				irr_application->AssetUpdate(mrigidBody);
			}
			created_body = mrigidBody;
		}

		// This is an optional hack that largely affects the stability of the
		// simulation. 
		// In fact, if particles happen to spin too fast, the collision detection
		// starts to be very slow, and maybe also inaccurate; also, the time integration
		// could diverge. To get rid of this problem wihtout reducing too much the timestep, 
		// one can enable a limit on angular velocity and/or linear velocity. NOTE that 
		// this achieves greater stability at the cost of lower realism of the simulation, 
		// so it should not be abused. ***ALEX

		bool do_velocity_clamping = true;

		if (!created_body.IsNull() && do_velocity_clamping)
		{
			created_body->SetLimitSpeed(true);
			created_body->SetMaxSpeed(100);
			created_body->SetMaxWvel(250);
		}

	}

} 
     
  
// Function that deletes old debris (to avoid infinite creation that fills memory)

void purge_debris(ChSystem& mysystem, double max_age = 5.0)
{
	for (unsigned int i=0; i<mysystem.Get_bodylist()->size(); i++)
	{
		ChBody* abody = (*mysystem.Get_bodylist())[i];

		bool to_delete = false;

		// Fetch the ElectricParticleProperty asset from the list of 
		// assets that have been attached to the object, and retrieve the
		// custom data that have been stored. ***ALEX
		for (unsigned int na= 0; na< abody->GetAssets().size(); na++)
		{
			ChSharedPtr<ChAsset> myasset = abody->GetAssetN(na);
			if (myasset.IsType<ElectricParticleProperty>())
			{
				ChSharedPtr<ElectricParticleProperty> electricproperties = myasset;
				double particle_birthdate  = electricproperties->birthdate ;
				double particle_age = mysystem.GetChTime() - particle_birthdate;
				if (particle_age > max_age)
				{
					to_delete = true;
				}
			}
		}

		if (to_delete)
		{
			abody->AddRef();	// dirty trick to convert basic pointer to..
			ChSharedPtr<ChBody> mysharedbody(abody); // ..shared pointer

			mysystem.Remove(mysharedbody);
			mysharedbody->RemoveRef(); //***???? shouldn't be needed..
			i--; // this because if deleted, the rest of the array is shifted back one position..
		}
	}

}

// Function that defines the forces on the debris

void apply_forces (	ChSystem* msystem,		// contains all bodies
						ChCoordsys<>& drum_csys, // pos and rotation of drum 
						double drumspeed,		 // speed of drum
						double numberofpoles,	 // number of couples of poles
						double intensity,		 // intensity of the magnetic field
						double drumdiameter,
						int totframes)		
{
	char padnumber[100];
	char filename[200];
	sprintf(padnumber, "%d", (totframes+10000));
	sprintf(filename, "output\\forces%s.dat", padnumber+1);
	//ChStreamOutAsciiFile data_forces(filename);
	ofstream test;
	test.open("output\\test.dat",ios::app); 

	for (unsigned int i=0; i<msystem->Get_bodylist()->size(); i++)
	{
		ChBody* abody = (*msystem->Get_bodylist())[i];

		ChVector<> diam = VNULL; // default values, to be set few lines below...
		double sigma = 0;

		// Fetch the ElectricParticleProperty asset from the list of 
		// assets that have been attached to the object, and retrieve the
		// custom data that have been stored. ***ALEX
		for (unsigned int na= 0; na< abody->GetAssets().size(); na++)
		{
			ChSharedPtr<ChAsset> myasset = abody->GetAssetN(na);
			if (myasset.IsType<ElectricParticleProperty>())
			{
				ChSharedPtr<ElectricParticleProperty> electricproperties = myasset;
				diam  = electricproperties->Cdim ;
				sigma = electricproperties->conductivity;
			}
		}

		// Remember to reset 'user forces accumulators':
		abody->Empty_forces_accumulators();

		// initialize speed of air (steady, if outside fan stream): 
		ChVector<> abs_wind(0,0,0);

		// calculate the position of body COG with respect to the drum COG:
		ChVector<> mrelpos = drum_csys.TrasformParentToLocal(abody->GetPos());
		double distx=mrelpos.x;
		double disty=-mrelpos.z;
		ChVector<> velocity=abody->GetPos_dt();
		double velocityx=velocity.x;
		double velocityy=velocity.y;
		double velocityz=velocity.z;
		ChVector <> rot_speed=abody->GetWvel_par();
		double rot_speedz=rot_speed.z; //bisogna tirare fuori la componente attorno all'asse z globale della velocità di rotazione

		double velocity_norm_sq=velocity.Length2();

		//ChQuaternion<> rot_velocity=abody->GetRot_dt;
		double distance = pow(distx*distx+disty*disty,0.5);//mrelpos.Length();
		double phi = atan2(disty,distx);
		double phi2 = atan2(-velocity.y,velocity.x);
		double B = intensity*pow((drumdiameter/2/distance),numberofpoles+1);
		double shape = abody->GetCollisionModel()->GetShapeType();
		ChMatrix33<> RotM = *abody->GetA();
		ChVector<> yG = RotM.Matr_x_Vect(VECT_Y);
		ChVector<> particleRspeed = abody->GetWvel_par();
		ChVector<> particleRspeed_loc = abody->GetWvel_loc();
		int a=0;
		int b=abody->GetIdentifier();
		double rho = acos(yG.y/yG.Length());
		
		

		if (rho > CH_C_PI_2)
		{
			rho = CH_C_PI-rho;
		}
		double alpha = rho/CH_C_PI_2;

		// check velocità rotazione particelle
	
		double constR;
		double constI;
		//double constTorque;
		double Volume,d;

		
		if (shape == 0) //sphere
		{
			d=diam.x;
			double csi = mu0*d*d* sigma *(drumspeed*numberofpoles+particleRspeed.z);
			constR = 21*pow(csi,2)/(20*(1764+pow(csi,2)));
			constI = 21*42*csi/(20*(1764+pow(csi,2)));
			//constTorque = 1/40*pow(diam.x,2)*(4/3*CH_C_PI*pow(diam.x/2,3))* sigma*(drumspeed*numberofpoles+particleRspeed.z);
			Volume = 1.3333*CH_C_PI*pow(diam.x/2,3);
			//constTorque = -pow(B,2)*4/3*CH_C_PI*pow(diam.x/2,3)*constI/mu0;


			//data_forces << csi << ",\t\t\t";
		}
		else if (shape == 2) // box (as a sphere)
		{
			d=diam.x;
			double csi = mu0*pow(diam.x,2)* sigma*(drumspeed*numberofpoles+particleRspeed.z);
			constR = 21*pow(csi,2)/20/(1764+pow(csi,2));
			constI = 21*42*csi/20/(1764+pow(csi,2));
			//constTorque = 1/40*pow(diam.x,2)*(4/3*CH_C_PI*pow(diam.x/2,3))* sigma*(drumspeed*numberofpoles+particleRspeed.z);
			Volume = pow(diam.x,3);
			//constTorque = -pow(B,2)*pow(diam.x,3)*constI/mu0;

			//data_forces << csi << ",\t\t\t";
		}
		else if (shape == 3) // cylinder
		{
			if (diam.y<diam.x) // disk
			{
				d=diam.x; //se togliamo alpha=0, cambiarlo...
				double csi_par = mu0*pow(diam.y,2)* sigma*(drumspeed*numberofpoles+particleRspeed.z);
				double csi_per = mu0*pow(diam.x,2)* sigma*(drumspeed*numberofpoles+particleRspeed.z);
				double den = 4*(256+pow(0.6*CH_C_PI*diam.y*csi_per/diam.x,2));
				constR = alpha*0.6*CH_C_PI*diam.y*pow(csi_per,2)/diam.x/den+(1-alpha)*pow(csi_par,2)/(144+pow(csi_par,2));
				constI = alpha*16*csi_per/den+(1-alpha)*12*csi_par/(144+pow(csi_par,2));
				//constTorque = (alpha*1/64*(diam.x,2)+(1-alpha)*1/12*(diam.y,2))* sigma*(drumspeed*numberofpoles+particleRspeed.z)* (CH_C_PI*pow(diam.x/2,2)*diam.y);
				Volume = CH_C_PI*pow(diam.x/2,2)*diam.y;
				//constTorque = -pow(B,2)*CH_C_PI*pow(diam.x/2,2)*diam.y*constI/mu0;
				//data_forces << csi_par << ",\t";
				//data_forces << csi_per << ",\t";
			}
			else // cylinder
			{	
				d=diam.x;
				double csi = mu0*pow(diam.x,2)* sigma*(drumspeed*numberofpoles+particleRspeed.z);
				constR = alpha*9*pow(csi,2)/(8*(576+pow(csi,2)))+(1-alpha)*3*pow(csi,2)/(2*(576+pow(csi,2)));
				constI = alpha*9*24*csi/(8*(576+pow(csi,2)))+(1-alpha)*3*24*csi/(2*(576+pow(csi,2)));
				//constTorque = (alpha*3/64+(1-alpha)*1/16)*pow(diam.x,2)* sigma*(drumspeed*numberofpoles+particleRspeed.z)* (CH_C_PI*pow(diam.x/2,2)*diam.y);
				Volume = CH_C_PI*pow(diam.x/2,2)*diam.y;
				//constTorque = -pow(B,2)*CH_C_PI*pow(diam.x/2,2)*diam.y*constI/mu0;

				//data_forces << csi << ",\t\t\t";
			}
		}
		double R0=(rot_speedz*2*d)/(2*pow(velocity_norm_sq,0.5));
		double CL=R0/(2.2*R0+0.7);
	    double CLdisk=1.4*R0/(R0+1);
		double CD=pow(0.1+2*pow(CL,2),0.5);
	    double CDdisk=pow(0.16+0.25*pow(CLdisk,2),0.5);
		
		if (shape==3 && diam.y<diam.x)
		{
			CL=CLdisk;
			CD=CDdisk;
		}

		ChVector<> InducedForce;
		/*double Induced_Fr = ((numberofpoles+1)*pow(B,2)*Volume/mu0/distance)*constR;
		double Induced_Fphi = ((numberofpoles+1)*pow(B,2)*Volume/mu0/distance)*constI;
		double InducedF = sqrt(pow(Induced_Fr,2)+pow(Induced_Fr,2));*/
		InducedForce.x = ((numberofpoles+1)*pow(B,2)*Volume/mu0/distance)*(constR*cos(phi)+constI*sin(phi));
		InducedForce.y = ((numberofpoles+1)*pow(B,2)*Volume/mu0/distance)*(constR*sin(phi)-constI*cos(phi));
		InducedForce.z = 0;	
		abody->Accumulate_force(InducedForce, abody->GetPos(), false);

		ChVector<> DragForce;
		DragForce.x = -CD*ro*velocity_norm_sq*CH_C_PI*diam.x*diam.y/2*cos(phi2);
		DragForce.y = CD*ro*velocity_norm_sq*CH_C_PI*diam.x*diam.y/2*sin(phi2);
		DragForce.z = 0;

		abody->Accumulate_force( DragForce, abody->GetPos(), false);

		ChVector<> LiftForce;
		LiftForce.x = CL*ro*velocity_norm_sq*CH_C_PI*diam.x*diam.y/2*sin(phi2);
	    LiftForce.y = CL*ro*velocity_norm_sq*CH_C_PI*diam.x*diam.y/2*cos(phi2);
		LiftForce.z = 0;	
	
		abody->Accumulate_force(LiftForce, abody->GetPos(), false);
		
		ChVector<> InducedTorque;
		InducedTorque.x = 0;
		InducedTorque.y = 0;
		//InducedTorque.z = -constTorque*pow(B,2);
		InducedTorque.z = -pow(B,2)*Volume*constI/mu0;
		abody->Accumulate_torque(InducedTorque, false);
		/*
		data_forces << phi << ",\t";
		data_forces << mu0 << ",\t";
		data_forces << msystem->GetChTime() << ",\t";
		data_forces << distance << ",\t";
		data_forces << drumspeed*numberofpoles+particleRspeed.z << ",\t";
		data_forces << sigma << ",\t";
		data_forces << (numberofpoles+1)*pow(B,2)*Volume/mu0/distance*constR << ",\t";
		data_forces << (numberofpoles+1)*pow(B,2)*Volume/mu0/distance*constI << ",\t";
		data_forces << InducedForce.x << ",\t";
		data_forces << InducedForce.y << ",\t";
		data_forces << InducedForce.z << ",\t";
		data_forces << constR << ",\t";
		data_forces << constI << ",\t";
		data_forces << shape << "\n";*/
		//coordinate del rotore. la y del rotore è la z delle coordinate del sistema
		ChVector<> pos = drum_csys.TrasformParentToLocal(abody->GetPos());
		ChVector<> acc_force=abody->Get_accumulated_force();
		ChVector<> acc_torque=abody->Get_accumulated_torque();
		
		ChVector<> iner=abody->GetInertiaXX();
		double ingxx = iner.x;
		double ingyy = iner.y;
		double ingzz = iner.z;

		double posx=pos.x;
		double posy=pos.y;
		if (i>12+n &&totframes%10==0){
	/*	test << b << "\t";
		test << diam.x << "\t";
		test << diam.y << "\t";
		test << diam.z << "\t";
		test << posx << "\t";
		test << posy << "\t";
		test << pos.z << "\t";
		test << B << "\t";
		test << distance << "\t";*/
		test << ingxx << "\t";
		test << ingyy << "\t";
		test << ingzz << "\t";
 		/*test << particleRspeed.x << "\t";
		test << particleRspeed.y << "\t"; 
		test << particleRspeed.z << "\t";*/
		//test << particleRspeed_loc.x << "\t";
		//test << particleRspeed_loc.y << "\t"; 
		//test << particleRspeed_loc.z << "\t";
		/*test << acc_force.x << "\t";
		test << acc_force.y << "\t";
		test << acc_force.z << "\t";
		test << DragForce.x << "\t";
		test << DragForce.y << "\t";
		test << LiftForce.x << "\t";
		test << LiftForce.y << "\t";
		test << InducedForce.x << "\t";
		test << InducedForce.y << "\t";
		test << acc_force.Length() << "\t";
		test << acc_torque.z << "\t"; 
		test << acc_torque.Length() << "\t";*/
		test << alpha << "\t";
		test << abody->GetConductivity() << "\n"; }
		
	}
}
 
// Control on the fall point

void fall_point (	ChIrrAppInterface& application,
					ChSystem* msystem,		// contains all bodies
					ChCoordsys<>& drum_csys, // pos and rotation of drum
					double y_threshold)		
{
	double deltaT = application.GetTimestep();
	double gravity = 9.81;
	double deltaS = 0.0005; //gravity*pow(deltaT,2)/2;
	ofstream threshold;
	threshold.open("output\\threshold.dat",ios::app); 
	
	
	/*char padnumber[100];
	char filename[200];
	sprintf(padnumber, "%d", (totframes+10000));
	sprintf(filename, "output\\threshold%s.dat", padnumber+1);
	ChStreamOutAsciiFile threshold(filename);*/
	
	for (unsigned int i=0; i<msystem->Get_bodylist()->size(); i++)
	{
		
		ChBody* abody = (*msystem->Get_bodylist())[i];
		int flag=i-13;
		// calculate the position of body COG with respect to the drum COG:
		ChVector<> mrelpos = drum_csys.TrasformParentToLocal(abody->GetPos());
		double shape = abody->GetCollisionModel()->GetShapeType();
		double conduct= abody->GetConductivity();
		double sphmass=abody->GetMass();
		double posy=-mrelpos.z; // vertical position of the particle with respect to the drum COG
		double posx=mrelpos.x;
		int a=0;
		int b=abody->GetIdentifier();
			if (posy <= y_threshold+bin_height && posy >= y_threshold && posx>0 && posx<bin_length && b!=0)
				{   
					/* vecchio codice 
					char padnumber[100];
					char filename[200];
					sprintf(padnumber, "%d", (totframes+10000));
					sprintf(filename, "output\\threshold%s.dat", padnumber+1);
					ChStreamOutAsciiFile threshold(filename);*/

					threshold << msystem->GetChTime() << "\t";
					threshold << mrelpos.x << "\t";
					threshold << -mrelpos.z << "\t";
					threshold << mrelpos.y << "\t";
					threshold << shape << "\t";
					threshold << sphmass << "\t";
					threshold << conduct << "\n";
					abody->SetIdentifier(a);
				}
		
	}
}

//void fall_point (	ChIrrAppInterface& application,
//					ISceneNode* parent,
//					ChSystem* msystem,		// contains all bodies
//					ChCoordsys<>& drum_csys, // pos and rotation of drum
//					double y_threshold)		
//{
//	char padnumber[100];
//	char filename[200];
//	sprintf(padnumber, "%d", (totframes+10000));
//	sprintf(filename, "output\\threshold%s.dat", padnumber+1);
//	ChStreamOutAsciiFile threshold(filename);
//
//	ISceneNodeList::ConstIterator it = parent->getChildren().begin();
//	for(; it != parent->getChildren().end(); ++it)
//	{
//		if (parent->getAbsolutePosition().Y <= y_threshold)
//		{
//			threshold << msystem->GetChTime() << ",\t";
//			threshold << parent->getAbsolutePosition().X << ",\t";
//			threshold << parent->getAbsolutePosition().Y << ",\t";
//			threshold << parent->getAbsolutePosition().Z << ",\t";
//
//			ISceneNode* todelete = (*it);
//			++it;
//			parent->removeChild(todelete);
//		}
//	}
//}   
 
int main(int argc, char* argv[])
{
	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	DLL_CreateGlobals();

	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;

	// From now on, functions in ChParticlesSceneNodeTools will find 3d .obj models 
	// in "../objects/", instead of default "../data/" dir:
	irrlicht_default_obj_dir = "../objects/";

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&mphysicalSystem, L"Conveyor belt",core::dimension2d<u32>(800,600),false);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo("../objects/");
	application.AddTypicalSky("../objects/skybox/");
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(1.5f,0.4f,-1.0f), core::vector3df(0.f,0.f,0.f));
	application.AddLightWithShadow(vector3df(-4.5,5.5,4.5), vector3df(0,0,0), 10, 1.2,10.2, 30,512, video::SColorf(1,0.9,0.9));

	// This is for GUI tweaking of system parameters..
	MyEventReceiver receiver(&application);
	  // note how to add the custom event receiver to the default interface:
	application.SetUserEventReceiver(&receiver);


	// Set small collision envelopes for objects that will be created from now on..
	ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
	ChCollisionModel::SetDefaultSuggestedMargin  (0.002);



	// Create two conveyor fences
	ChBodySceneNode* mfence1 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, application.GetSceneManager(),
											1.0,
											ChVector<>(0,fence_height/2,-conveyor_width/2-fence_width/2),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(conveyor_length,fence_height,fence_width) );
	mfence1->GetBody()->SetBodyFixed(true);
	mfence1->GetBody()->SetFriction(0.01);
	mfence1->GetBody()->GetCollisionModel()->SetFamily(1);
    mfence1->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2); 
	mfence1->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3); 

	ChBodySceneNode* mfence2 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, application.GetSceneManager(),
											1.0,
											ChVector<>(0,fence_height/2,conveyor_width/2+fence_width/2),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(conveyor_length,fence_height,fence_width) );
	mfence2->GetBody()->SetBodyFixed(true);
	mfence2->GetBody()->SetFriction(0.01);
    mfence2->GetBody()->GetCollisionModel()->SetFamily(1);
    mfence2->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2); 
	mfence2->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3); 

	ChBodySceneNode* mfence3 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, application.GetSceneManager(),
											1.0,
											ChVector<>(-conveyor_length/2-fence_width/2,fence_height/2, 0),
											ChQuaternion<>(0,0,1,0), 
											ChVector<>(fence_width,fence_height,conveyor_width+2*fence_width) );
	mfence3->GetBody()->SetBodyFixed(true);
	mfence3->GetBody()->SetFriction(0.01);
    mfence3->GetBody()->GetCollisionModel()->SetFamily(1);
    mfence3->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2); 
	mfence3->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3); 

	ChBodySceneNode* mfence4 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
					    &mphysicalSystem, application.GetSceneManager(),1.0,
						ChVector<>(conveyor_length/2-fence_width/2,y_posbin+bin_height/2-drumdiameter/2, 0),
						ChQuaternion<>(0,0,1,0), 
					        ChVector<>(fence_width,bin_height,bin_width+2*fence_width) );
	mfence4->GetBody()->SetBodyFixed(true);
	mfence4->GetBody()->SetFriction(0.1);

    ChBodySceneNode* mfence5 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
						&mphysicalSystem, application.GetSceneManager(),1.0,
					    ChVector<>(conveyor_length/2+bin_length/2,y_posbin-binbase/2-drumdiameter/2, 0),
						ChQuaternion<>(1,0,0,0), 
						ChVector<>(bin_length,binbase,bin_width) );
	mfence5->GetBody()->SetBodyFixed(true);
	mfence5->GetBody()->SetFriction(0.1);
	
    ChBodySceneNode* mfence6 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
						&mphysicalSystem, application.GetSceneManager(),1.0,
						ChVector<>(conveyor_length/2+bin_length+fence_width/2,y_posbin+bin_height/2-drumdiameter/2, 0),
						ChQuaternion<>(0,0,1,0), 
						ChVector<>(fence_width,bin_height,bin_width+2*fence_width) );
	mfence6->GetBody()->SetBodyFixed(true);
	mfence6->GetBody()->SetFriction(0.1);

    ChBodySceneNode* mfence7 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
						&mphysicalSystem, application.GetSceneManager(),1.0,
						ChVector<>(conveyor_length/2+bin_length/2,y_posbin+bin_height/2-drumdiameter/2,-bin_width/2-fence_width/2),
						ChQuaternion<>(0,1,0,0), 
						ChVector<>(bin_length,bin_height,fence_width) );
	mfence7->GetBody()->SetBodyFixed(true);
	mfence7->GetBody()->SetFriction(0.1);

    ChBodySceneNode* mfence8 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
						&mphysicalSystem, application.GetSceneManager(),1.0,
						ChVector<>(conveyor_length/2+bin_length/2,y_posbin+bin_height/2-drumdiameter/2, bin_width/2+fence_width/2),
						ChQuaternion<>(0,1,0,0), 
						ChVector<>(bin_length,bin_height,fence_width) );
	mfence8->GetBody()->SetBodyFixed(true);
	mfence8->GetBody()->SetFriction(0.1);

if (n == 2)
	        {
        double x_splitter1=bin_length/2;
	ChBodySceneNode* mfence9 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
						&mphysicalSystem, application.GetSceneManager(),1.0,
						ChVector<>(conveyor_length/2+x_splitter1,y_posbin+splitter_height/2-drumdiameter/2,0),
						//ChQuaternion<>(0,0,1,0), 
						ChQuaternion<>(0.96,0,0,0.25), 
						ChVector<>(splitter_width,splitter_height,bin_width) );
	mfence9->GetBody()->SetBodyFixed(true);
	mfence9->GetBody()->SetFriction(0.1);	

		}
if (n == 3)
	        {
        double x_splitter1=bin_length/3;
        double x_splitter2=2*bin_length/3;
	ChBodySceneNode* mfence9 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
						&mphysicalSystem, application.GetSceneManager(),1.0,
						ChVector<>(conveyor_length/2+x_splitter1,y_posbin+splitter_height/2-drumdiameter/2,0),
						ChQuaternion<>(0,0,1,0), 
						ChVector<>(splitter_width,splitter_height,bin_width) );
	mfence9->GetBody()->SetBodyFixed(true);
	mfence9->GetBody()->SetFriction(0.1);
    ChBodySceneNode* mfence10 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
						&mphysicalSystem, application.GetSceneManager(),1.0,
						ChVector<>(conveyor_length/2+x_splitter2,y_posbin+splitter_height/2-drumdiameter/2,0),
						ChQuaternion<>(0,0,1,0), 
						ChVector<>(splitter_width,splitter_height,bin_width) );
	mfence10->GetBody()->SetBodyFixed(true);
	mfence10->GetBody()->SetFriction(0.1);
	
		}
if (n == 4)
	        {
        double x_splitter1=bin_length/2;
        double x_splitter2=bin_length/4;
        double x_splitter3=3*bin_length/4;
	ChBodySceneNode* mfence9 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
						&mphysicalSystem, application.GetSceneManager(),1.0,
						ChVector<>(conveyor_length/2+x_splitter1,y_posbin+splitter_height/2-drumdiameter/2,0),
						ChQuaternion<>(0,0,1,0), 
						ChVector<>(splitter_width,splitter_height,bin_width) );
	mfence9->GetBody()->SetBodyFixed(true);
	mfence9->GetBody()->SetFriction(0.1);
    ChBodySceneNode* mfence10 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
						&mphysicalSystem, application.GetSceneManager(),1.0,
						ChVector<>(conveyor_length/2+x_splitter2,y_posbin+splitter_height/2-drumdiameter/2,0),
						ChQuaternion<>(0,0,1,0), 
						ChVector<>(splitter_width,splitter_height,bin_width) );
	mfence10->GetBody()->SetBodyFixed(true);
	mfence10->GetBody()->SetFriction(0.1);
	ChBodySceneNode* mfence11 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
						&mphysicalSystem, application.GetSceneManager(),1.0,
						ChVector<>(conveyor_length/2+x_splitter3,y_posbin+splitter_height/2-drumdiameter/2,0),
						ChQuaternion<>(0,0,1,0), 
						ChVector<>(splitter_width,splitter_height,bin_width) );
	mfence11->GetBody()->SetBodyFixed(true);
	mfence11->GetBody()->SetFriction(0.1);
		}

ChBodySceneNode* mfence12 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
						&mphysicalSystem, application.GetSceneManager(),1.0,
						ChVector<>(xnozzle-xnozzlesize/2-fence_width/2,ynozzlesize/2+ynozzle,0),
						ChQuaternion<>(0,0,1,0), 
						ChVector<>(fence_width,ynozzlesize,znozzlesize+2*fence_width) );
	mfence12->GetBody()->SetBodyFixed(true);
	mfence12->GetBody()->SetFriction(0.1);

ChBodySceneNode* mfence13 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
						&mphysicalSystem, application.GetSceneManager(),1.0,
						ChVector<>(xnozzle+xnozzlesize/2+fence_width/2,ynozzlesize/2+ynozzle,0),
						ChQuaternion<>(0,0,1,0), 
						ChVector<>(fence_width,ynozzlesize,znozzlesize+2*fence_width) );
	mfence13->GetBody()->SetBodyFixed(true);
	mfence13->GetBody()->SetFriction(0.1);

ChBodySceneNode* mfence14 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
						&mphysicalSystem, application.GetSceneManager(),1.0,
						ChVector<>(xnozzle,ynozzlesize/2+ynozzle,znozzlesize/2+fence_width/2),
						ChQuaternion<>(0,1,0,0), 
						ChVector<>(xnozzlesize,ynozzlesize,fence_width) );
	mfence14->GetBody()->SetBodyFixed(true);
	mfence14->GetBody()->SetFriction(0.1);

ChBodySceneNode* mfence15 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
						&mphysicalSystem, application.GetSceneManager(),1.0,
						ChVector<>(xnozzle,ynozzlesize/2+ynozzle,-znozzlesize/2-fence_width/2),
						ChQuaternion<>(0,1,0,0), 
						ChVector<>(xnozzlesize,ynozzlesize,fence_width) );
	mfence15->GetBody()->SetBodyFixed(true);
	mfence15->GetBody()->SetFriction(0.1);
  

/*
ChBodySceneNode* convbase = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, application.GetSceneManager(),
											1.0,
											ChVector<>(0,0,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(conveyor_length,conv_thick,conv_width) );
	convbase->GetBody()->SetBodyFixed(true);
	convbase->GetBody()->SetFriction(0.1);
*/

	
/*
	// HOW TO IMPORT A SOLIDWORK MODEL ***ALEX***

	// 1) create the Python engine. This is necessary in order to parse the files that 
	// have been saved using the SolidWorks add-in for Chrono::Engine.

	ChPythonEngine my_python;

	// 2) loads the .py file (as saved from SolidWorks) and fill the system.
	try
	{
		my_python.ImportSolidWorksSystem("../objects/cad_conveyor", mphysicalSystem);  // note, don't type the .py suffic in filename..
	}
	catch (ChException myerror)
	{
		GetLog() << myerror.what();
	}

	// 3) this is needed for rendering in Irrlicht 3D views:
	application.AssetBindAll();
	application.AssetUpdateAll();
	application.AddShadowAll();

*/


	// Create the conveyor belt (this is a pure Chrono::Engine object, 
	// because an Irrlicht 'SceneNode' wrapper is not yet available, so it is invisible - no 3D preview)

	
	ChSharedPtr<ChConveyor> mconveyor (new ChConveyor(conveyor_length, conv_thick, conveyor_width));
	mconveyor->SetBodyFixed(true);
	mconveyor->SetFriction(0.9);
	mconveyor->SetConveyorSpeed(STATIC_speed);
	mconveyor->SetPos( ChVector<>(0, 0, 0) );

	mphysicalSystem.Add(mconveyor);
	mconveyor->GetCollisionModel()->SetFamily(2);
    mconveyor->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1); 
	mconveyor->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3); 

	double drumradius = 0.15;
	ChBodySceneNode* mDrum = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
												application.GetSystem(), application.GetSceneManager(),
												1,
												ChVector<>(conveyor_length/2, -drumradius-conv_thick/2,0),
												ChQuaternion<> (pow(2,0.5)/2,pow(2,0.5)/2,0,0), 
												ChVector<>(drumradius*2,conveyor_width,drumradius*2));

	mDrum->GetBody()->SetBodyFixed(true);
	mDrum->GetBody()->SetFriction(0.01);
	mDrum->GetBody()->GetCollisionModel()->SetFamily(3);
    mDrum->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1); 
	mDrum->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2); 



	// 
	// THE SOFT-REAL-TIME CYCLE
	//
	
	application.SetStepManage(true);
	application.SetTimestep(0.001);
	
	double threshold = -0.815;
	ofstream record;
	record.open("output\\threshold.dat",ios::trunc); 
	record << "Time" << "\t";
	record << "X Position" << "\t";
	record << "Y Position" << "\t";
	record << "Z Position" << "\t";
	record << "Shape" << "\t";
	record << "Mass (kg)" << "\t";
	record << "Conductivity" << "\n";

	int stepnum = 0;	
	int savenum = 0;

	while(application.GetDevice()->run()) 
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();

		application.DoStep();

		mDrum->GetBody()->GetFrame_REF_to_abs().GetCoord();

		totframes++;

		apply_forces (	&mphysicalSystem,		// contains all bodies
						mDrum->GetBody()->GetFrame_REF_to_abs().GetCoord(), // pos and rotation of drum 
						drumspeed,		 // speed of drum
						numberofpoles,	 // number of couples of poles
						intensity,		 // intensity of the magnetic field
						drumdiameter,
						totframes);

		fall_point (	application,
						&mphysicalSystem,		// contains all bodies
						mDrum->GetBody()->GetFrame_REF_to_abs().GetCoord(), // pos and rotation of drum 
						threshold);

		//fall_point (	application,
		//				parent,
		//				&mphysicalSystem,		// contains all bodies
		//				mDrum->GetBody()->GetFrame_REF_to_abs().GetCoord(), // pos and rotation of drum 
		//				threshold);
		if (!application.GetPaused())
		{ 
			
			// Continuosly create debris that fall on the conveyor belt
			create_debris(application.GetTimestep(), STATIC_flow, *application.GetSystem(), &application);

			// Limit the max age (in seconds) of debris particles on the scene, 
			// deleting the oldest ones, for performance
			purge_debris (*application.GetSystem(), 6.0);

			// Maybe the user played with the slider and changed STATIC_speed...
		 	mconveyor->SetConveyorSpeed(STATIC_speed);

			//***ALEX esempio salvataggio su file...
			stepnum++;
			if ((stepnum % saveEachNframes == 0) && (savefile == true))
			{
				savenum++;
				char buffer[120];
				sprintf(buffer, "esempio_output%05d.txt", savenum);
				ChStreamOutAsciiFile file_for_output(buffer);
				for (unsigned int i=0; i<mphysicalSystem.Get_bodylist()->size(); i++)
				{
					ChBody* abody = (*mphysicalSystem.Get_bodylist())[i];

					// Fetch the ElectricParticleProperty asset from the list
					for (unsigned int na= 0; na< abody->GetAssets().size(); na++)
					{
						ChSharedPtr<ChAsset> myasset = abody->GetAssetN(na);
						if (myasset.IsType<ElectricParticleProperty>())
						{
							// ok, its a particle!
							ChSharedPtr<ElectricParticleProperty> electricproperties = myasset;
							double my_cond  = electricproperties->conductivity ; // ..
							// Save on disk some infos...
							file_for_output << abody->GetPos().x << ", "
											<< abody->GetPos().y << ", "
											<< abody->GetPos().z << ", "
											<< abody->GetPos_dt().x << ", "
											<< abody->GetPos_dt().y << ", "
											<< abody->GetPos_dt().z << ", "
											<< my_cond << "\n";
						}
					}
				}
			} // end saving code


			
		
		}

		application.GetVideoDriver()->endScene();  
		
	}
	

	

 
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}