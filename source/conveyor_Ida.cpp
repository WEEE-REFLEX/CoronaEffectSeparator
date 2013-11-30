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
#include "core/ChFileutils.h"
#include "irrlicht_interface/CHbodySceneNode.h"
#include "irrlicht_interface/CHbodySceneNodeTools.h" 
#include "irrlicht_interface/CHirrApp.h"
#include "core/CHrealtimeStep.h"
#include "core/CHmath.h"
#include <irrlicht.h>
#include <fstream>
#include "unit_PYTHON/ChPython.h"
#include "unit_POSTPROCESS/ChPovRay.h" 

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
//
using namespace std;

// Static values valid through the entire program (bad
// programming practice, but enough for quick tests)


double STATIC_flow = 1000; 
double STATIC_speed = 0.72; //[m/s]

//const double mu0 = 0.0000012566; //vacuum permability [Tm/A]
const double epsilon = 8.85941e-12; // dielectric constant [F/m] *****ida 
const double epsilonO = 8.854187e-12; //vacuum permeability
const double epsilonR = 2.5; //relative permeability
const double drumspeed = 6.28; //[rad/s]
const double eta = 0.0000181; // Air drag coefficent [N*s/m^2]
const double numberofpoles = 9;
const double intensity = 0.32; 
const double drumdiameter = 0.228;
const double electrodediameter = 0.038;
const double U = 25000; // supplied high-voltage [v]
const double L = 0.21; //certer distance of rotating roll electrode and electrostatic pole *****ida
const double alpha = (CH_C_PI/180)*30; //angle of horizontal line and electrodes center line *****ida
const double h1 = (pow(L,2)+pow((drumdiameter/2),2)-((electrodediameter/2),2))/(2*L); //analytical parameter****ida
const double h2 = (pow(L,2)-pow((drumdiameter/2),2)+((electrodediameter/2),2))/(2*L);//analytical parameter****ida
const double j = sqrt(pow(h1,2)-pow((drumdiameter/2),2));//analytical parameter****ida
const double f = U/log(((h1+j-(drumdiameter/2))*(h2+j-(electrodediameter/2)))/((drumdiameter/2)+j-h1)*((electrodediameter/2)+j-h2));//analytical parameter****ida


// conveyor constant
const double conveyor_length=1;// accorciato da 2 a 1,*****ida
const double conveyor_width=0.6;
const double conv_thick = 0.01; // no troppo sottile, altrimenti non va la collision detection! Non importa se compenetra il cilindro.

const double ro=1.225;  //fluid density (air) [Kg/m^3]

// fence constant
const double fence_width = 0.02;
const double fence_height=0.2;
// bin constant 
const double y_posbin=-0.815;
const double binbase=0.02; // base thickness of the bin
const double bin_length=1; // accorciato da 3 a 1, ida
const double bin_width=1.5;
const double bin_height=0.2;
const double n = 2; // number of bins (values from 2 to 4), a regime devo importare il cad dei 3 contenitori, ida
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
const double ynozzle = 0.01;
const double xnozzle = -conveyor_length/2+xnozzlesize/2+fence_width; //portato avanti****ida

const double densityMetal = 8900;//1820;
const double densityPlastic = 900;

int myid = 1;

// set as true for saving log files each n frames
bool save_dataset = false;
bool save_irrlicht_screenshots = false;
bool save_POV_screenshots = false;
int saveEachNframes = 10;

bool irr_cast_shadows = false;

int totframes = 0;
	
bool init_particle_speed = true;

double particle_magnification = 5;


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
	double chargeM;		//***ida + ale (coulomb, for plastic)
	double chargeP;
	ChVector<> ElectricForce;
	ChVector<> StokesForce;
	ChVector<> ElectricImageForce;
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
		chargeM = 0;
		chargeP = 0;
	}
	
};





// This can be added to store the trajectory on a per-particle basis.

class ParticleTrajectory : public ChAsset
{
public:
	std::list< ChVector<> > positions;
	std::list< ChVector<> > speeds;
	unsigned int max_points;

	ParticleTrajectory() 
	{
		max_points = 80;
	}
};



/// Class that can be used to generate sample numbers according to a 
/// probability distribution. Probability distribution is defined with x,y points,
/// at least a dozen of pairs to be more precise in the reconstruction of probability.

class ChRandomDistribution
{
public:

		/// Create an object that can be used to generate sample numbers according to a 
		/// probability distribution. The probability distribution is a curve represented
		/// by simplified x,y pairs of points. The integral of the probability curve
		/// must be unit, i.e normalized (but if not, a normalization will be enforced)
		/// Note: too few points means approximate results, but too many points might give a 
		/// small performance overhead when calling GetRandom().
	ChRandomDistribution (ChMatrix<>& mx, ChMatrix<>& my)
	{
	
		if (mx.GetRows() != my.GetRows())
			throw ChException("Probability curve must have same number of rows in abscysse and ordinates");
		if ((mx.GetColumns() != 1) || (my.GetColumns() != 1))
			throw ChException("Probability curve must have column-vectors as input");

		x = new ChMatrixDynamic<>;
		y = new ChMatrixDynamic<>;
		cdf_x = new ChMatrixDynamic<>;
		cdf_y = new ChMatrixDynamic<>;

		*x = mx;
		*y = my;

		*cdf_x = mx;
		*cdf_y = my;

			// compute CDF
		double integral = 0;
		for (int i = 0; i< x->GetRows()-1; i++)
		{
			integral += 0.5*( (*y)(i) + (*y)(i+1) ) * ( (*x)(i+1) - (*x)(i) );
			(*cdf_y)(i) = integral;
			(*cdf_x)(i) = 0.5* ( (*x)(i+1) + (*x)(i) );
		}
			// normalize if P(x) had not unit integral
		double totintegral = (*cdf_y)(x->GetRows()-2);
		if (totintegral != 1.0)
		{
			for (int i = 0; i< x->GetRows()-1; i++)
			{
				(*cdf_y)(i) *= 1./totintegral;
			}
		}
		(*cdf_x)(x->GetRows()-1) = (*x)(x->GetRows()-1);
		(*cdf_y)(x->GetRows()-1) = 1.0;
		
	}

	~ChRandomDistribution()
	{
		delete x;
		delete y;
		delete cdf_x;
		delete cdf_y;
	}

		/// Compute a random value whose probability is the probability curve that has 
		/// been entered with x,y points during the creation of this object.
	double GetRandom()
	{
		double mx1 =  (*x)(0);
		double mx2 =  (*cdf_x)(0);
		double my1 =  0;
		double my2 =  (*cdf_y)(0);

		double rand = ChRandom();
		for (int i = 0; i< x->GetRows()-1; i++)
		{
			if (( rand <= (*cdf_y)(i+1) ) &&
				( rand >  (*cdf_y)(i)   ) )
			{
				mx1 = (*cdf_x)(i);
				mx2 = (*cdf_x)(i+1);
				my1 = (*cdf_y)(i);
				my2 = (*cdf_y)(i+1);
				break;
			}
		}
		// linear interp
		double val = mx1 +  ((rand - my1)/(my2-my1)) * (mx2-mx1);
		return val;
	}

	ChMatrix<>* GetProbabilityXpoints() {return x;}
	ChMatrix<>* GetProbabilityYpoints() {return y;}
	ChMatrix<>* GetProbabilityCDFcumulativeX() {return cdf_x;}
	ChMatrix<>* GetProbabilityCDFcumulativeY() {return cdf_y;}

private:
	ChMatrix<>* x;
	ChMatrix<>* y;

	ChMatrix<>* cdf_x;
	ChMatrix<>* cdf_y;
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
								true, rect<s32>(560, 15, 700, 15+20), 0, 101);
				scrollbar_flow->setMax(100); 
				scrollbar_flow->setPos(50);
				text_flow = application->GetIGUIEnvironment()->addStaticText(
							L"Flow [particles/s]", rect<s32>(710,15,800,15+20), false);

				// ..add GUI slider to control the speed
				scrollbar_speed = application->GetIGUIEnvironment()->addScrollBar(
								true, rect<s32>(560, 40, 700, 40+20), 0, 102);
				scrollbar_speed->setMax(100); 
				scrollbar_speed->setPos(100);
				text_speed = application->GetIGUIEnvironment()->addStaticText(
								L"Conveyor speed [m/s]:", rect<s32>(710,40,800,40+20), false);

				// ..add GUI checkmark to enable plotting forces
				checkbox_plotforces = application->GetIGUIEnvironment()->addCheckBox(false,core::rect<s32>(560,65, 560+150,55+20),
								0, 105, L"Plot applied CES forces");

				// ..add GUI checkmark to enable plotting forces
				checkbox_plottrajectories = application->GetIGUIEnvironment()->addCheckBox(false,core::rect<s32>(560,90, 560+150,90+20),
								0, 106, L"Plot trajectories");
				

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
								STATIC_flow = 1000* ((double)pos/50);
								char message[50]; sprintf(message,"Flow %d [particl/s]", (int)STATIC_flow);
								text_flow->setText(core::stringw(message).c_str());
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

public:
	IGUIScrollBar*  scrollbar_flow;
	IGUIStaticText* text_flow;
	IGUIScrollBar*  scrollbar_speed;
	IGUIStaticText* text_speed;
	IGUICheckBox*	checkbox_plotforces;
	IGUICheckBox*	checkbox_plottrajectories;
};


// Shortcut for easy creation of a body that has both a box visualization and a box collision,
// with default values for mass inertia etc. 

ChSharedPtr<ChBody> create_box_collision_shape(ChVector<> pos, //  center of box
											   ChVector<> size, // size along box x y z
											   ChQuaternion<> rot = QUNIT) // optional rotation
{
	// Create a body
	ChSharedPtr<ChBody> mrigidBody(new ChBody);

	mrigidBody->SetPos(pos);
			
	// Define a collision shape 
	mrigidBody->GetCollisionModel()->ClearModel();
	mrigidBody->GetCollisionModel()->AddBox(0.5*size.x, 0.5*size.y, 0.5*size.z);
	mrigidBody->GetCollisionModel()->BuildModel();
	mrigidBody->SetCollide(true);

	// Attach a visualization shape asset. 
	ChSharedPtr<ChBoxShape> mbox(new ChBoxShape);
	mbox->GetBoxGeometry().SetLenghts(size);
	mrigidBody->AddAsset(mbox);

	return mrigidBody;
}


// Function that creates debris that fall on the conveyor belt, to be called at each dt

void create_debris(double dt, double particles_second, 
				   ChSystem& mysystem, 
				   ChIrrApp* irr_application,
				   ChPovRay* mpov_exporter)
{

	double sph_fraction = 1;//0.3; // 30% cubes
	double box_fraction = 0;//0.4; // 40% cylinders
	double cyl_fraction = 1-box_fraction-sph_fraction;

	double sphrad = 0.6e-3 + (ChRandom()-0.5)*(0.6e-3);
	double cylhei = 0.035;
	double cylrad = sphrad;
	double cylmass = CH_C_PI*pow(cylrad,2)*cylhei* 1.0;  // now with default 1.0 density
	double sphmass = (4./3.)*CH_C_PI*pow(sphrad,3)* 1.0; // now with default 1.0 density
	double sphinertia = 0.4*pow(sphrad,2)*sphmass;      // now with default 1.0 density
	double cylinertia = 0.0833*(pow(cylhei,2)+3*pow(sphrad,2))*cylmass;//0.0833*(pow(cylhei,2)+3*pow(sphrad,2))*cylmass;  // now with default 1.0 density
	double cylinertia2 = 0.5*pow(sphrad,2)*cylmass; //0.5*pow(sphrad,2)*cylmass; // now with default 1.0 density
	

	double exact_particles_dt = dt * particles_second;
	double particles_dt = floor(exact_particles_dt);
	double remaind = exact_particles_dt - particles_dt;
	
	if (remaind > ChRandom()) particles_dt +=1;

	for (int i = 0; i < particles_dt; i++)
	{
		ChSharedPtr<ChBody> created_body;
		ChSharedPtr<ElectricParticleProperty> created_electrical_asset;

		double rand_shape_fract = ChRandom();

		//
		// 1 ---- Create particle 
		// 

		if (rand_shape_fract < sph_fraction)
		{
			// Create a body
			ChSharedPtr<ChBody> mrigidBody(new ChBody);

			mrigidBody->SetPos(ChVector<>(-0.5*xnozzlesize+ChRandom()*xnozzlesize+xnozzle, ynozzle, -0.5*znozzlesize+ChRandom()*znozzlesize));
			mrigidBody->SetMass(sphmass);
			mrigidBody->SetInertiaXX(ChVector<>(sphinertia,sphinertia,sphinertia));
			mrigidBody->SetFriction(0.2f);
			mrigidBody->SetImpactC(0.0f); 
			mrigidBody->SetIdentifier(myid); // NB fatto solo per le sfere!!!!!!!!!
			       
			// mrigidBody->SetRollingFriction(0.1);
			// mrigidBody->SetSpinningFriction(0.1);


			// Define a collision shape 
			mrigidBody->GetCollisionModel()->ClearModel();
			mrigidBody->GetCollisionModel()->AddSphere(sphrad);
			//mrigidBody->GetCollisionModel()->AddSphere(sphrad, &ChVector<>(0.005,0,0)); // etc. optional
			mrigidBody->GetCollisionModel()->BuildModel();
			mrigidBody->SetCollide(true);

			// Attach a visualization shape asset. 
			ChSharedPtr<ChSphereShape> msphere(new ChSphereShape);
			msphere->GetSphereGeometry().rad = sphrad * particle_magnification; 
			msphere->GetSphereGeometry().center = ChVector<>(0,0,0);
			mrigidBody->AddAsset(msphere);
			/* etc. optional
			ChSharedPtr<ChSphereShape> msphere2(new ChSphereShape);
			msphere2->GetSphereGeometry().rad = sphrad*5; // test****
			msphere2->GetSphereGeometry().center = ChVector<>(0.005,0,0);
			mrigidBody->AddAsset(msphere2);
			*/

			// Attach a custom asset. Look how to create and add a custom asset to the object! ***ALEX
			ChSharedPtr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty); 
			electric_asset->Cdim         = ChVector<>(2*sphrad,2*sphrad,2*sphrad);
			electric_asset->fraction	 = ElectricParticleProperty::e_fraction_sphere;
			electric_asset->birthdate	 = mysystem.GetChTime();
			mrigidBody->AddAsset(electric_asset);
			
			// Finally, do not forget to add the body to the system:
			mysystem.Add(mrigidBody);

			created_body = mrigidBody;
			created_electrical_asset = electric_asset; // for reference later
		}

		if ((rand_shape_fract > sph_fraction) && 
			(rand_shape_fract < box_fraction+sph_fraction))
		{
			double xscale = 1.3*(1-0.8*ChRandom()); // for oddly-shaped boxes..
			double yscale = 1.3*(1-0.8*ChRandom());
			double zscale = 1.3*(1-0.8*ChRandom());

			//	ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
			//	ChCollisionModel::SetDefaultSuggestedMargin  (0.002);

			// Create a body
			ChSharedPtr<ChBody> mrigidBody(new ChBody);

			mrigidBody->SetPos(ChVector<>(-0.5*xnozzlesize+ChRandom()*xnozzlesize+xnozzle, ynozzle, -0.5*znozzlesize+ChRandom()*znozzlesize));
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
			mbox->GetBoxGeometry().SetLenghts(ChVector<>(sphrad*2*xscale, sphrad*2*yscale, sphrad*2*yscale) * particle_magnification);
			mrigidBody->AddAsset(mbox);

			// Attach a custom asset. Look how to create and add a custom asset to the object! ***ALEX
			ChSharedPtr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty); 
			electric_asset->Cdim         = ChVector<>(2*sphrad,2*sphrad,2*sphrad);
			electric_asset->fraction	 = ElectricParticleProperty::e_fraction_box;
			electric_asset->birthdate	 = mysystem.GetChTime();
			mrigidBody->AddAsset(electric_asset);
			
			// Finally, do not forget to add the body to the system:
			mysystem.Add(mrigidBody);

			created_body = mrigidBody; // for reference later
			created_electrical_asset = electric_asset; // for reference later
		}

		if (rand_shape_fract > box_fraction+sph_fraction)
		{

			//	ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
			//	ChCollisionModel::SetDefaultSuggestedMargin  (0.002);

			// Create a body
			ChSharedPtr<ChBody> mrigidBody(new ChBody);

			mrigidBody->SetPos(ChVector<>(-0.5*xnozzlesize+ChRandom()*xnozzlesize+xnozzle, ynozzle, -0.5*znozzlesize+ChRandom()*znozzlesize));
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
			mcyl->GetCylinderGeometry().p1 = ChVector<>(0, cylhei/2,0) * particle_magnification;
			mcyl->GetCylinderGeometry().p2 = ChVector<>(0,-cylhei/2,0) * particle_magnification;
			mrigidBody->AddAsset(mcyl);

			// Attach a custom asset. Look how to create and add a custom asset to the object! ***ALEX
			ChSharedPtr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty); 
			electric_asset->Cdim         = ChVector<>(sphrad*2,cylhei,sphrad*2);
			electric_asset->fraction	 = ElectricParticleProperty::e_fraction_cylinder;
			electric_asset->birthdate	 = mysystem.GetChTime();
			mrigidBody->AddAsset(electric_asset);
			
			// Finally, do not forget to add the body to the system:
			mysystem.Add(mrigidBody);

			created_body = mrigidBody; // for reference later
			created_electrical_asset = electric_asset; // for reference later
		}

		//
		// 2 ---- Adjust stuff that depends on the metal/plastic fraction
		//

		// Depending on a randomized fraction, set the material type as 'plastic' 
		// or 'metal', by setting the 'material_type' in the ElectricParticleProperty of the
		// created particle and the 'conductivity' (regardless if it was a cylinder 
		// or cube etc.) ***ALEX
		if (!created_electrical_asset.IsNull())
		{
			double rand_mat = ChRandom();

			double plastic_fract=0.3;
		
			if (rand_mat < plastic_fract)
			{
				created_electrical_asset->conductivity = 0;
                created_electrical_asset->material_type = ElectricParticleProperty::e_mat_plastic;

				// Attach a 'pink' texture to easily recognize plastic stuff in 3d view
				ChSharedPtr<ChTexture> mtexture(new ChTexture);
				mtexture->SetTextureFilename("../objects/pinkwhite.png");
				created_body->AddAsset(mtexture);

				// Multiply the default mass & intertia tensor by density (previously assumed =1)
				created_body->SetMass( created_body->GetMass() * ::densityPlastic);
				created_body->SetInertiaXX( created_body->GetInertiaXX() * ::densityPlastic);
			}
			if (rand_mat > plastic_fract)
			{
				created_electrical_asset->conductivity = 6670000;
				created_electrical_asset->material_type = ElectricParticleProperty::e_mat_metal;

				// Attach a 'blue' texture to easily recognize metal stuff in 3d view
				ChSharedPtr<ChTexture> mtexture(new ChTexture);
				mtexture->SetTextureFilename("../objects/bluwhite.png");
				created_body->AddAsset(mtexture);

				// Multiply the default mass & intertia tensor by density (previously assumed =1)
				created_body->SetMass( created_body->GetMass() * ::densityMetal);
				created_body->SetInertiaXX( created_body->GetInertiaXX() * ::densityMetal);
			}


			if (created_electrical_asset->fraction == ElectricParticleProperty::e_fraction_sphere)
			{
			}

		}

		//
		// 3 ---- Set parameters that are common for whatever created particle, ex. limit speed threshold:
		//

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

		//
		// 4 ---- Irrlicht setup for enabling the 3d view of the particle
		//

		// If Irrlicht is used, setup also the visualization proxy.
		// This is necessary so that Irrlicht 3D view can show the visualization
		// assets that have been added to the created bodies. **ALEX
		if (!created_body.IsNull() && irr_application)
		{
			irr_application->AssetBind(created_body);
			irr_application->AssetUpdate(created_body);
		}

		//
		// 5  ---- If a POV exporter is used, one must add the particle to its database
		// 

		if (!created_body.IsNull() && mpov_exporter && save_POV_screenshots)
		{
			mpov_exporter->Add(created_body);
		}


		// 
		// 5 ---- it could be useful to attach an asset for storing the trajectory of particle
		//

		if (!created_body.IsNull())
		{
			ChSharedPtr<ParticleTrajectory> massettraj(new ParticleTrajectory);
			created_body->AddAsset(massettraj);
		}

		if (init_particle_speed)
		{
			created_body->SetPos_dt(ChVector<>(STATIC_speed, 0,0));
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
			// mysharedbody->RemoveRef(); //***NOT needed - previously needed cause always Add() to POV exporter..
			i--; // this because if deleted, the rest of the array is shifted back one position..
		}
	}

}

// Function that defines the forces on the debris ****ida

void apply_forces (	ChSystem* msystem,		// contains all bodies
						ChCoordsys<>& drum_csys, // pos and rotation of drum 
						double drumspeed,		 // speed of drum
						double numberofpoles,	 
						double intensity,		 
						double drumdiameter,
						double h1,
						double h2,
						double L,
						double electrodediameter,
						double j,
						double alpha,
			            double U,
						double f,
					


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

		bool was_a_particle = false;
		ChSharedPtr<ElectricParticleProperty> electricproperties; // null by default

		// Fetch the ElectricParticleProperty asset from the list of 
		// assets that have been attached to the object, and retrieve the
		// custom data that have been stored. ***ALEX
		for (unsigned int na= 0; na< abody->GetAssets().size(); na++)
		{
			ChSharedPtr<ChAsset> myasset = abody->GetAssetN(na);
			if (myasset.IsType<ElectricParticleProperty>())
			{
				// OK! THIS WAS A PARTICLE! ***ALEX
				was_a_particle = true;		
				electricproperties = myasset;
			} 
		}

		// Do the computation of forces only on bodies that had 
		// the 'ElectricParticleProperty' attached.. **ALEX
		if(was_a_particle)
		{

			ChVector<> diam = electricproperties->Cdim; 
			double sigma =    electricproperties->conductivity;

			// Remember to reset 'user forces accumulators':
			abody->Empty_forces_accumulators();

			// initialize speed of air (steady, if outside fan stream): 
			ChVector<> abs_wind(0,0,0);

			// calculate the position of body COG with respect to the drum COG:
			ChVector<> mrelpos = drum_csys.TrasformParentToLocal(abody->GetPos());
			double distx=mrelpos.x;
			double disty=mrelpos.y;
			ChVector<> velocity=abody->GetPos_dt();
			double velocityx=velocity.x;
			double velocityy=velocity.y;
			double velocityz=velocity.z;
			//ChVector <> rot_speed=abody->GetWvel_par();
			//double rot_speedz=rot_speed.z; //bisogna tirare fuori la componente attorno all'asse z globale della velocità di rotazione

			double velocity_norm_sq=velocity.Length2();

			//ChQuaternion<> rot_velocity=abody->GetRot_dt;
			
			// Polar coordinates of particles respect to the axis of the rotor, may be useful later **ALEX

			double distance = pow(distx*distx+disty*disty,0.5);
			double phi = atan2(disty,distx);
			double phi2 = atan2(-velocity.y,velocity.x);
			

		

			//
			// STOKES FORCES
			//


			double average_rad = 0.5* electricproperties->Cdim.Length(); // Approximate to sphere radius. Ida: this can be improved, by having Stokes forces for three Cdim x y z values maybe 

			ChVector<> StokesForce = electricproperties->StokesForce;
			
			electricproperties->StokesForce = (-6*CH_C_PI*eta*average_rad) * velocity;
	
			abody->Accumulate_force(StokesForce, abody->GetPos(), false);


            

            //Calculating the analytical expressions of the electric field***ida

			double xuno=distx*cos(alpha)+ disty*sin(alpha);//analytical parameter****ida
            double yuno=disty*cos(alpha)- distx*sin(alpha);//analytical parameter****ida

			double Ex=(((j-h1+xuno)/(pow((j-h1+xuno),2)+pow(yuno,2))+((j+h1-xuno)/(pow((j+h1-xuno),2)+pow(yuno,2)))*f));//analytical expression of the electric field x direction***ida
			double Ey=((yuno/(pow((j-h1+xuno),2)+pow(yuno,2))-(yuno/(pow((j+h1-xuno),2)+pow(yuno,2)))*f));//analytical expression of the electric field y direction***ida
			double Ez=0;

			ChVector<> vE (Ex, Ey, Ez);
			double E = vE.Length();

          
			
		    //
			//===== METAL FORCES ==========
			//

			if (electricproperties->material_type == ElectricParticleProperty::e_mat_metal)
			{ 
				// charge the particle? (contact w. drum)
				if ((distx > 0) && (disty > 0))
				{
					electricproperties->chargeM = 0.666666666666667*pow(CH_C_PI,3)*epsilon*pow(average_rad,2)*E;
					
				}

				
				ChVector<> ElectricForce = electricproperties->ElectricForce;

				electricproperties->ElectricForce = 0.832 * electricproperties->chargeM * vE;

				abody->Accumulate_force(ElectricForce, abody->GetPos(), false);


			} // end if material==metal


			//
			//===== PLASTIC FORCES ==========
			//

		    

			if (electricproperties->material_type == ElectricParticleProperty::e_mat_plastic) //forze sulle particelle non metalliche
			{
				

				// charge the particle? (contact w. drum)
				if ((distx > 0) && (disty > 0))
				{
					electricproperties->chargeP = 3*CH_C_PI*epsilonO*pow(2*average_rad,2)*1500000*(epsilonR/(epsilonR+2)); // charge
				}
				// discharge the particle? (contact w. blade)
				if ((distx < -(drumdiameter*0.5 - 0.009)) && (disty < -0.009))
				{
					electricproperties->chargeP = 0; // charge
				}

				ChVector<> ElectricImageForce = electricproperties->ElectricImageForce;


				electricproperties->ElectricImageForce.x = -((pow( electricproperties->chargeP,2))/(4*CH_C_PI*epsilon*pow((2*average_rad),2))*cos(atan2(disty,distx)));
				electricproperties->ElectricImageForce.y = -((pow( electricproperties->chargeP,2))/(4*CH_C_PI*epsilon*pow((2*average_rad),2))*sin(atan2(disty,distx)));
				electricproperties->ElectricImageForce.z = 0;	
						
				


				abody->Accumulate_force(ElectricImageForce, abody->GetPos(), false);
				

			   }  // end if material==plastic
			




			//ChVector<> DragForce;
			//DragForce.x = -CD*ro*velocity_norm_sq*CH_C_PI*diam.x*diam.y/2*cos(phi2);
			//DragForce.y = CD*ro*velocity_norm_sq*CH_C_PI*diam.x*diam.y/2*sin(phi2);
			//DragForce.z = 0;

			//abody->Accumulate_force( DragForce, abody->GetPos(), false);

			//ChVector<> LiftForce;
			//LiftForce.x = CL*ro*velocity_norm_sq*CH_C_PI*diam.x*diam.y/2*sin(phi2);
			//LiftForce.y = CL*ro*velocity_norm_sq*CH_C_PI*diam.x*diam.y/2*cos(phi2);
			//LiftForce.z = 0;	
		
			//abody->Accumulate_force(LiftForce, abody->GetPos(), false);
			
			//ChVector<> InducedTorque;
			//InducedTorque.x = 0;
			//InducedTorque.y = 0;
			//InducedTorque.z = -constTorque*pow(B,2);
			//InducedTorque.z = -pow(B,2)*Volume*constI/mu0;
			//abody->Accumulate_torque(InducedTorque, false);
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
			//ChVector<> acc_torque=abody->Get_accumulated_torque();
			
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
		
		} // end if(was_a_particle) , i.e. a body with electrical asset

	} // end for() loop on all bodies
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
		//int a=0;
		//int b=abody->GetIdentifier();
			//if (posy <= y_threshold+bin_height && posy >= y_threshold && posx>0 && posx<bin_length && b!=0)
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
					//abody->SetIdentifier(a);
				}
		
	}
}


void draw_forces(ChIrrApp& application, double scalefactor = 1.0)
{
	ChSystem* msystem = application.GetSystem();

	for (unsigned int i=0; i<msystem->Get_bodylist()->size(); i++)
	{
		ChBody* abody = (*msystem->Get_bodylist())[i];

		bool was_a_particle = false;
		ChSharedPtr<ElectricParticleProperty> electricproperties; // null by default

		// Fetch the ElectricParticleProperty asset from the list of 
		// assets that have been attached to the object, and retrieve the
		// custom data that have been stored. ***ALEX
		for (unsigned int na= 0; na< abody->GetAssets().size(); na++)
		{
			ChSharedPtr<ChAsset> myasset = abody->GetAssetN(na);
			if (myasset.IsType<ElectricParticleProperty>())
			{
				// OK! THIS WAS A PARTICLE! ***ALEX
				was_a_particle = true;		
				electricproperties = myasset;
			} 
		}

		// Do the computation of forces only on bodies that had 
		// the 'ElectricParticleProperty' attached.. **ALEX
		if(was_a_particle)
		{
			ChVector<> custom_force = abody->Get_accumulated_force();
			custom_force *= scalefactor;
			ChIrrTools::drawSegment(application.GetVideoDriver(), 
				abody->GetPos(), 
				abody->GetPos() + custom_force,
				video::SColor(255,   0,0,255));
		}
	}
}


void UpdateTrajectories(ChIrrApp& application)
{
	ChSystem* msystem = application.GetSystem();

	for (unsigned int i=0; i<msystem->Get_bodylist()->size(); i++)
	{
		ChBody* abody = (*msystem->Get_bodylist())[i];

		bool was_a_particle = false;
		ChSharedPtr<ParticleTrajectory> trajectoryasset; // null by default

		// Fetch the ElectricParticleProperty asset from the list of 
		// assets that have been attached to the object, and retrieve the
		// custom data that have been stored. ***ALEX
		for (unsigned int na= 0; na< abody->GetAssets().size(); na++)
		{
			ChSharedPtr<ChAsset> myasset = abody->GetAssetN(na);
			if (myasset.IsType<ParticleTrajectory>())
			{
				// OK! trajectory storage!	
				trajectoryasset = myasset;
				trajectoryasset->positions.push_back( abody->GetPos() );
				trajectoryasset->speeds.push_back( abody->GetPos_dt() );
				
				// remove excessive amount of elements
				while (trajectoryasset->positions.size() > trajectoryasset->max_points )
					trajectoryasset->positions.pop_front();
				while (trajectoryasset->speeds.size() > trajectoryasset->max_points )
					trajectoryasset->speeds.pop_front();
			} 
		}
	}
}

void DrawTrajectories(ChIrrApp& application)
{
	ChSystem* msystem = application.GetSystem();

	for (unsigned int i=0; i<msystem->Get_bodylist()->size(); i++)
	{
		ChBody* abody = (*msystem->Get_bodylist())[i];

		ChVector<> pointA = abody->GetPos();

		bool was_a_particle = false;
		ChSharedPtr<ParticleTrajectory> trajectoryasset; // null by default

		// Fetch the ElectricParticleProperty asset from the list of 
		// assets that have been attached to the object, and retrieve the
		// custom data that have been stored. ***ALEX
		for (unsigned int na= 0; na< abody->GetAssets().size(); na++)
		{
			ChSharedPtr<ChAsset> myasset = abody->GetAssetN(na);
			if (myasset.IsType<ParticleTrajectory>())
			{
				trajectoryasset = myasset;
				int npoints = 0;
				std::list< ChVector<> >::const_iterator iterator;
				std::list< ChVector<> >::const_iterator iteratorspeed;
				iteratorspeed = trajectoryasset->speeds.begin();
				for (iterator = trajectoryasset->positions.begin(); iterator != trajectoryasset->positions.end(); ++iterator)
				{
					ChVector<> pointB = *iterator;
					ChVector<> speed = *iteratorspeed;
					if (npoints >0)
					{
						double scalarspeed = speed.Length();
						double normalizedspeed = scalarspeed / 5.0;
						video::SColor mcol (255, (int)(255.*normalizedspeed), (int)(255.*normalizedspeed), (int)(255.*(1.0-normalizedspeed)) );
						ChIrrTools::drawSegment(application.GetVideoDriver(), 
								pointA, 
								pointB,
								mcol);
					}
					pointA = pointB;
					++npoints;
					++iteratorspeed;
				}
			} 
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
	application.AddTypicalCamera(core::vector3df(1.5f,0.4f,-1.0f), core::vector3df(0.5f,0.f,0.f));
	if (irr_cast_shadows)
		application.AddLightWithShadow(vector3df(-4.5,5.5,4.5), vector3df(0,0,0), 10, 1.2,10.2, 30,512, video::SColorf(1,0.9,0.9));

	// This is for GUI tweaking of system parameters..
	MyEventReceiver receiver(&application);
	  // note how to add the custom event receiver to the default interface:
	application.SetUserEventReceiver(&receiver);


	// Set small collision envelopes for objects that will be created from now on..
	ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
	ChCollisionModel::SetDefaultSuggestedMargin  (0.002);



	// Create two conveyor fences

	ChSharedPtr<ChBody> mfence1 = create_box_collision_shape(
				ChVector<>(0,fence_height/2,-conveyor_width/2-fence_width/2),	// pos
				ChVector<>(conveyor_length,fence_height,fence_width)			// size
				);
	application.GetSystem()->Add(mfence1);
	mfence1->SetBodyFixed(true);
	mfence1->SetFriction(0.01);
	mfence1->GetCollisionModel()->SetFamily(1);
    mfence1->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2); 
	mfence1->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3); 


	ChSharedPtr<ChBody> mfence2 = create_box_collision_shape(
				ChVector<>(0,fence_height/2,conveyor_width/2+fence_width/2),	// pos
				ChVector<>(conveyor_length,fence_height,fence_width)			// size
				);
	application.GetSystem()->Add(mfence2);
	mfence2->SetBodyFixed(true);
	mfence2->SetFriction(0.01);
    mfence2->GetCollisionModel()->SetFamily(1);
    mfence2->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2); 
	mfence2->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3); 


	ChSharedPtr<ChBody> mfence3 = create_box_collision_shape(
				ChVector<>(-conveyor_length/2-fence_width/2,fence_height/2, 0),	// pos
				ChVector<>(fence_width,fence_height,conveyor_width+2*fence_width),	// size
				ChQuaternion<>(0,0,1,0)
				);
	application.GetSystem()->Add(mfence3);
	mfence3->SetBodyFixed(true);
	mfence3->SetFriction(0.01);
    mfence3->GetCollisionModel()->SetFamily(1);
    mfence3->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2); 
	mfence3->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3); 


	ChSharedPtr<ChBody> mfence4 = create_box_collision_shape(
				ChVector<>(conveyor_length/2-fence_width/2-2*drumdiameter,y_posbin+bin_height/2-drumdiameter/2, 0),//prima parete verticale delle scatole in basso verso il rotore
				ChVector<>(fence_width,bin_height,bin_width+2*fence_width),	// size
				ChQuaternion<>(0,0,1,0)
				);
	application.GetSystem()->Add(mfence4);
	mfence4->SetBodyFixed(true);
	mfence4->SetFriction(0.1);


	ChSharedPtr<ChBody> mfence5 = create_box_collision_shape(
				ChVector<>(conveyor_length/2+bin_length/2-2*drumdiameter,y_posbin-binbase/2-drumdiameter/2+binbase, 0),//fondo della scatola
				ChVector<>(bin_length,binbase,bin_width)	// size
				);
	application.GetSystem()->Add(mfence5);
	mfence5->SetBodyFixed(true);
	mfence5->SetFriction(0.1);


	ChSharedPtr<ChBody> mfence6 = create_box_collision_shape(
				ChVector<>(conveyor_length/2+bin_length+fence_width/2-2*drumdiameter,y_posbin+bin_height/2-drumdiameter/2, 0),//ultima parete verticale della scatola in basso lontano dal rotore
				ChVector<>(fence_width,bin_height,bin_width+2*fence_width),	// size
				ChQuaternion<>(0,0,1,0)
				);
	application.GetSystem()->Add(mfence6);
	mfence6->SetBodyFixed(true);
	mfence6->SetFriction(0.1);


	ChSharedPtr<ChBody> mfence7 = create_box_collision_shape(
				ChVector<>(conveyor_length/2+bin_length/2-2*drumdiameter,y_posbin+bin_height/2-drumdiameter/2,-bin_width/2-fence_width/2), //parete sx guardando dal fondo
				ChVector<>(bin_length,bin_height,fence_width),	// size
				ChQuaternion<>(0,1,0,0)
				);
	application.GetSystem()->Add(mfence7);
	mfence7->SetBodyFixed(true);
	mfence7->SetFriction(0.1);


	ChSharedPtr<ChBody> mfence8 = create_box_collision_shape(
				ChVector<>(conveyor_length/2+bin_length/2-2*drumdiameter,y_posbin+bin_height/2-drumdiameter/2, bin_width/2+fence_width/2), //parete dx guardando dal fondo
				ChVector<>(bin_length,bin_height,fence_width),	// size
				ChQuaternion<>(0,1,0,0)
				);
	application.GetSystem()->Add(mfence8);
	mfence8->SetBodyFixed(true);
	mfence8->SetFriction(0.1);

	if (n == 2)
	{
        double x_splitter1=bin_length/2;
		ChSharedPtr<ChBody> mfence9 = create_box_collision_shape(
						ChVector<>(conveyor_length/2+x_splitter1-2*drumdiameter,y_posbin+splitter_height/2-drumdiameter/2+binbase,0),
						ChVector<>(splitter_width,splitter_height,bin_width),	// size
						ChQuaternion<>(0,0,1,0)
						);
		application.GetSystem()->Add(mfence9);
		mfence9->SetBodyFixed(true);
		mfence9->SetFriction(0.1);
	}
	if (n == 3)
	{
		double x_splitter1=bin_length/3;
        double x_splitter2=2*bin_length/3;
		ChSharedPtr<ChBody> mfence9 = create_box_collision_shape(
						ChVector<>(conveyor_length/2+x_splitter1-2*drumdiameter,y_posbin+splitter_height/2-drumdiameter/2+binbase,0),
						ChVector<>(splitter_width,splitter_height,bin_width),	// size
						ChQuaternion<>(0,0,1,0)
						);
		application.GetSystem()->Add(mfence9);
		mfence9->SetBodyFixed(true);
		mfence9->SetFriction(0.1);
		ChSharedPtr<ChBody> mfence10 = create_box_collision_shape(
						ChVector<>(conveyor_length/2+x_splitter2-2*drumdiameter,y_posbin+splitter_height/2-drumdiameter/2+binbase,0),
						ChVector<>(splitter_width,splitter_height,bin_width),	// size
						ChQuaternion<>(0,0,1,0)
						);
		application.GetSystem()->Add(mfence10);
		mfence10->SetBodyFixed(true);
		mfence10->SetFriction(0.1);
	}
	if (n == 4)
	{
		double x_splitter1=bin_length/2;
        double x_splitter2=bin_length/4;
        double x_splitter3=3*bin_length/4;
		ChSharedPtr<ChBody> mfence9 = create_box_collision_shape(
						ChVector<>(conveyor_length/2+x_splitter1-2*drumdiameter,y_posbin+splitter_height/2-drumdiameter/2+binbase,0),
						ChVector<>(splitter_width,splitter_height,bin_width),	// size
						ChQuaternion<>(0,0,1,0)
						);
		application.GetSystem()->Add(mfence9);
		mfence9->SetBodyFixed(true);
		mfence9->SetFriction(0.1);
		ChSharedPtr<ChBody> mfence10 = create_box_collision_shape(
						ChVector<>(conveyor_length/2+x_splitter2-2*drumdiameter,y_posbin+splitter_height/2-drumdiameter/2+binbase,0),
						ChVector<>(splitter_width,splitter_height,bin_width),	// size
						ChQuaternion<>(0,0,1,0)
						);
		application.GetSystem()->Add(mfence10);
		mfence10->SetBodyFixed(true);
		mfence10->SetFriction(0.1);
		ChSharedPtr<ChBody> mfence11 = create_box_collision_shape(
						ChVector<>(conveyor_length/2+x_splitter3-2*drumdiameter,y_posbin+splitter_height/2-drumdiameter/2+binbase,0),
						ChVector<>(splitter_width,splitter_height,bin_width),	// size
						ChQuaternion<>(0,0,1,0)
						);
		application.GetSystem()->Add(mfence11);
		mfence11->SetBodyFixed(true);
		mfence11->SetFriction(0.1);
	}

	ChSharedPtr<ChBody> mfence12 = create_box_collision_shape(
						ChVector<>(xnozzle-xnozzlesize/2-fence_width/2,ynozzlesize/2+ynozzle+binbase,0),
						ChVector<>(fence_width,ynozzlesize,znozzlesize+2*fence_width),
						ChQuaternion<>(0,0,1,0)
						);
	application.GetSystem()->Add(mfence12);
	mfence12->SetBodyFixed(true);
	mfence12->SetFriction(0.1);

	ChSharedPtr<ChBody> mfence13 = create_box_collision_shape(
						ChVector<>(xnozzle+xnozzlesize/2+fence_width/2,ynozzlesize/2+ynozzle+binbase,0),
						ChVector<>(fence_width,ynozzlesize,znozzlesize+2*fence_width),
						ChQuaternion<>(0,0,1,0)
						);
	application.GetSystem()->Add(mfence13);
	mfence13->SetBodyFixed(true);
	mfence13->SetFriction(0.1);

	ChSharedPtr<ChBody> mfence14 = create_box_collision_shape(
						ChVector<>(xnozzle,ynozzlesize/2+ynozzle+binbase,znozzlesize/2+fence_width/2),
						ChVector<>(xnozzlesize,ynozzlesize,fence_width),
						ChQuaternion<>(0,1,0,0)
						);
	application.GetSystem()->Add(mfence14);
	mfence14->SetBodyFixed(true);
	mfence14->SetFriction(0.1);

	ChSharedPtr<ChBody> mfence15 = create_box_collision_shape(
						ChVector<>(xnozzle,ynozzlesize/2+ynozzle+binbase,-znozzlesize/2-fence_width/2),
						ChVector<>(xnozzlesize,ynozzlesize,fence_width),
						ChQuaternion<>(0,1,0,0)
						);
	application.GetSystem()->Add(mfence15);
	mfence15->SetBodyFixed(true);
	mfence15->SetFriction(0.1);

	
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

	//
	// Create the conveyor belt (this is a pure Chrono::Engine object, 
	// because an Irrlicht 'SceneNode' wrapper is not yet available, so it is invisible - no 3D preview)
	//

	
	ChSharedPtr<ChConveyor> mconveyor (new ChConveyor(conveyor_length, conv_thick, conveyor_width));
	mconveyor->SetBodyFixed(true);
	mconveyor->SetFriction(0.9);
    mconveyor->SetRollingFriction(0.01);
	mconveyor->SetConveyorSpeed(STATIC_speed);

	mconveyor->SetPos( ChVector<>(0, 0-conv_thick, 0) );

	mconveyor->GetPlate()->GetCollisionModel()->SetFamily(2); // note the additional  ->GetPlate()
    mconveyor->GetPlate()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1); 
	mconveyor->GetPlate()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3); 

	mphysicalSystem.Add(mconveyor);

	// Attach a visualization shape asset (optional). 
	ChSharedPtr<ChBoxShape> mbox(new ChBoxShape);
	mbox->GetBoxGeometry().SetLenghts(ChVector<>(conveyor_length,conv_thick, conveyor_width));
	mconveyor->AddAsset(mbox);


	//
	// Create a collision shape for the rotating drum (end of the belt)
	//

	double drummass = 100;
	double Ixx = 25;
	double drumradius = 0.114; 
	ChSharedPtr<ChBody> mrigidBodyDrum(new ChBody);

	mrigidBodyDrum->SetPos(ChVector<>(conveyor_length/2, -drumradius-conv_thick/2,0));
	mrigidBodyDrum->SetRot(ChQuaternion<> (pow(2,0.5)/2,pow(2,0.5)/2,0,0) );
	mrigidBodyDrum->SetMass(drummass);
	mrigidBodyDrum->SetInertiaXX(ChVector<>(Ixx,Ixx,Ixx));
	mrigidBodyDrum->SetFriction(0.9f); 
	mrigidBodyDrum->SetRollingFriction(0.01);
	
	// Define a collision shape 
	mrigidBodyDrum->GetCollisionModel()->ClearModel();
	mrigidBodyDrum->GetCollisionModel()->AddCylinder(drumradius,drumradius,conveyor_width);
	mrigidBodyDrum->GetCollisionModel()->BuildModel();
	mrigidBodyDrum->SetCollide(true);

	mrigidBodyDrum->GetCollisionModel()->SetFamily(3);
	mrigidBodyDrum->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
	mrigidBodyDrum->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);

		// Finally, do not forget to add the body to the system:
	application.GetSystem()->Add(mrigidBodyDrum);


	// Attach a visualization shape asset. 
	ChSharedPtr<ChCylinderShape> mcyl(new ChCylinderShape);
	mcyl->GetCylinderGeometry().p1 = ChVector<>(0 , -0.5*conveyor_width, 0);
	mcyl->GetCylinderGeometry().p2 = ChVector<>(0 ,  0.5*conveyor_width, 0);
	mcyl->GetCylinderGeometry().rad = drumradius;
	mrigidBodyDrum->AddAsset(mcyl);
	
	

	// Optional: attach a 'blue' texture to easily recognize metal stuff in 3d view
	ChSharedPtr<ChTexture> mtexturedrum(new ChTexture);
	mtexturedrum->SetTextureFilename("../objects/bluwhite.png");
	mrigidBodyDrum->AddAsset(mtexturedrum);



	//
	// Create a truss (absolute fixed reference body, for connecting the rotating cyl.)
	//

	ChSharedPtr<ChBody> mtruss(new ChBody);
	mtruss->SetBodyFixed(true);

	// Finally, do not forget to add the body to the system:
	application.GetSystem()->Add(mtruss);



	// 
	// Create a motor constraint between the cylinder and the truss
	//

	ChCoordsys<> drum_axis_coordsys(ChCoordsys<>(ChVector<>(conveyor_length/2, -drumradius-conv_thick/2,0)));

	ChSharedPtr<ChLinkEngine> mengine(new ChLinkEngine);
	mengine->Initialize(mrigidBodyDrum, mtruss, drum_axis_coordsys);

	mengine->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
	if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(mengine->Get_spe_funct()))
		mfun->Set_yconst(-STATIC_speed/(drumdiameter*0.5));

	// Finally, do not forget to add the body to the system:
	application.GetSystem()->Add(mengine);




	// Electrode 

	double elecradius = 0.019; //***************ida

	ChBodySceneNode* mElec = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
		                                         application.GetSystem(), application.GetSceneManager(),
												1,
												ChVector<>(4*(drumradius+(L*cos(alpha)-drumradius)),drumradius+(sin(alpha)*L-drumradius),0),
												ChQuaternion<> (pow(2,0.5)/2,pow(2,0.5)/2,0,0), 
												ChVector<>(elecradius*2,conveyor_width,elecradius*2));
	mElec->GetBody()->SetBodyFixed(true);
    //mElec->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);  not needed
	//mElec->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);
	//mElec->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);


	
	//********************************************************************** ida
	// Create a collision shape for the rotating brush 
	//

	double brushmass = 25;
	double Ixx2 = 6.25;
	double brushradius = 0.02; 
	ChSharedPtr<ChBody> mrigidBodyBrush(new ChBody);

	mrigidBodyBrush->SetPos(ChVector<>(conveyor_length/2-drumradius-brushradius, -drumradius-brushradius-conv_thick/2,0));//***************************************************
	mrigidBodyBrush->SetRot(ChQuaternion<> (pow(2,0.5)/2,pow(2,0.5)/2,0,0) );
	mrigidBodyBrush->SetMass(brushmass);
	mrigidBodyBrush->SetInertiaXX(ChVector<>(Ixx2,Ixx2,Ixx2));
	mrigidBodyBrush->SetFriction(0.9f);
	mrigidBodyBrush->SetRollingFriction(0.01);

	// Define a collision shape 
	mrigidBodyBrush->GetCollisionModel()->ClearModel();
	mrigidBodyBrush->GetCollisionModel()->AddCylinder(brushradius,brushradius,conveyor_width);
	mrigidBodyBrush->GetCollisionModel()->BuildModel();
	mrigidBodyBrush->SetCollide(true);

	mrigidBodyBrush->GetCollisionModel()->SetFamily(1);
	mrigidBodyBrush->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);
	mrigidBodyBrush->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);

	// Attach a visualization shape asset. 
	ChSharedPtr<ChCylinderShape> mcyl2(new ChCylinderShape);
	mcyl2->GetCylinderGeometry().p1 = ChVector<>(0 , -0.5*conveyor_width, 0);
	mcyl2->GetCylinderGeometry().p2 = ChVector<>(0 ,  0.5*conveyor_width, 0);
	mcyl2->GetCylinderGeometry().rad = brushradius;
	mrigidBodyBrush->AddAsset(mcyl2);

	// Finally, do not forget to add the body to the system:
	application.GetSystem()->Add(mrigidBodyBrush);


	// Optional: attach a 'pink' texture to easily recognize metal stuff in 3d view
	ChSharedPtr<ChTexture> mtexturebrush(new ChTexture);
	mtexturebrush->SetTextureFilename("../objects/pinkwhite.png");
	mrigidBodyBrush->AddAsset(mtexturebrush);


	//
	// Create a truss (absolute fixed reference body, for connecting the rotating cyl.)
	//

	ChSharedPtr<ChBody> mtruss2(new ChBody);
	mtruss2->SetBodyFixed(true);

	// Finally, do not forget to add the body to the system:
	application.GetSystem()->Add(mtruss2);



	// 
	// Create a motor constraint between the cylinder and the truss
	//

	ChCoordsys<> drum_axis_coordsys2(ChCoordsys<>(ChVector<>(conveyor_length/2-drumradius-brushradius, -drumradius-brushradius-conv_thick/2,0))); 

	ChSharedPtr<ChLinkEngine> mengine2(new ChLinkEngine);
	mengine2->Initialize(mrigidBodyBrush, mtruss2, drum_axis_coordsys2);

	mengine2->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
	if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(mengine2->Get_spe_funct()))
		mfun->Set_yconst(-STATIC_speed/(drumdiameter*0.5));

	// Finally, do not forget to add the body to the system:
	application.GetSystem()->Add(mengine2); //********** ida

	
	//
	// Create an (optional) exporter to POVray 
	// 
	ChPovRay pov_exporter = ChPovRay(&mphysicalSystem);

	if (save_POV_screenshots)
	{
				// Sets some file names for in-out processes.
		pov_exporter.SetTemplateFile		("../objects/_template_POV.pov");
		pov_exporter.SetOutputScriptFile    ("rendering_frames.pov");
			
				// save the .dat files and the .bmp files
				// in two subdirectories, to avoid cluttering the current directory...
		ChFileutils::MakeDirectory("outputPOV");
		ChFileutils::MakeDirectory("animPOV");

		pov_exporter.SetOutputDataFilebase("outputPOV/my_state");
		pov_exporter.SetPictureFilebase("animPOV/picture");

				// optional: modify the POV default light
		pov_exporter.SetLight(ChVector<>(1.5f,4.4f,-1.0f), ChColor(0.1,0.1,0.1), false);

		pov_exporter.SetCamera(ChVector<>(1.5f,0.8f,-1.0f),ChVector<>(0.5f,0.f,0.f),60,false);

				// optional: use SetCustomPOVcommandsScript() to add further POV commands,
				// ex. create an additional light, and an additional grid, etc. 
				// Remember the "\" char per each newline.
		
		pov_exporter.SetCustomPOVcommandsScript(" \
			light_source {   \
			  <2, 10, -3>  \
			  color rgb<1.8,1.8,1.8> \
			  area_light <4, 0, 0>, <0, 0, 4>, 5, 5 \
			  adaptive 1 \
			  jitter\
			} \
			object{ Grid(0.5,0.01, rgb<0.9,0.9,0.9>, rgbt<1,1,1,1>) rotate <90, 0, 0>  } \
			object{ Grid(0.1,0.04, rgb<1.5,1.5,1.5>, rgbt<1,1,1,1>) rotate <90, 0, 0> translate 0.001*z} \
		");

				// IMPORTANT! Tell to the POVray exporter that 
				// he must take care of converting the shapes of
				// all items!
		pov_exporter.AddAll();

				// IMPORTANT! Create the two .pov and .ini files for POV-Ray (this must be done
				// only once at the beginning of the simulation).
		pov_exporter.ExportScript();

	}


	//
	// For enabling Irrlicht visualization of assets (that have been added so far)
	//

	application.AssetBindAll();
	application.AssetUpdateAll();
	if (irr_cast_shadows)
		application.AddShadowAll();


	// 
	// THE SOFT-REAL-TIME CYCLE
	//
	
	application.SetStepManage(true);
	application.SetTimestep(0.001);
	
	application.GetSystem()->SetIntegrationType(ChSystem::INT_TASORA);
	application.GetSystem()->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD); // or ChSystem::LCP_ITERATIVE_BARZILAIBORWEIN for max precision
		// important! dt is small, and particles are small, so it's better to keep this small...
	application.GetSystem()->SetMaxPenetrationRecoverySpeed(0.15); // not needed in INT_TASORA, only for INT_ANITESCU


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
	
	int savenum = 0;

	ChFileutils::MakeDirectory("screenshots");
	ChFileutils::MakeDirectory("output");


	while(application.GetDevice()->run()) 
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();

		if (!application.GetPaused())
		{ 
			application.DoStep();

			totframes++;

			apply_forces (	&mphysicalSystem,		// contains all bodies
							drum_axis_coordsys, // pos and rotation of axis of drum (not rotating reference!)
							drumspeed,		 // speed of drum
							numberofpoles,	 // number of couples of poles
							intensity,		 // intensity of the magnetic field
							drumdiameter,
							h1,
							h2,
							L,
							electrodediameter,
							j,
							alpha,
							U,
							f,
							totframes);

			fall_point (	application,
							&mphysicalSystem,		// contains all bodies
							mrigidBodyDrum->GetFrame_REF_to_abs().GetCoord(), // pos and rotation of drum 
							threshold);

			if (receiver.checkbox_plotforces->isChecked())
				draw_forces ( application , 1000);

			if (receiver.checkbox_plottrajectories->isChecked())
				DrawTrajectories(application);

			
			// Continuosly create debris that fall on the conveyor belt
			create_debris(application.GetTimestep(), STATIC_flow, *application.GetSystem(), &application, &pov_exporter);

			// Limit the max age (in seconds) of debris particles on the scene, 
			// deleting the oldest ones, for performance
			purge_debris (*application.GetSystem(),6);

			// Maybe the user played with the slider and changed STATIC_speed...
		 	mconveyor->SetConveyorSpeed(STATIC_speed);
			// ..also for rotating drum:
			if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(mengine->Get_spe_funct()))
				mfun->Set_yconst(-STATIC_speed/(drumdiameter*0.5));

			// update the assets containing the trajectories, if any
			if (receiver.checkbox_plottrajectories->isChecked())
				if (totframes % 20 == 0)
					UpdateTrajectories(application);

			// Save data on file (each n integration steps, to avoid filling
			// the hard disk and to improve performance)
			if (totframes % saveEachNframes == 0)
			{
				savenum++;

				// Save log file as '.txt' files?

				if (save_dataset == true)
				{
					char buffer[120];
					sprintf(buffer, "output/esempio_output%05d.txt", savenum);
					GetLog() << "\n saving dataset: " << buffer;
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
								ChVector<> my_ElectricForce = electricproperties->ElectricForce;
								ChVector<> my_ElectricImageForce = electricproperties->ElectricImageForce;
								ChVector<> my_StokesForce = electricproperties->StokesForce;
								
								// Save on disk some infos...
								file_for_output << abody->GetIdentifier()<< ", "
												<< abody->GetPos().x << ", "
												<< abody->GetPos().y << ", "
												<< abody->GetPos().z << ", "
												<< my_cond << ", "
												<< abody->GetMass()<< "\n";
												//<< abody->GetPos_dt().x << ", "
												//<< abody->GetPos_dt().y << ", "
												//<< abody->GetPos_dt().z << ", "
												//<< my_StokesForce << ", "
												//<< my_ElectricImageForce << ", "
												//<< my_ElectricForce << "\n";
						                       
								               
							}
						}
					}
				}
				
				// Save Irrlicht screenshots?

				if (save_irrlicht_screenshots == true)
				{
					video::IImage* image = application.GetVideoDriver()->createScreenShot();
					char buffer[120];
					sprintf(buffer, "screenshots/screenshot%05d.bmp", savenum);
					GetLog() << "\n saving screenshot: " << buffer;
					if (image)
						application.GetVideoDriver()->writeImageToFile(image, buffer);
					image->drop();
				}

				// Save POV screenshots?

				if(save_POV_screenshots)
				{
					pov_exporter.ExportData();
					GetLog() << "\n saving POV data n." << savenum;
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