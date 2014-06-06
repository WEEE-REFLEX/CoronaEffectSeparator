#ifndef SIMULATORCES_H
#define SIMULATORCES_H

///
/// HEADER for the simulator of the Corona Electrostatic Separator
/// for the WEEE project
/// Authors I. Critelli, A. Tasora, 2014
///



#include "physics/CHapidll.h" 
#include "physics/CHsystem.h"
#include "physics/CHconveyor.h"
#include "physics/CHbodyAuxRef.h"
#include "core/ChFileutils.h"
#include "irrlicht_interface/CHbodySceneNode.h"
#include "irrlicht_interface/CHbodySceneNodeTools.h" 
#include "irrlicht_interface/CHirrApp.h"
#include "core/ChRealtimeStep.h"
#include "core/ChMath.h"
#include "core/ChDistribution.h"
//#include <irrlicht.h>
#include <fstream>
#include "unit_PYTHON/ChPython.h"
#include "unit_POSTPROCESS/ChPovRay.h"

#include "ElectricParticleProperty.h"
#include "UserInterfaceEventReceiver.h"

// Use the namespace of Chrono

using namespace chrono;
using namespace postprocess;

// Use the main namespaces of Irrlicht

using namespace std;

// Static values valid through the entire program (bad
// programming practice, but enough for quick tests)



class SimulatorCES
{

public:
			// 
			// DATA of the simulator
			//

	double STATIC_rpm;
	double STATIC_flow;

	double epsilon; // dielectric constant [F/m] *****ida 
	double epsilonO; //vacuum permeability
	double epsilonR; //relative permeability
	double drumspeed; //[rad/s]
	double drumdiameter;
	double STATIC_speed; //[m/s]
	double eta; // Air drag coefficent [N*s/m^2]
	double electrodediameter;
	double U; // supplied high-voltage [v]
	double L; //certer distance of rotating roll electrode and electrostatic pole *****ida
	double alpha; //angle of horizontal line and electrodes center line *****ida
	double h1; //analytical parameter****ida
	double h2;//analytical parameter****ida
	double j;//analytical parameter****ida
	double f;//analytical parameter****ida
	double sphrad;
	double sphrad2;
	double sphrad3;
	double particles_dt;
	double debris_number;
	double max_numb_particles;


	// conveyor constant
	double conveyor_length;//***ALEX, from CAD
	double conveyor_width; //***ALEX, from CAD, was 0.6
	double conv_thick; // non troppo sottile, altrimenti non va la collision detection! Non importa se compenetra il cilindro.

	double ro;  //fluid density (air) [Kg/m^3]

	// fence constant
	double fence_width;
	double fence_height;
	// bin constant 
	double y_posbin;
	double binbase; // base thickness of the bin
	double bin_length; // accorciato da 3 a 1, ida
	double bin_width;
	double bin_height;
	double n; // number of bins (values from 2 to 4), a regime devo importare il cad dei 3 contenitori, ida
	// splitter constant
	double x_splitter1;
	double x_splitter2;
	double x_splitter3;
	double splitter_width;
	double splitter_height;
	// hopper constant
	double xnozzlesize;//0.2;
	double znozzlesize; //**from CAD, nozzle width
	double ynozzlesize;//0.5;
	double ynozzle;
	double xnozzle; //portato avanti****ida

	double densityMetal; // sn-pb //8900;//rame//1820 vecchia densità;
	double densityPlastic;// polipropilene //900 vecchia densità;
	int myid;

	// Coordinate systems with position and rotation of important items in the 
	// simulator. These are initializad with constant values, but if loading the
	// SolidWorks model, they will be changed accordingly to what is found in the CAD 
	// file (see later, where the SolidWorks model is parsed). 

	ChCoordsys<> conveyor_csys;
	ChCoordsys<> drum_csys;
	ChCoordsys<> nozzle_csys;
	ChCoordsys<> Splitter1_csys;
	ChCoordsys<> Splitter2_csys;
	ChCoordsys<> Spazzola_csys;


	// set as true for saving log files each n frames
	bool save_dataset;
	bool save_irrlicht_screenshots;
	bool save_POV_screenshots;
	int saveEachNframes;

	bool irr_cast_shadows;

	int totframes;
		
	bool init_particle_speed;

	double particle_magnification; // for larger visualization of particle



		///
		/// Create the SimulatorCES
		/// and initialize member data
		/// 
	SimulatorCES()
	{
			// initialize member data:
		STATIC_rpm = 44.8;
		STATIC_flow = 1000;

		epsilon = 8.85941e-12; // dielectric constant [F/m] *****ida 
		epsilonO = 8.854187e-12; //vacuum permeability
		epsilonR = 2.5; //relative permeability
		drumspeed = STATIC_rpm*((2.0*CH_C_PI)/60.0); //[rad/s]
		drumdiameter = 0.320;
		STATIC_speed = (drumdiameter/2.0)*(STATIC_rpm*((2.0*CH_C_PI)/60.0)); //[m/s]
		eta = 0.0000181; // Air drag coefficent [N*s/m^2]
		electrodediameter = 0.038;
		U = 30000; // supplied high-voltage [v]
		L = 0.267; //certer distance of rotating roll electrode and electrostatic pole *****ida
		alpha = (CH_C_PI/180)*30; //angle of horizontal line and electrodes center line *****ida
		h1 = (pow(L,2)+pow((drumdiameter/2),2)-((electrodediameter/2),2))/(2*L); //analytical parameter****ida
		h2 = (pow(L,2)-pow((drumdiameter/2),2)+((electrodediameter/2),2))/(2*L);//analytical parameter****ida
		j = sqrt(pow(h1,2)-pow((drumdiameter/2),2));//analytical parameter****ida
		f = U/log(((h1+j-(drumdiameter/2))*(h2+j-(electrodediameter/2)))/((drumdiameter/2)+j-h1)*((electrodediameter/2)+j-h2));//analytical parameter****ida
		sphrad = 0.38e-3;
		sphrad2 = 0.25e-3;
		sphrad3 = 0.794e-3;
		particles_dt;
		debris_number = 0;
		max_numb_particles = 100;


		// conveyor constant
		conveyor_length=0.400;//***ALEX, from CAD
		conveyor_width=0.3; //***ALEX, from CAD, was 0.6
		conv_thick = 0.01; // non troppo sottile, altrimenti non va la collision detection! Non importa se compenetra il cilindro.

		ro=1.225;  //fluid density (air) [Kg/m^3]

		// fence constant
		fence_width = 0.02;
		fence_height=0.2;
		// bin constant 
		y_posbin=-0.815;
		binbase=0.02; // base thickness of the bin
		bin_length=1; // accorciato da 3 a 1, ida
		bin_width=1.5;
		bin_height=0.2;
		n = 2; // number of bins (values from 2 to 4), a regime devo importare il cad dei 3 contenitori, ida
		// splitter constant
		x_splitter1=0;
		x_splitter2=0;
		x_splitter3=0;
		splitter_width=0.01;
		splitter_height=0.4;
		// hopper constant
		xnozzlesize = 0.1;//0.2;
		znozzlesize = 0.182; //**from CAD, nozzle width
		ynozzlesize = 0.1;//0.5;
		ynozzle = 0.01;
		xnozzle = -conveyor_length/2+xnozzlesize/2+fence_width; //portato avanti****ida

		densityMetal =  8400; // sn-pb //8900;//rame//1820 vecchia densità;
		densityPlastic = 946;// polipropilene //900 vecchia densità;
		myid = 1;

		// Coordinate systems with position and rotation of important items in the 
		// simulator. These are initializad with constant values, but if loading the
		// SolidWorks model, they will be changed accordingly to what is found in the CAD 
		// file (see later, where the SolidWorks model is parsed). 

		conveyor_csys	= ChCoordsys<>( ChVector<>(0, 0-conv_thick, 0) ) ; // default position
		drum_csys		= ChCoordsys<>( ChVector<>(conveyor_length/2, -(drumdiameter*0.5)-conv_thick/2,0) );  // default position
		nozzle_csys		= ChCoordsys<>( ChVector<>(xnozzle, ynozzle, 0) ); // default position
		Splitter1_csys	= ChCoordsys<>( ChVector<>(conveyor_length/2+0.2, -(drumdiameter*0.5)-conv_thick/2,0) );  // default position
		Splitter2_csys	= ChCoordsys<>( ChVector<>(conveyor_length/2+0.4, -(drumdiameter*0.5)-conv_thick/2,0) );  // default position
		Spazzola_csys	= ChCoordsys<>( ChVector<>(conveyor_length/2-0.10, -(drumdiameter*0.5)-conv_thick/2,0) );  // default position


		// set as true for saving log files each n frames
		save_dataset = false;
		save_irrlicht_screenshots = false;
		save_POV_screenshots = false;
		saveEachNframes = 3;

		irr_cast_shadows = true;

		totframes = 0;
			
		init_particle_speed = true;

		particle_magnification = 3; // for larger visualization of particle

	}



		//
		// This can be added to store the trajectory on a per-particle basis.
		//
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





		/// Shortcut for easy creation of a body that has both a box visualization and a box collision,
		/// with default values for mass inertia etc. 
		///

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


		///
		/// Function that creates debris that fall on 
		/// the conveyor belt, to be called at each dt
		///
	void create_debris(double dt, double particles_second, 
					   ChCoordsys<> mnozzle_csys,
					   ChSystem& mysystem, 
					   ChIrrApp* irr_application,
					   ChPovRay* mpov_exporter)
	{

		double sph_fraction = 0.33;
		double sph2_fraction = 0;
		double sph3_fraction = 0.67;
		double box_fraction = 0;
		double cyl_fraction = 1-box_fraction-(sph_fraction + sph2_fraction + sph3_fraction);

		//double sphrad = 0.6e-3 + (ChRandom()-0.5)*(0.6e-3); vecchia distribuzione
		//double sphrad = 0.5e-3; messa come variabile globale per esportazione e postprocessing
		double cylhei = 0.035;
		double cylrad = sphrad;
		double cylmass = CH_C_PI*pow(cylrad,2)*cylhei* 1.0;  // now with default 1.0 density
		double sphmass = (4./3.)*CH_C_PI*pow(sphrad,3)* 1.0; // now with default 1.0 density
		double sphmass2 = (4./3.)*CH_C_PI*pow(sphrad2,3)* 1.0; // now with default 1.0 density
		double sphmass3 = (4./3.)*CH_C_PI*pow(sphrad3,3)* 1.0; // now with default 1.0 density
		double sphinertia = 0.4*pow(sphrad,2)*sphmass;      // now with default 1.0 density
		double sphinertia2 = 0.4*pow(sphrad2,2)*sphmass2;      // now with default 1.0 density
		double sphinertia3 = 0.4*pow(sphrad3,2)*sphmass3;      // now with default 1.0 density
		double cylinertia = 0.0833*(pow(cylhei,2)+3*pow(sphrad,2))*cylmass;//0.0833*(pow(cylhei,2)+3*pow(sphrad,2))*cylmass;  // now with default 1.0 density
		double cylinertia2 = 0.5*pow(sphrad,2)*cylmass; //0.5*pow(sphrad,2)*cylmass; // now with default 1.0 density
		

		double exact_particles_dt = dt * particles_second;
		//double particles_dt= floor(exact_particles_dt);
		particles_dt= floor(exact_particles_dt);
		double remaind = exact_particles_dt - particles_dt;

	    
		if (remaind > ChRandom()) particles_dt +=1;

	    
		for (int i = 0; i < particles_dt; i++)
		{
			ChSharedPtr<ChBody> created_body;
			ChSharedPtr<ElectricParticleProperty> created_electrical_asset;

			double rand_shape_fract = ChRandom();

			double max_angvel = 12;
			ChVector<> rand_position = mnozzle_csys.TrasformLocalToParent( ChVector<>(-0.5*xnozzlesize+ChRandom()*xnozzlesize, 0, -0.5*znozzlesize+ChRandom()*znozzlesize) ) ;
			ChVector<> rand_velocity =(0.4 + (ChRandom()-0.5)*(0.4), 0,0);
			ChVector<> rand_angvel =((ChRandom()-0.5)*2*(max_angvel), 
									 (ChRandom()-0.5)*2*(max_angvel),
									 (ChRandom()-0.5)*2*(max_angvel));
			//
			// 1 ---- Create particle 
			// 

			if (rand_shape_fract < sph_fraction)
			{
				// Create a body
				ChSharedPtr<ChBody> mrigidBody(new ChBody);

				mrigidBody->SetPos(rand_position);
				mrigidBody->SetPos_dt(rand_velocity);
				mrigidBody->SetWvel_par(rand_angvel);
				mrigidBody->SetMass(sphmass);
				mrigidBody->SetInertiaXX(ChVector<>(sphinertia,sphinertia,sphinertia));
				mrigidBody->SetFriction(0.2f);
				mrigidBody->SetImpactC(0.75f);
				mrigidBody->SetIdentifier(myid); // NB fatto solo per le sfere!!!!!!!!!
				
				       
				mrigidBody->SetRollingFriction(0.2);
				mrigidBody->SetSpinningFriction(0.2);


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
			}//**********************************************************************************************************************
			if ((rand_shape_fract > sph_fraction) && (rand_shape_fract < sph_fraction + sph2_fraction))
			{
				// Create a body
				ChSharedPtr<ChBody> mrigidBody(new ChBody);

				mrigidBody->SetPos(rand_position);
				mrigidBody->SetPos_dt(rand_velocity);
				mrigidBody->SetWvel_par(rand_angvel);
				mrigidBody->SetMass(sphmass2);
				mrigidBody->SetInertiaXX(ChVector<>(sphinertia2,sphinertia2,sphinertia2));
				mrigidBody->SetFriction(0.2f);
				//mrigidBody->SetImpactC(0.75f); 
				mrigidBody->SetIdentifier(myid); // NB fatto solo per le sfere!!!!!!!!!
				      
				// Define a collision shape 
				mrigidBody->GetCollisionModel()->ClearModel();
				mrigidBody->GetCollisionModel()->AddSphere(sphrad2);
				mrigidBody->GetCollisionModel()->BuildModel();
				mrigidBody->SetCollide(true);

				// Attach a visualization shape asset. 
				ChSharedPtr<ChSphereShape> msphere(new ChSphereShape);
				msphere->GetSphereGeometry().rad = sphrad2 * particle_magnification; 
				msphere->GetSphereGeometry().center = ChVector<>(0,0,0);
				mrigidBody->AddAsset(msphere);
		
				// Attach a custom asset. Look how to create and add a custom asset to the object! ***ALEX
				ChSharedPtr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty); 
				electric_asset->Cdim         = ChVector<>(2*sphrad2,2*sphrad2,2*sphrad2);
				electric_asset->fraction	 = ElectricParticleProperty::e_fraction_sphere;
				electric_asset->birthdate	 = mysystem.GetChTime();
				mrigidBody->AddAsset(electric_asset);
				
				// Finally, do not forget to add the body to the system:
				mysystem.Add(mrigidBody);

				created_body = mrigidBody;
				created_electrical_asset = electric_asset; // for reference later

			}
		   if (((rand_shape_fract > sph_fraction + sph2_fraction)) && 
			   (rand_shape_fract < (sph_fraction + sph2_fraction + sph3_fraction)))
			{
				// Create a body
				ChSharedPtr<ChBody> mrigidBody(new ChBody);

				mrigidBody->SetPos(rand_position);
				mrigidBody->SetPos_dt(rand_velocity);
				mrigidBody->SetWvel_par(rand_angvel);
				mrigidBody->SetMass(sphmass3);
				mrigidBody->SetInertiaXX(ChVector<>(sphinertia3,sphinertia3,sphinertia3));
				mrigidBody->SetFriction(0.2f);
				//mrigidBody->SetImpactC(0.75f); 
				mrigidBody->SetIdentifier(myid); // NB fatto solo per le sfere!!!!!!!!!
				      
				// mrigidBody->SetRollingFriction(0.1);
				// mrigidBody->SetSpinningFriction(0.1);


				// Define a collision shape 
				mrigidBody->GetCollisionModel()->ClearModel();
				mrigidBody->GetCollisionModel()->AddSphere(sphrad3);
				//mrigidBody->GetCollisionModel()->AddSphere(sphrad, &ChVector<>(0.005,0,0)); // etc. optional
				mrigidBody->GetCollisionModel()->BuildModel();
				mrigidBody->SetCollide(true);

				// Attach a visualization shape asset. 
				ChSharedPtr<ChSphereShape> msphere(new ChSphereShape);
				msphere->GetSphereGeometry().rad = sphrad3 * particle_magnification; 
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
				electric_asset->Cdim         = ChVector<>(2*sphrad3,2*sphrad3,2*sphrad3);
				electric_asset->fraction	 = ElectricParticleProperty::e_fraction_sphere;
				electric_asset->birthdate	 = mysystem.GetChTime();
				mrigidBody->AddAsset(electric_asset);
				
				// Finally, do not forget to add the body to the system:
				mysystem.Add(mrigidBody);

				created_body = mrigidBody;
				created_electrical_asset = electric_asset; // for reference later

			}
			//***********************************************************************************************************************
			if ((rand_shape_fract > (sph_fraction + sph2_fraction + sph3_fraction)) && 
				(rand_shape_fract < box_fraction+(sph_fraction + sph2_fraction + sph3_fraction)))
			{
				double xscale = 1.3*(1-0.8*ChRandom()); // for oddly-shaped boxes..
				double yscale = 1.3*(1-0.8*ChRandom());
				double zscale = 1.3*(1-0.8*ChRandom());

				//	ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
				//	ChCollisionModel::SetDefaultSuggestedMargin  (0.002);

				// Create a body
				ChSharedPtr<ChBody> mrigidBody(new ChBody);

				mrigidBody->SetPos(rand_position);
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

			if ((rand_shape_fract > box_fraction+(sph_fraction + sph2_fraction + sph3_fraction)))
			{

				//	ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
				//	ChCollisionModel::SetDefaultSuggestedMargin  (0.002);

				// Create a body
				ChSharedPtr<ChBody> mrigidBody(new ChBody);

				mrigidBody->SetPos(rand_position);
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
				  
				//double rand_mat = ChRandom();

				//double plastic_fract = 0.7;
					
				if (rand_shape_fract > sph_fraction+sph2_fraction+box_fraction+cyl_fraction) //(rand_mat < plastic_fract)
				{
					created_electrical_asset->conductivity = 0;
					created_electrical_asset->material_type = ElectricParticleProperty::e_mat_plastic;

					// Attach a 'pink' texture to easily recognize plastic stuff in 3d view
					ChSharedPtr<ChTexture> mtexture(new ChTexture);
					mtexture->SetTextureFilename("../objects/pinkwhite.png");
					created_body->AddAsset(mtexture);

					// Multiply the default mass & intertia tensor by density (previously assumed =1)
					
					created_body->SetDensity(densityPlastic);
					created_body->SetMass( created_body->GetMass() * densityPlastic);
					created_body->SetInertiaXX( created_body->GetInertiaXX() * densityPlastic);
				} 
				if (rand_shape_fract < sph_fraction+sph2_fraction+box_fraction+cyl_fraction) //(rand_mat > plastic_fract)
				{
					created_electrical_asset->conductivity = 6428000;//6670000 conducibilità vecchia;
					created_electrical_asset->material_type = ElectricParticleProperty::e_mat_metal;

					// Attach a 'blue' texture to easily recognize metal stuff in 3d view
					ChSharedPtr<ChTexture> mtexture(new ChTexture);
					mtexture->SetTextureFilename("../objects/bluwhite.png");
					created_body->AddAsset(mtexture);

					// Multiply the default mass & intertia tensor by density (previously assumed =1)
			
					created_body->SetDensity(densityMetal);
					created_body->SetMass( created_body->GetMass() * densityMetal);
					created_body->SetInertiaXX( created_body->GetInertiaXX() * densityMetal);
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
				created_body->SetMaxSpeed(5);
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

			// 
			// 6 --- set initial speed to particles as soon as created, so they do not have transient slip with belt
			//

			if (init_particle_speed)
			{
				created_body->SetPos_dt(ChVector<>(0,0,0));
				//created_body->SetPos_dt(ChVector<>(STATIC_speed, 0,0));
			}

		}

	} 


	     
		///	  
		/// Function that deletes old debris 
		/// (to avoid infinite creation that fills memory)
		///
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

		///
		/// Function that defines the forces on the debris ****ida
		///
	void apply_forces (	ChSystem* msystem,		// contains all bodies
							ChCoordsys<>& drum_csys, // pos and rotation of drum 
							double drumspeed,		 // speed of drum
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
						if (electricproperties->chargeM == 0)
						{
						electricproperties->chargeM = 0.666666666666667*pow(CH_C_PI,3)*epsilon*pow(average_rad,2)*E;
						electricproperties->chargeM *= (1.0 - 0.3*ChRandom() );
						
						}
					}

					
					ChVector<> ElectricForce = electricproperties->ElectricForce;

					electricproperties->ElectricForce = 0.832 * electricproperties->chargeM * vE;

					// switch off electric forces if too out-of-plane
					if ((mrelpos.z > conveyor_width*0.5) || (mrelpos.z < -conveyor_width*0.5))
						ElectricForce = 0; 

					abody->Accumulate_force(ElectricForce, abody->GetPos(), false);


				} // end if material==metal


				//
				//===== PLASTIC FORCES ==========
				//

			    

				if (electricproperties->material_type == ElectricParticleProperty::e_mat_plastic) //forze sulle particelle non metalliche
				{
					

					// charge the particle? (contact w. drum)
					if ((distx > 0.04) && (disty > 0))
					{
						if (electricproperties->chargeP == 0)
						{
							electricproperties->chargeP = 3*CH_C_PI*epsilonO*pow(2*average_rad,2)*450000*(epsilonR/(epsilonR+2)); // charge
							electricproperties->chargeP *= (1.0 - 0.3*ChRandom() );
						}
					} //15000000,750000, 
					// discharge the particle? (contact w. blade)
					if (distx < -(drumdiameter*0.5 -0.009) && (disty > -(drumdiameter*0.5 + 0.009)) || sqrt(pow(distx,2)+ pow(disty,2))> (1.03*drumdiameter*0.5))
					{
						electricproperties->chargeP = 0; // charge
					}

					ChVector<> ElectricImageForce = electricproperties->ElectricImageForce;


					electricproperties->ElectricImageForce.x = -((pow( electricproperties->chargeP,2))/(4*CH_C_PI*epsilon*pow((2*average_rad),2))*cos(atan2(disty,distx)));
					electricproperties->ElectricImageForce.y = -((pow( electricproperties->chargeP,2))/(4*CH_C_PI*epsilon*pow((2*average_rad),2))*sin(atan2(disty,distx)));
					electricproperties->ElectricImageForce.z = 0;	
							
					
					// switch off electric forces if too out-of-plane
					if ((mrelpos.z > conveyor_width*0.5) || (mrelpos.z < -conveyor_width*0.5))
						ElectricImageForce = 0; 


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
				

			} // end if(was_a_particle) , i.e. a body with electrical asset

		} // end for() loop on all bodies
	}
	 

		///
		/// Function for drawing forces
		///
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

		///
		/// Function to update trajectories. Must be
		/// called at each timestep
		///
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

		///
		/// Function to draw trajectories. Must be
		/// called at each timestep
		///
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


		///
		/// Main function of the simulator.
		/// Initialize the simulation, and
		/// Performs the simulation
		/// by running the loop of time integration
		///
	int simulate()
	{
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
			application.AddLightWithShadow(vector3df(-4.5f,5.5f,4.5f), vector3df(0.f,0.f,0.f), 10, 1.2,10.2, 30,512, video::SColorf(1.f,0.9f,0.9f));

		// This is for GUI tweaking of system parameters..
		UserInterfaceEventReceiver receiver(&application, this);
		  // note how to add the custom event receiver to the default interface:
		application.SetUserEventReceiver(&receiver);


		// Set small collision envelopes for objects that will be created from now on..
		ChCollisionModel::SetDefaultSuggestedEnvelope(0.002); 
		ChCollisionModel::SetDefaultSuggestedMargin  (0.0008);


	
		// IMPORT A SOLIDWORK MODEL 

		// 1) create the Python engine. This is necessary in order to parse the files that 
		// have been saved using the SolidWorks add-in for Chrono::Engine.

		ChPythonEngine my_python;

		// 2) loads the .py file (as saved from SolidWorks) and fill the system.
		try
		{
			my_python.ImportSolidWorksSystem("../CAD_conveyor/conveyor_Ida", mphysicalSystem);  // note, don't type the .py suffix in filename..
		}
		catch (ChException myerror)
		{
			GetLog() << myerror.what();
		}


		// 3) fetch coordinate values and objects from what was imported from CAD

		//ChCoordsys<> conveyor_csys = CSYSNORM;


		ChSharedPtr<ChMarker> my_marker = mphysicalSystem.SearchMarker("centro_nastro");
		if (my_marker.IsNull())
			GetLog() << "Error: cannot find centro_nastro marker from its name in the C::E system! \n";
		else
			conveyor_csys = my_marker->GetAbsCoord();  // fetch both pos and rotation of CAD
			
		//****Ida

		my_marker = mphysicalSystem.SearchMarker("Splitter1");
		if (my_marker.IsNull())
			GetLog() << "Error: cannot find Splitter1 marker from its name in the C::E system! \n";
		else
			Splitter1_csys = my_marker->GetAbsCoord();  // fetch both pos and rotation of CAD

		my_marker = mphysicalSystem.SearchMarker("Splitter2");
		if (my_marker.IsNull())
			GetLog() << "Error: cannot find Splitter2 marker from its name in the C::E system! \n";
		else
			Splitter2_csys = my_marker->GetAbsCoord();  // fetch both pos and rotation of CAD

		my_marker = mphysicalSystem.SearchMarker("Spazzola");
		if (my_marker.IsNull())
			GetLog() << "Error: cannot find Spazzola marker from its name in the C::E system! \n";
		else
			Spazzola_csys = my_marker->GetAbsCoord();  // fetch both pos and rotation of CAD

		//***Ida

		
		my_marker = mphysicalSystem.SearchMarker("centro_nozzle");
		if (my_marker.IsNull())
			GetLog() << "Error: cannot find centro_nozzle marker from its name in the C::E system! \n";
		else
			nozzle_csys = my_marker->GetAbsCoord();  // fetch both pos and rotation of CAD

		
		my_marker = mphysicalSystem.SearchMarker("centro_cilindro");
		if (my_marker.IsNull())
			GetLog() << "Error: cannot find centro_cilindro marker from its name in the C::E system! \n";
		else
			drum_csys = my_marker->GetAbsCoord();  // fetch both pos and rotation of CAD
			
			// fetch mrigidBodyDrum pointer! will be used for changing the friction, the collision family, and later to create the motor
		ChSharedPtr<ChBodyAuxRef> mrigidBodyDrum = mphysicalSystem.Search("drum-1");  
		if (mrigidBodyDrum.IsNull())
			GetLog() << "ERROR: cannot find drum-1 from its name in the C::E system! ! \n";
		else
		{
			mrigidBodyDrum->GetCollisionModel()->SetFamily(3);
			mrigidBodyDrum->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
			mrigidBodyDrum->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);
			mrigidBodyDrum->SetFriction(0.1f); 
			mrigidBodyDrum->SetImpactC(0.75f);
			//mrigidBodyDrum->SetRollingFriction(0.2f);
			//mrigidBodyDrum->SetSpinningFriction(0.2f);
		}
		
		//***Ida

		ChSharedPtr<ChBodyAuxRef> mrigidBodySplitter1 = mphysicalSystem.Search("Splitter-10");  
		if (mrigidBodySplitter1.IsNull())
			GetLog() << "ERROR: cannot find Splitter-10 from its name in the C::E system! ! \n";
		else
		{
			mrigidBodySplitter1->SetBodyFixed(true);
			mrigidBodySplitter1->GetCollisionModel()->SetFamily(3); // rivedere 
			mrigidBodySplitter1->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);// rivedere
			mrigidBodySplitter1->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);// rivedere
			mrigidBodySplitter1->SetFriction(0.9f); 
		}

		ChSharedPtr<ChBodyAuxRef> mrigidBodySplitter2 = mphysicalSystem.Search("Splitter2-1");  
		if (mrigidBodySplitter2.IsNull())
			GetLog() << "ERROR: cannot find Splitter2-1 from its name in the C::E system! ! \n";
		else
		{
			mrigidBodySplitter2->SetBodyFixed(true);
			mrigidBodySplitter2->GetCollisionModel()->SetFamily(3);// rivedere
			mrigidBodySplitter2->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);// rivedere
			mrigidBodySplitter2->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);// rivedere
			mrigidBodySplitter2->SetFriction(0.9f); 
		}

		ChSharedPtr<ChBodyAuxRef> mrigidBodySpazzola = mphysicalSystem.Search("Spazzola-1");  
		if (mrigidBodySpazzola.IsNull())
			GetLog() << "ERROR: cannot find Spazzola-1 from its name in the C::E system! ! \n";
		else
		{
			mrigidBodySpazzola->GetCollisionModel()->SetFamily(1); // rivedere
			mrigidBodySpazzola->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);// rivedere
			mrigidBodySpazzola->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);// rivedere
			mrigidBodySpazzola->SetFriction(0.9f);
					
		}

		ChSharedPtr<ChBodyAuxRef> mrigidBodyConveyor = mphysicalSystem.Search("conveyor-1");  
		if (mrigidBodyConveyor.IsNull())
			GetLog() << "ERROR: cannot find conveyor from its name in the C::E system! ! \n";
		else
		{
			mrigidBodyConveyor->GetCollisionModel()->SetFamily(2);
			mrigidBodyConveyor->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
			mrigidBodyConveyor->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);
			mrigidBodyConveyor->SetFriction(0.9f);
			mrigidBodyConveyor->SetImpactC(0.75f);
		}


		//***Ida
		

		//
		// Create a truss (absolute fixed reference body, for connecting the rotating cyl.)
		//

		ChSharedPtr<ChBody> mtruss(new ChBody);
		mtruss->SetBodyFixed(true);

		// Finally, do not forget to add the body to the system:
		application.GetSystem()->Add(mtruss);
	    
		//**Ida

		ChSharedPtr<ChBody> mtruss2(new ChBody);
		mtruss2->SetBodyFixed(true);

		// Finally, do not forget to add the body to the system:
		application.GetSystem()->Add(mtruss2);

		

		//
		// Create a collision shape for the rotating drum (end of the belt)
		//

	//	ChSharedPtr<ChBody> mrigidBodyDrum;
		double drumradius = drumdiameter * 0.5;
		//ChCoordsys<> drum_axis_coordsys(ChCoordsys<>(ChVector<>(conveyor_length/2, -drumradius-conv_thick/2,0)));

		

		// 
		// Create a motor constraint between the cylinder and the truss
		//

		ChSharedPtr<ChLinkEngine> mengine;

		if (!mrigidBodyDrum.IsNull())
		{
			mengine = ChSharedPtr<ChLinkEngine>(new ChLinkEngine);
			ChSharedPtr<ChBody> mdrum(mrigidBodyDrum);
			mengine->Initialize(mdrum, mtruss, drum_csys);

			mengine->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
			if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(mengine->Get_spe_funct()))
				mfun->Set_yconst(-STATIC_speed/(drumdiameter*0.5));

			// Finally, do not forget to add the body to the system:
			application.GetSystem()->Add(mengine);
		}

		//***Ida

		ChSharedPtr<ChLinkEngine> mengine2;

		if (!mrigidBodySpazzola.IsNull())
		{
			mengine2 = ChSharedPtr<ChLinkEngine>(new ChLinkEngine);
			ChSharedPtr<ChBody> mSpazzola(mrigidBodySpazzola);
			mengine2->Initialize(mSpazzola, mtruss2, Spazzola_csys);

			mengine2->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
			if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(mengine2->Get_spe_funct()))
				mfun->Set_yconst(-STATIC_speed/(drumdiameter*0.5));

			// Finally, do not forget to add the body to the system:
			application.GetSystem()->Add(mengine2);


		}
	    

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
			pov_exporter.SetLight(ChVector<>(1.5f,4.4f,-1.0f), ChColor(0.1f,0.1f,0.1f), false);

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
		
		application.GetSystem()->SetIntegrationType(ChSystem::INT_ANITESCU);
		application.GetSystem()->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD); // or ChSystem::LCP_ITERATIVE_BARZILAIBORWEIN for max precision
			// important! dt is small, and particles are small, so it's better to keep this small...
		application.GetSystem()->SetMaxPenetrationRecoverySpeed(0.15);// not needed in INT_TASORA, only for INT_ANITESCU
		application.GetSystem()->SetMinBounceSpeed(0.01);

		application.GetSystem()->Set_G_acc(ChVector<>(0, -9.81, 0));

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
		
		application.GetSystem()->ShowHierarchy(GetLog());

		while(application.GetDevice()->run()) 
		{
			application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

			application.DrawAll();

			application.DoStep();

			if (!application.GetPaused())
			{ 
				
				totframes++;

				apply_forces (	&mphysicalSystem,		// contains all bodies
								drum_csys,		 // pos and rotation of axis of drum (not rotating reference!)
								drumspeed,		 // speed of drum
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

			

				if (receiver.checkbox_plotforces->isChecked())
					draw_forces ( application , 1000);

				if (receiver.checkbox_plottrajectories->isChecked())
					DrawTrajectories(application);

				
				// Continuosly create debris that fall on the conveyor belt
				
				if (debris_number <= max_numb_particles)
				{
				create_debris(	application.GetTimestep(), 
								STATIC_flow, 
								nozzle_csys,
								*application.GetSystem(), 
								&application, 
								&pov_exporter);

				debris_number = debris_number + particles_dt;
				}

				// Limit the max age (in seconds) of debris particles on the scene, 
				// deleting the oldest ones, for performance
				purge_debris (*application.GetSystem(),6);

				// Maybe the user played with the slider and changed STATIC_speed...
				if (!mengine.IsNull())
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
									//double my_cond  = electricproperties->conductivity ;
									ChVector<> my_ElectricForce = electricproperties->ElectricForce;
									ChVector<> my_ElectricImageForce = electricproperties->ElectricImageForce;
									ChVector<> my_StokesForce = electricproperties->StokesForce;
									double rad = ((abody->GetMass())*3)/((abody->GetDensity())*4*CH_C_PI);
									
									// Save on disk some infos...
									file_for_output << abody->GetIdentifier()<< ", "
													<< abody->GetPos().x << ", "
													<< abody->GetPos().y << ", "
													<< abody->GetPos().z << ", "
													<< abody->GetDensity() << ", "
													//<< my_cond << ", "
													<< abody->GetMass()<< ", "
													<< pow(rad,1.0/3) << "\n";
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

		return 0;
	}



}; // end of class



#endif