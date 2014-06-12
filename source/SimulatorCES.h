#ifndef SIMULATORCES_H
#define SIMULATORCES_H

///
/// HEADER for the simulator of the Corona Electrostatic Separator
/// for the WEEE project
/// Authors I. Critelli, A. Tasora, 2014
///



#include "physics/CHapidll.h" 
#include "physics/CHsystem.h"
#include "physics/ChBodyEasy.h"
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

#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"

#include "ElectricParticleProperty.h"
#include "UserInterfaceEventReceiver.h"
#include "ChParticleEmitter.h"
#include "ElectricForcesCES.h"

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

	ElectricForcesCES ces_forces; // this contains data for computing the CES electric forces

	ChParticleEmitter emitter;
	ChSharedPtr<ChRandomParticlePositionRectangleOutlet> emitter_positions;
	ChSharedPtr<ChRandomParticleAlignment> emitter_rotations;

	double STATIC_flow;

	double drumspeed_rpm; // [rpm]
	double drumspeed_radss; //[rad/s]
	
	double sphrad;
	double sphrad2;
	double sphrad3;
	double particles_dt;
	double debris_number;
	double max_numb_particles;

	// material surfaces
	float surface_drum_friction;
	float surface_drum_rolling_friction;
	float surface_drum_spinning_friction;
	float surface_drum_restitution;
	float surface_plate_friction;
	float surface_plate_rolling_friction;
	float surface_plate_spinning_friction;
	float surface_plate_restitution;
	float surface_particles_friction;
	float surface_particles_rolling_friction;
	float surface_particles_spinning_friction;
	float surface_particles_restitution;


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
	
	double xnozzlesize; 
	double znozzlesize; 
	
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
	std::string solidworks_py_modelfile;
	double timestep;

		///
		/// Create the SimulatorCES
		/// and initialize member data
		/// 
	SimulatorCES()
	{
			// initialize member data:

			// initialize the randomizer for positions
		emitter_positions = ChSharedPtr<ChRandomParticlePositionRectangleOutlet>(new ChRandomParticlePositionRectangleOutlet);
		emitter_positions->OutletWidth() = 0.1;    // default x outlet size, from CAD;
		emitter_positions->OutletHeight() = 0.182; // default y outlet size, from CAD;
		emitter.SetParticlePositioner(emitter_positions);

			// initialize the randomizer for alignments
		emitter_rotations = ChSharedPtr<ChRandomParticleAlignmentUniform>(new ChRandomParticleAlignmentUniform);
		emitter.SetParticleAligner(emitter_rotations);
		
			// initialize the randomizer for creations, with statistical distributions
		/*
		//***TEST***
		ChSharedPtr<ChRandomShapeCreatorBoxes> mcreator1(new ChRandomShapeCreatorBoxes);
		mcreator1->SetXsizeDistribution( ChSmartPtr<ChWeibullDistribution>(new ChWeibullDistribution(0.002, 1.0)) );
		mcreator1->SetYsizeDistribution( ChSmartPtr<ChWeibullDistribution>(new ChWeibullDistribution(0.002, 1.0)) );
		mcreator1->SetZsizeDistribution( ChSmartPtr<ChWeibullDistribution>(new ChWeibullDistribution(0.002, 1.0)) );

		ChSharedPtr<ChRandomShapeCreatorCylinders> mcreator3(new ChRandomShapeCreatorCylinders);
		mcreator3->SetRadiusDistribution( ChSmartPtr<ChMinMaxDistribution>(new ChMinMaxDistribution(0.002, 0.0002)) );
		mcreator3->SetLenghtFactorDistribution( ChSmartPtr<ChZhangDistribution>(new ChZhangDistribution(4, 4/2.25)) );

		ChSharedPtr<ChRandomShapeCreatorConvexHulls> mcreator4(new ChRandomShapeCreatorConvexHulls);
		mcreator4->SetChordDistribution( ChSmartPtr<ChZhangDistribution>(new ChZhangDistribution(0.01, 0.003)) );
		mcreator4->SetSizeRatioYZDistribution( ChSmartPtr<ChMinMaxDistribution>(new ChMinMaxDistribution(1.0, 0.3)) );
		mcreator4->SetNpoints(14);

		ChSharedPtr<ChRandomShapeCreatorFromFamilies> mcreatorTot(new ChRandomShapeCreatorFromFamilies);
		mcreatorTot->AddFamily(mcreator2, 0.3);
		mcreatorTot->AddFamily(mcreator3, 0.7);
		mcreatorTot->Setup();
		*/

		ChSharedPtr<ChRandomShapeCreatorSpheres> mcreator2(new ChRandomShapeCreatorSpheres);
		mcreator2->SetRadiusDistribution( ChSmartPtr<ChZhangDistribution>(new ChZhangDistribution(0.002, 0.002/3.25)) );

		class MyCreatorFamily1 : public ChCallbackPostCreation
		{
			public: virtual void PostCreation(ChSharedPtr<ChBody> mbody, ChCoordsys<> mcoords)
			{
				ChSharedPtr<ChTexture> mtexture(new ChTexture);
				mtexture->SetTextureFilename("../objects/pinkwhite.png");
				mbody->AddAsset(mtexture);
			}
		};
		MyCreatorFamily1* callback_family1 = new MyCreatorFamily1;

		mcreator2->SetCallbackPostCreation(callback_family1);

		emitter.SetParticleCreator(mcreator2);
		


		solidworks_py_modelfile = "../CAD_conveyor/conveyor_Ida"; // note! do not add ".py" after the filename

		STATIC_flow =0; //1000;

		drumspeed_rpm = 44.8;
		drumspeed_radss = drumspeed_rpm*((2.0*CH_C_PI)/60.0); //[rad/s]

		sphrad = 0.38e-3;
		sphrad2 = 0.25e-3;
		sphrad3 = 0.794e-3;
		particles_dt;
		debris_number = 0;
		max_numb_particles = 100;

		surface_drum_friction =0.5f;
		surface_drum_rolling_friction =0;
		surface_drum_spinning_friction =0;
		surface_drum_restitution =0;
		surface_plate_friction =0.2f;
		surface_plate_rolling_friction =0;
		surface_plate_spinning_friction =0;
		surface_plate_restitution =0;
		surface_particles_friction =0.2f;
		surface_particles_rolling_friction =0;
		surface_particles_spinning_friction =0;
		surface_particles_restitution =0;

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
		znozzlesize = 0.182; //**from CAD, nozzle width
		xnozzlesize = 0.1; //**from CAD, nozzle width

		densityMetal =  8400; // sn-pb //8900;//rame//1820 vecchia densità;
		densityPlastic = 946;// polipropilene //900 vecchia densità;
		myid = 1;

		// Init coordinate systems with position and rotation of important items in the 
		// simulator. These are initializad with constant values, but if loading the
		// SolidWorks model, they will be changed accordingly to what is found in the CAD 
		// file (see later, where the SolidWorks model is parsed). 
		/*
		//***ALEX disabled because initialized by SolidWroks file, anyway
		double conv_thick = 0.01; 
		double conveyor_length = 0.6;
		conveyor_csys	= ChCoordsys<>( ChVector<>(0, -conv_thick, 0) ) ; // default position
		drum_csys		= ChCoordsys<>( ChVector<>(conveyor_length/2, -(drumdiameter*0.5)-conv_thick/2,0) );  // default position
		nozzle_csys		= ChCoordsys<>( ChVector<>(0, 0.01, 0) ); // default position
		Splitter1_csys	= ChCoordsys<>( ChVector<>(conveyor_length/2+0.2, -(drumdiameter*0.5)-conv_thick/2,0) );  // default position
		Splitter2_csys	= ChCoordsys<>( ChVector<>(conveyor_length/2+0.4, -(drumdiameter*0.5)-conv_thick/2,0) );  // default position
		Spazzola_csys	= ChCoordsys<>( ChVector<>(conveyor_length/2-0.10, -(drumdiameter*0.5)-conv_thick/2,0) );  // default position
		*/

		// set as true for saving log files each n frames
		save_dataset = false;
		save_irrlicht_screenshots = false;
		save_POV_screenshots = false;
		saveEachNframes = 3;

		irr_cast_shadows = true;

		totframes = 0;
			
		init_particle_speed = true;

		particle_magnification = 3; // for larger visualization of particle

		timestep = 0.001;

		// Set small collision envelopes for objects that will be created from now on..
		ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);  //0.002
		ChCollisionModel::SetDefaultSuggestedMargin  (0.0005); //0.0008
	}


			///
			///Parser
			/// - load settings from a .ces input file, with simulator settings in JSON format
	bool ParseSettings(const char* filename)
	{
		try 
		{
			// Prepare  input stream and copy to char* buffer
			ChStreamInAsciiFile settingfile(filename);
			std::stringstream buffer;
			buffer << settingfile.GetFstream().rdbuf();
			std::string mstr = buffer.str();
			const char* stringbuffer = mstr.c_str();

			rapidjson::Document document;
			document.Parse<0>( stringbuffer );
			if (document.HasParseError())
			{
				std::string errstrA( (const char*)(&stringbuffer[ChMax(document.GetErrorOffset()-10,0)]) ); errstrA.resize(10);
				std::string errstrB( (const char*)(&stringbuffer[document.GetErrorOffset()]) ); errstrB.resize(20);
				throw (ChException("the file has bad JSON syntax," + std::string(document.GetParseError()) + " \n\n[...]" + errstrA + " <--- " + errstrB + "[...]\n" ));
			}
			if (!document.IsObject())
				throw (ChException("the file is not a valid JSON document"));

			char* token;

			token = "solidworks_exported_model";
			if (document.HasMember(token)) {
				if (!document[token].IsString()) {throw (ChException( "Invalid filename string after '"+std::string(token)+"'"));}
				this->solidworks_py_modelfile = document[token].GetString();
			}
			token = "drum_rpm";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				this->drumspeed_rpm   = document[token].GetDouble();
				this->drumspeed_radss = drumspeed_rpm*((2.0*CH_C_PI)/60.0); //[rad/s]
			}
			token = "save_each_Nsteps";
			if (document.HasMember(token)) {
				if (!document[token].IsInt()) {throw (ChException( "Invalid integer number after '"+std::string(token)+"'"));}
				this->saveEachNframes = document[token].GetInt();
			}
			token = "save_dataset";
			if (document.HasMember(token)) {
				if (!document[token].IsBool()) {throw (ChException( "Invalid true/false flag after '"+std::string(token)+"'"));}
				this->save_dataset = document[token].GetBool();
			}
			token = "save_irrlicht_screenshots";
			if (document.HasMember(token)) {
				if (!document[token].IsBool()) {throw (ChException( "Invalid true/false flag after '"+std::string(token)+"'"));}
				this->save_irrlicht_screenshots = document[token].GetBool();
			}
			token = "save_POV_screenshots";
			if (document.HasMember(token)) {
				if (!document[token].IsBool()) {throw (ChException( "Invalid true/false flag after '"+std::string(token)+"'"));}
				this->save_POV_screenshots = document[token].GetBool();
			}
			token = "timestep";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				this->timestep = document[token].GetDouble();
			}
			token = "surface_drum_friction";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				this->surface_drum_friction = (float)document[token].GetDouble();
			}
			token = "surface_drum_rolling_friction";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				this->surface_drum_rolling_friction = (float)document[token].GetDouble();
			}
			token = "surface_drum_spinning_friction";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				this->surface_drum_spinning_friction = (float)document[token].GetDouble();
			}
			token = "surface_drum_restitution";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				this->surface_drum_restitution = (float)document[token].GetDouble();
			}
			token = "surface_plate_friction";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				this->surface_plate_friction = (float)document[token].GetDouble();
			}
			token = "surface_plate_rolling_friction";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				this->surface_plate_rolling_friction = (float)document[token].GetDouble();
			}
			token = "surface_plate_spinning_friction";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				this->surface_plate_spinning_friction = (float)document[token].GetDouble();
			}
			token = "surface_plate_restitution";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				this->surface_plate_restitution = (float)document[token].GetDouble();
			}
			token = "surface_particles_friction";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				this->surface_particles_friction = (float)document[token].GetDouble();
			}
			token = "surface_particles_rolling_friction";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				this->surface_particles_rolling_friction = (float)document[token].GetDouble();
			}
			token = "surface_particles_spinning_friction";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				this->surface_particles_spinning_friction = (float)document[token].GetDouble();
			}
			token = "surface_particles_restitution";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				this->surface_particles_restitution = (float)document[token].GetDouble();
			}
			token = "default_collision_envelope";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				ChCollisionModel::SetDefaultSuggestedEnvelope(document[token].GetDouble());
			}
			token = "default_collision_margin";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				ChCollisionModel::SetDefaultSuggestedMargin(document[token].GetDouble());
			}
			token = "CES_forces";
			if (document.HasMember(token)) {
				if (!document[token].IsObject()) {throw (ChException( "Invalid object after '"+std::string(token)+"'"));}
				rapidjson::Value& mval = document[token];
				token = "U";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					this->ces_forces.U = mval[token].GetDouble();
				}
				token = "L";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					this->ces_forces.L = mval[token].GetDouble();
				}
				token = "alpha_deg";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					this->ces_forces.alpha = (CH_C_PI/180)*mval[token].GetDouble();
				}
				token = "drum_diameter";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					this->ces_forces.drumdiameter = (CH_C_PI/180)*mval[token].GetDouble();
				}
				token = "drum_width";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					this->ces_forces.drum_width = (CH_C_PI/180)*mval[token].GetDouble();
				}
				token = "electrode_diameter";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					this->ces_forces.electrodediameter = (CH_C_PI/180)*mval[token].GetDouble();
				}
			}
			token = "emitter";
			if (document.HasMember(token)) {
				if (!document[token].IsObject()) {throw (ChException( "Invalid object after '"+std::string(token)+"'"));}
				rapidjson::Value& mval = document[token];
				token = "outlet_height";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					(ChSharedPtr<ChRandomParticlePositionRectangleOutlet>(emitter_positions))->OutletHeight() = mval[token].GetDouble();
				}
				token = "outlet_width";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					(ChSharedPtr<ChRandomParticlePositionRectangleOutlet>(emitter_positions))->OutletWidth() = mval[token].GetDouble();
				}
				token = "particles_per_second";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					this->emitter.ParticlesPerSecond() = mval[token].GetDouble();
				}
				token = "use_particle_reservoir";
				if (mval.HasMember(token)) {
					if (!mval[token].IsBool()) {throw (ChException( "Invalid true/false flag after '"+std::string(token)+"'"));}
					this->emitter.SetUseParticleReservoir( mval[token].GetBool() );
				}
				token = "particle_reservoir";
				if (mval.HasMember(token)) {
					if (!mval[token].IsInt()) {throw (ChException( "Invalid integer after '"+std::string(token)+"'"));}
					this->emitter.ParticleReservoirAmount() = mval[token].GetInt();
				}
				
			}
		}
		catch (ChException me)
		{
			GetLog()<< "ERROR loading settings file: \n   " << filename << "\n Reason: " << me.what() << "\n\n"; 
		}

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
				mrigidBody->SetFriction(surface_particles_friction);
				mrigidBody->SetImpactC(surface_particles_restitution);
				mrigidBody->SetIdentifier(myid); // NB fatto solo per le sfere!!!!!!!!!
				
				       
				mrigidBody->SetRollingFriction(surface_particles_rolling_friction);
				mrigidBody->SetSpinningFriction(surface_particles_spinning_friction);


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
				mrigidBody->SetFriction(surface_particles_friction);
				mrigidBody->SetImpactC(surface_particles_restitution); 
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
				mrigidBody->SetFriction(surface_particles_friction);
				mrigidBody->SetImpactC(surface_particles_restitution); 
				mrigidBody->SetIdentifier(myid); // NB fatto solo per le sfere!!!!!!!!!
				      
				// mrigidBody->SetRollingFriction(surface_particles_rolling_friction);
				// mrigidBody->SetSpinningFriction(surface_particles_spinning_friction);


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
				mrigidBody->SetFriction(surface_particles_friction);
				mrigidBody->SetImpactC(surface_particles_restitution); 

				// Define a collision shape 
				mrigidBody->GetCollisionModel()->ClearModel();
				mrigidBody->GetCollisionModel()->AddBox(sphrad*2*xscale, sphrad*2*yscale, sphrad*2*yscale);
				// oppure aggiungi molte sfere che 'approssimano' la sagoma, es per cilindro:
				//for (int i=0; i<5; ++i)
				//	mrigidBody->GetCollisionModel()->AddSphere(sphrad, &ChVector<>(i*0.001,0,0));
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
				mrigidBody->SetFriction(surface_particles_friction);
				mrigidBody->SetImpactC(surface_particles_restitution); 

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
					
					created_body->SetDensity((float)densityPlastic);
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
			
					created_body->SetDensity((float)densityMetal);
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
				//created_body->SetPos_dt(ChVector<>(drumspeed*(drumdiameter/2.0), 0,0));
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
		//ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);  //0.002
		//ChCollisionModel::SetDefaultSuggestedMargin  (0.0005); //0.0008


	
		// IMPORT A SOLIDWORK MODEL 

		// 1) create the Python engine. This is necessary in order to parse the files that 
		// have been saved using the SolidWorks add-in for Chrono::Engine.

		ChPythonEngine my_python;

		// 2) loads the .py file (as saved from SolidWorks) and fill the system.
		try
		{
			my_python.ImportSolidWorksSystem(this->solidworks_py_modelfile.c_str(), mphysicalSystem);  // note, don't use the .py suffix in filename..
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

		emitter_positions->Outlet() = nozzle_csys;
		emitter_positions->Outlet().rot.Q_from_AngAxis(CH_C_PI_2, VECT_X); // rotate outlet 90° on x

		
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
			mrigidBodyDrum->SetFriction(surface_drum_friction); 
			mrigidBodyDrum->SetImpactC(surface_drum_restitution);
			mrigidBodyDrum->SetRollingFriction(surface_drum_rolling_friction);
			mrigidBodyDrum->SetSpinningFriction(surface_drum_spinning_friction);
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
			mrigidBodySplitter1->SetFriction(0.1f); 
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
			mrigidBodySplitter2->SetFriction(0.1f); 
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
			mrigidBodyConveyor->SetFriction(surface_plate_friction);
			mrigidBodyConveyor->SetImpactC(surface_plate_restitution);
			mrigidBodyConveyor->SetRollingFriction(surface_plate_rolling_friction);
			mrigidBodyConveyor->SetSpinningFriction(surface_plate_spinning_friction);
		}


		

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
				mfun->Set_yconst(-drumspeed_radss);  // angular speed in [rad/s]

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
				mfun->Set_yconst(-drumspeed_radss); // angular speed in [rad/s]

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

		class MyCreatorForAll : public ChCallbackPostCreation
		{
			public: virtual void PostCreation(ChSharedPtr<ChBody> mbody, ChCoordsys<> mcoords)
			{
				irrlicht_application->AssetBind(mbody);
				irrlicht_application->AssetUpdate(mbody);
			}
			irr::ChIrrApp* irrlicht_application;
		};
		MyCreatorForAll* mcreation_callback = new MyCreatorForAll;
		mcreation_callback->irrlicht_application = &application;
		emitter.SetCallbackPostCreation(mcreation_callback);


		application.AssetBindAll();
		application.AssetUpdateAll();
		if (irr_cast_shadows)
			application.AddShadowAll();


		// 
		// THE SOFT-REAL-TIME CYCLE
		//
		
		application.SetStepManage(true);
		application.SetTimestep(this->timestep);
		
		application.GetSystem()->SetIntegrationType(ChSystem::INT_ANITESCU);
		application.GetSystem()->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD);// LCP_ITERATIVE_SOR_MULTITHREAD or ChSystem::LCP_ITERATIVE_BARZILAIBORWEIN for max precision
			// important! dt is small, and particles are small, so it's better to keep this small...
		application.GetSystem()->SetMaxPenetrationRecoverySpeed(0.15);// not needed in INT_TASORA, only for INT_ANITESCU
		application.GetSystem()->SetMinBounceSpeed(0.1);

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

				// Apply the forces caused by electrodes of the CES machine:

				ces_forces.apply_forces (	&mphysicalSystem,		// contains all bodies
								drum_csys,		 // pos and rotation of axis of drum (not rotating reference!)
								drumspeed_radss, // speed of drum
								totframes);

			

				if (receiver.checkbox_plotforces->isChecked())
					draw_forces ( application , 1000);

				if (receiver.checkbox_plottrajectories->isChecked())
					DrawTrajectories(application);

				
				// Continuosly create debris that fall on the conveyor belt
				
				// **ALEX*** new approach!
				this->emitter.EmitParticles(mphysicalSystem, application.GetTimestep()); //***TEST***

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

				// Maybe the user played with the slider and changed the speed of drum...
				if (!mengine.IsNull())
				  if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(mengine->Get_spe_funct()))
					mfun->Set_yconst(-drumspeed_radss);  // angular speed in [rad/s]

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