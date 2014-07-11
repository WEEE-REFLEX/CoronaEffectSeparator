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
#include "collision/ChCCollisionSystemBullet.h"
#include "particlefactory/ChParticleEmitter.h"

#include <fstream>
#include "unit_PYTHON/ChPython.h"
#include "unit_POSTPROCESS/ChPovRay.h"

#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"

#include "ElectricParticleProperty.h"
#include "UserInterfaceEventReceiver.h"
#include "ElectricForcesCES.h"
#include "ParserEmitter.h"
#include "ParserElectricForcesCES.h"

// Use the namespace of Chrono

using namespace chrono;
using namespace postprocess;
using namespace particlefactory;

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

	// The ChronoENGINE physical system
	ChSystem mphysicalSystem;

	ElectricForcesCES ces_forces; // this contains data for computing the CES electric forces

	ChParticleEmitter emitter;
	ChSharedPtr<ChRandomParticlePositionRectangleOutlet> emitter_positions;
	ChSharedPtr<ChRandomParticleAlignment> emitter_rotations;

	double drumspeed_rpm; // [rpm]
	double drumspeed_radss; //[rad/s]
	

	// material surfaces
	float surface_drum_friction;
	float surface_drum_rolling_friction;
	float surface_drum_spinning_friction;
	float surface_drum_restitution;
	float surface_plate_friction;
	float surface_plate_rolling_friction;
	float surface_plate_spinning_friction;
	float surface_plate_restitution;
	ChSharedPtr<ChMaterialSurface> surface_particles;

	double max_particle_age;

	double xnozzlesize; 
	double znozzlesize; 
	

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
	std::string results_file;
	double timestep;
	double Tmax;


		///
		/// Create the SimulatorCES
		/// and initialize member data
		/// 
	SimulatorCES()
	{
		// Initialize member data:

		solidworks_py_modelfile = "../CAD_conveyor/conveyor_Ida"; // note! do not add ".py" after the filename

		results_file = "output/results.dat";

		drumspeed_rpm = 44.8;
		drumspeed_radss = drumspeed_rpm*((2.0*CH_C_PI)/60.0); //[rad/s]

		//sphrad = 0.38e-3;
		//sphrad2 = 0.25e-3;
		//sphrad3 = 0.794e-3;

		surface_drum_friction =0.5f;
		surface_drum_rolling_friction =0;
		surface_drum_spinning_friction =0;
		surface_drum_restitution =0;
		surface_plate_friction =0.2f;
		surface_plate_rolling_friction =0;
		surface_plate_spinning_friction =0;
		surface_plate_restitution =0;
	
		surface_particles = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
		surface_particles->SetFriction(0.2f);
		surface_particles->SetRollingFriction(0);
		surface_particles->SetSpinningFriction(0);
		surface_particles->SetRestitution(0);

		max_particle_age = 2;

		// nozzle sizes
		znozzlesize = 0.182; //**from CAD, nozzle z width
		xnozzlesize = 0.1; //**from CAD, nozzle x width


		// Init coordinate systems with position and rotation of important items in the 
		// simulator. These are initializad with constant values, but if loading the
		// SolidWorks model, they will be changed accordingly to what is found in the CAD 
		// file (see later, where the SolidWorks model is parsed). 
		/*
		//***ALEX disabled because initialized by SolidWorks file, anyway
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
		// Set contact breaking/merging tolerance of Bullet:
		ChCollisionSystemBullet::SetContactBreakingThreshold(0.001);

		// Important! dt is small, and particles are small, so it's better to keep this small...
		mphysicalSystem.SetMaxPenetrationRecoverySpeed(0.15);// not needed in INT_TASORA, only for INT_ANITESCU
		mphysicalSystem.SetMinBounceSpeed(0.1);


		// In the following there is a default initialization of the 
		// particle creation system, based on ChParticleEmitter. 
		// This is a default configuration, that is _overridden_ if you 
		// call ParseSettings() and load a settings.ces file that contain different
		// configurations for the emitter.

		// ---Initialize the randomizer for positions
		emitter_positions = ChSharedPtr<ChRandomParticlePositionRectangleOutlet>(new ChRandomParticlePositionRectangleOutlet);
		emitter_positions->OutletWidth() = 0.1;    // default x outlet size, from CAD;
		emitter_positions->OutletHeight() = 0.182; // default y outlet size, from CAD;
		emitter.SetParticlePositioner(emitter_positions);

		// ---Initialize the randomizer for alignments
		emitter_rotations = ChSharedPtr<ChRandomParticleAlignmentUniform>(new ChRandomParticleAlignmentUniform);
		emitter.SetParticleAligner(emitter_rotations);
		
		// ---Initialize the randomizer for creations, with statistical distribution

		 // Create a ChRandomShapeCreator object (ex. here for metal particles)
		ChSharedPtr<ChRandomShapeCreatorSpheres> mcreator_metal(new ChRandomShapeCreatorSpheres);
		mcreator_metal->SetDiameterDistribution( ChSmartPtr<ChMinMaxDistribution>(new ::ChMinMaxDistribution(0.002, 0.003)) );

		 // Optional: define a callback to be exectuted at each creation of a metal particle:
		class MyCreator_metal : public ChCallbackPostCreation
		{
			// Here do custom stuff on the just-created particle:
			public: virtual void PostCreation(ChSharedPtr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator)
			{
				  // Attach some optional visualization stuff
				//ChSharedPtr<ChTexture> mtexture(new ChTexture);
				//mtexture->SetTextureFilename("../objects/pinkwhite.png");
				//mbody->AddAsset(mtexture);
				ChSharedPtr<ChColorAsset> mvisual(new ChColorAsset);
				mvisual->SetColor(ChColor(0.9f,0.4f,0.2f));
				mbody->AddAsset(mvisual);
				  // Attach a custom asset. It will hold electrical properties
				ChSharedPtr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty); 
				electric_asset->fraction	  = ElectricParticleProperty::e_fraction_sphere;
				electric_asset->material_type = ElectricParticleProperty::e_mat_metal;
				electric_asset->conductivity  = 6428000;
				electric_asset->birthdate	  = this->systemreference->GetChTime();
				ChVector<> Cradii; // use equivalent-inertia ellipsoid to get characteristic size:
				ChVector<> Ine = mbody->GetInertiaXX();
				Cradii.x = sqrt((5./(2.*mbody->GetMass()))*(Ine.y+Ine.z-Ine.x));
				Cradii.y = sqrt((5./(2.*mbody->GetMass()))*(Ine.x+Ine.z-Ine.y));
				Cradii.z = sqrt((5./(2.*mbody->GetMass()))*(Ine.x+Ine.y-Ine.z));
				electric_asset->Cdim         = Cradii*2.;
				mbody->AddAsset(electric_asset);
			}
			// here put custom data that might be needed by the callback:
			ChSystem* systemreference;
		};
		MyCreator_metal* callback_metal = new MyCreator_metal;
		callback_metal->systemreference = &this->mphysicalSystem;
		mcreator_metal->SetCallbackPostCreation(callback_metal);


		 // Create a ChRandomShapeCreator object (ex. here for metal particles)
		ChSharedPtr<ChRandomShapeCreatorSpheres> mcreator_plastic(new ChRandomShapeCreatorSpheres);
		mcreator_plastic->SetDiameterDistribution( ChSmartPtr<ChMinMaxDistribution>(new ::ChMinMaxDistribution(0.002, 0.002)) );

		 // Optional: define a callback to be exectuted at each creation of a plastic particle:
		class MyCreator_plastic : public ChCallbackPostCreation
		{
			// Here do custom stuff on the just-created particle:
			public: virtual void PostCreation(ChSharedPtr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator)
			{
				  // Attach some optional visualization stuff
				//ChSharedPtr<ChTexture> mtexture(new ChTexture);
				//mtexture->SetTextureFilename("../objects/bluwhite.png");
				//mbody->AddAsset(mtexture);
				ChSharedPtr<ChColorAsset> mvisual(new ChColorAsset);
				mvisual->SetColor(ChColor(0.3f,0.6f,0.7f));
				mbody->AddAsset(mvisual);
				  // Attach a custom asset. It will hold electrical properties
				ChSharedPtr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty); 
				electric_asset->fraction	  = ElectricParticleProperty::e_fraction_sphere; 
				electric_asset->material_type = ElectricParticleProperty::e_mat_plastic;
				electric_asset->conductivity  = 0;
				electric_asset->birthdate	  = this->systemreference->GetChTime();
				ChVector<> Cradii;  // use equivalent-inertia ellipsoid to get characteristic size:
				ChVector<> Ine = mbody->GetInertiaXX();
				Cradii.x = sqrt((5./(2.*mbody->GetMass()))*(Ine.y+Ine.z-Ine.x));
				Cradii.y = sqrt((5./(2.*mbody->GetMass()))*(Ine.x+Ine.z-Ine.y));
				Cradii.z = sqrt((5./(2.*mbody->GetMass()))*(Ine.x+Ine.y-Ine.z));
				electric_asset->Cdim         = Cradii*2.; 
				mbody->AddAsset(electric_asset);
			}
			// here put custom data of the callback
			ChSystem* systemreference;
		};
		MyCreator_plastic* callback_plastic = new MyCreator_plastic;
		callback_plastic->systemreference = &this->mphysicalSystem;
		mcreator_plastic->SetCallbackPostCreation(callback_plastic);


		 // Create a parent ChRandomShapeCreator that 'mixes' the two generators above,
		 // mixing them with a given percentual:
		ChSharedPtr<ChRandomShapeCreatorFromFamilies> mcreatorTot(new ChRandomShapeCreatorFromFamilies);
		mcreatorTot->AddFamily(mcreator_metal,   0.4);	// 1st creator family, with percentual
		mcreatorTot->AddFamily(mcreator_plastic, 0.4);	// 2nd creator family, with percentual
		mcreatorTot->Setup();

		 // Finally, tell to the emitter that it must use the 'mixer' above:
		emitter.SetParticleCreator(mcreatorTot);


		// ---Initialize the randomizer for velocities, with statistical distribution

		ChSharedPtr<ChRandomParticleVelocityConstantDirection> mvelo(new ChRandomParticleVelocityConstantDirection);
		mvelo->SetDirection(-VECT_Y);
		mvelo->SetModulusDistribution(0.0);
 
		emitter.SetParticleVelocity(mvelo);
	}



			///
			/// Parser
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
			token = "results_file";
			if (document.HasMember(token)) {
				if (!document[token].IsString()) {throw (ChException( "Invalid filename string after '"+std::string(token)+"'"));}
				this->results_file = document[token].GetString();
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
				//this->surface_particles_friction = (float)document[token].GetDouble();
				this->surface_particles->SetFriction( (float)document[token].GetDouble() );
			}
			token = "surface_particles_rolling_friction";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				//this->surface_particles_rolling_friction = (float)document[token].GetDouble();
				this->surface_particles->SetRollingFriction( (float)document[token].GetDouble() );
			}
			token = "surface_particles_spinning_friction";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				//this->surface_particles_spinning_friction = (float)document[token].GetDouble();
				this->surface_particles->SetSpinningFriction( (float)document[token].GetDouble() );
			}
			token = "surface_particles_restitution";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				//this->surface_particles_restitution = (float)document[token].GetDouble();
				this->surface_particles->SetRestitution( (float)document[token].GetDouble() );
			}
			token = "max_particle_age";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				this->max_particle_age =  (float)document[token].GetDouble();
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
			token = "default_contact_breaking";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				ChCollisionSystemBullet::SetContactBreakingThreshold(document[token].GetDouble());
			}
			token = "max_penetration_recovery_speed";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				mphysicalSystem.SetMaxPenetrationRecoverySpeed( document[token].GetDouble() );
			}
			token = "min_bounce_speed";
			if (document.HasMember(token)) {
				if (!document[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
				mphysicalSystem.SetMinBounceSpeed( document[token].GetDouble() );
			}
			token = "CES_forces";
			if (document.HasMember(token)) 
			{
				// Parse the settings of the emitter, emitter positioner etc.
				ParserElectricForcesCES::Parse(this->ces_forces, document[token]);
			}
			token = "emitter";
			if (document.HasMember(token)) 
			{
				// Parse the settings of the emitter, emitter positioner etc.
				ParserEmitter::Parse(this->emitter, this->mphysicalSystem, this->emitter_positions, this->emitter_rotations, document[token]);
			}
		}
		catch (ChException me)
		{
			GetLog()<< "ERROR loading settings file: \n   " << filename << "\n Reason: " << me.what() << "\n\n"; 
			throw(ChException("Error in parsing."));
		}

		return true;
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
					ChSharedPtr<ElectricParticleProperty> electricproperties = myasset.DynamicCastTo<ElectricParticleProperty>();
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
					electricproperties = myasset.DynamicCastTo<ElectricParticleProperty>();
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
					trajectoryasset = myasset.DynamicCastTo<ParticleTrajectory>();
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
					trajectoryasset = myasset.DynamicCastTo<ParticleTrajectory>();
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
		ChSharedPtr<ChBodyAuxRef> mrigidBodyDrum = mphysicalSystem.Search("drum-1").DynamicCastTo<ChBodyAuxRef>();  
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

		ChSharedPtr<ChBodyAuxRef> mrigidBodySplitter1 = mphysicalSystem.Search("Splitter-10").DynamicCastTo<ChBodyAuxRef>();  
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

		ChSharedPtr<ChBodyAuxRef> mrigidBodySplitter2 = mphysicalSystem.Search("Splitter2-1").DynamicCastTo<ChBodyAuxRef>();  
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

		ChSharedPtr<ChBodyAuxRef> mrigidBodySpazzola = mphysicalSystem.Search("Spazzola-1").DynamicCastTo<ChBodyAuxRef>();  
		if (mrigidBodySpazzola.IsNull())
			GetLog() << "ERROR: cannot find Spazzola-1 from its name in the C::E system! ! \n";
		else
		{
			mrigidBodySpazzola->GetCollisionModel()->SetFamily(1); // rivedere
			mrigidBodySpazzola->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);// rivedere
			mrigidBodySpazzola->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);// rivedere
			mrigidBodySpazzola->SetFriction(0.9f);
					
		}

		ChSharedPtr<ChBodyAuxRef> mrigidBodyConveyor = mphysicalSystem.Search("conveyor-1").DynamicCastTo<ChBodyAuxRef>();  
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
			pov_exporter.SetLight(ChVector<>(0.5f,0.75f,0.0f), ChColor(0.1f,0.1f,0.1f), true);

			pov_exporter.SetCamera(ChVector<>(0.5f,0.75f,0.5f),ChVector<>(0.2f,0.6f,0.f),30,false);

					// optional: use SetCustomPOVcommandsScript() to add further POV commands,
					// ex. create an additional light, and an additional grid, etc. 
					// Remember the "\" char per each newline.
			
			pov_exporter.SetCustomPOVcommandsScript(" \
				light_source {   \
				  <0.5, 0.8, 0.2>  \
				  color rgb<1.7,1.7,1.7> \
				  area_light <0.4, 0, 0>, <0, 0, 0.4>, 5, 5 \
				  adaptive 1 \
				  jitter\
				} \
				//object{ Grid(0.5,0.01, rgb<0.9,0.9,0.9>, rgbt<1,1,1,1>) rotate <90, 0, 0>  } \
				//object{ Grid(0.1,0.04, rgb<1.5,1.5,1.5>, rgbt<1,1,1,1>) rotate <90, 0, 0> translate 0.001*z} \
			");

					// IMPORTANT! Tell to the POVray exporter that 
					// he must take care of converting the shapes of
					// all items (that have been added so far)!
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
		// What to do by default on ALL newly created particles? 
		// A callback executed at each particle creation can be attached to the emitter..
		// 

		// a- define a class that implement your custom PostCreation method...
		class MyCreatorForAll : public ChCallbackPostCreation
		{
			public: virtual void PostCreation(ChSharedPtr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator)
			{
				// Set the friction properties (using a shared ChSurfaceMaterial
				mbody->SetMaterialSurface( asurface_material ); 

				// Attach an asset to show trajectories
				//ChSharedPtr<ParticleTrajectory> massettraj(new ParticleTrajectory);
				//mbody->AddAsset(massettraj);

				// Enable Irrlicht visualization for all particles
				airrlicht_application->AssetBind(mbody);
				airrlicht_application->AssetUpdate(mbody);

				// Enable POV visualization
				if (apov_exporter)
					apov_exporter->Add(mbody);

				// Disable gyroscopic forces for increased integrator stabilty
				mbody->SetNoGyroTorque(true);
			}
			irr::ChIrrApp* airrlicht_application;
			ChPovRay* apov_exporter;
			ChSharedPtr<ChMaterialSurface> asurface_material;
		};
		// b- create the callback object...
		MyCreatorForAll* mcreation_callback = new MyCreatorForAll;
		// c- set callback own data that he might need...
		mcreation_callback->airrlicht_application = &application;
		mcreation_callback->asurface_material = this->surface_particles;
		if (this->save_POV_screenshots) 
			mcreation_callback->apov_exporter = &pov_exporter;
		else
			mcreation_callback->apov_exporter = 0;
		// d- attach the callback to the emitter!
		emitter.SetCallbackPostCreation(mcreation_callback);





		// 
		// THE SOFT-REAL-TIME CYCLE
		//
		
		application.SetStepManage(true);
		application.SetTimestep(this->timestep);
		
		application.GetSystem()->SetIntegrationType(ChSystem::INT_ANITESCU);
		application.GetSystem()->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD);// LCP_ITERATIVE_SOR_MULTITHREAD or ChSystem::LCP_ITERATIVE_BARZILAIBORWEIN for max precision

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
				this->emitter.EmitParticles(mphysicalSystem, application.GetTimestep()); //***TEST***


				// Limit the max age (in seconds) of debris particles on the scene, 
				// deleting the oldest ones, for performance
				purge_debris (*application.GetSystem(), this->max_particle_age);


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

									
									ChSharedPtr<ElectricParticleProperty> electricproperties = myasset.DynamicCastTo<ElectricParticleProperty>();
									//double my_cond  = electricproperties->conductivity ;
									ChVector<> my_ElectricForce = electricproperties->ElectricForce;
									ChVector<> my_ElectricImageForce = electricproperties->ElectricImageForce;
									ChVector<> my_StokesForce = electricproperties->StokesForce;
									double rad = ((abody->GetMass())*3)/((abody->GetDensity())*4*CH_C_PI);
									int fraction_identifier = (int)electricproperties->fraction; // id will be 0=box, 1=cylinder, 2=sphere, 3=hull, 4=shavings, etc. (see enum)
									int material_identifier = (int)electricproperties->material_type; // id will be 0=plastic, 1=metal, 2=others (see enum)
									// Save on disk some infos...
									file_for_output << fraction_identifier << ", "
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