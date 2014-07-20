#ifndef PARSERCHRANDOMSHAPECREATOR_H
#define PARSERCHRANDOMSHAPECREATOR_H

#include "particlefactory/ChParticleEmitter.h"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"
#include <fstream>

#include "ParserChDistribution.h"
#include "assets/ChColorAsset.h"

using namespace chrono;
using namespace chrono::particlefactory;


/// Class for parsing data of ChRandomShapeGenerator
class ParserChRandomShapeGenerator 
{ 
public:
	
		/// Just a simple static function here, call as ParserChRandomShapeGenerator::Parse(...)
	static ChSharedPtr<ChRandomShapeCreator> ParseCreate (rapidjson::Value& mval, ChSystem& msystem)
	{
		if (!mval.IsObject()) {throw (ChException( "Invalid shape creator object."));}

		ChSharedPtr<ChRandomShapeCreator> parsed_creator;

		char* token;
		
		token = "type";
		if (mval.HasMember(token)) 
		{
			if (!mval[token].IsString()) {throw (ChException( "Invalid string after '"+std::string(token)+"'"));}

			std::string mtype = mval[token].GetString();

			if (mtype == std::string("ChRandomShapeCreatorSpheres") )
			{
				// Create a sphere generator!
				ChSharedPtr<ChRandomShapeCreatorSpheres> mcreator(new ChRandomShapeCreatorSpheres);
				parsed_creator = mcreator;

				token = "diameter_distribution";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetDiameterDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "density";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetDensityDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
			}

			if (mtype == std::string("ChRandomShapeCreatorBoxes") )
			{
				// Create a boxes generator!
				ChSharedPtr<ChRandomShapeCreatorBoxes> mcreator(new ChRandomShapeCreatorBoxes);
				parsed_creator = mcreator;

				token = "Xsize_distribution";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetXsizeDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "sizeratioYZ";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetSizeRatioYZDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "sizeratioZ";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetSizeRatioZDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "density";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetDensityDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
			}

			if (mtype == std::string("ChRandomShapeCreatorCylinders") )
			{
				// Create a boxes generator!
				ChSharedPtr<ChRandomShapeCreatorCylinders> mcreator(new ChRandomShapeCreatorCylinders);
				parsed_creator = mcreator;

				token = "diameter";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetDiameterDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "lenght_factor";
				if (mval[token].HasMember(token)) {
					if (!mval.IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetLenghtFactorDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "density";
				if (mval[token].HasMember(token)) {
					if (!mval.IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetDensityDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
			}

			if (mtype == std::string("ChRandomShapeCreatorConvexHulls") )
			{
				// Create a boxes generator!
				ChSharedPtr<ChRandomShapeCreatorConvexHulls> mcreator(new ChRandomShapeCreatorConvexHulls);
				parsed_creator = mcreator;

				token = "npoints";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid integer number after '"+std::string(token)+"'"));}
					mcreator->SetNpoints( mval[token].GetInt()  );
				}
				token = "chord";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetChordDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "sizeratioYZ";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetSizeRatioYZDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "sizeratioZ";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetSizeRatioZDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "density";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetDensityDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
			}

			if (mtype == std::string("ChRandomShapeCreatorShavings") )
			{
				// Create a boxes generator!
				ChSharedPtr<ChRandomShapeCreatorShavings> mcreator(new ChRandomShapeCreatorShavings);
				parsed_creator = mcreator;

				token = "spacing_factor";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					mcreator->SetSpheresSpacingFactor( mval[token].GetDouble() );
				}
				token = "diameter";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetDiameterDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "twistU";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetTwistDistributionU( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "twistV";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetTwistDistributionV( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "lengthratio";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetLengthRatioDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "density";
				if (mval.HasMember(token)) {
					if (!mval[token].IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetDensityDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
			}


			if (mtype == std::string("ChRandomShapeCreatorFromFamilies") )
			{
				// Create a family generator!
				ChSharedPtr<ChRandomShapeCreatorFromFamilies> mcreator(new ChRandomShapeCreatorFromFamilies);
				parsed_creator = mcreator;

				char* famtoken = "families";
				if (mval.HasMember(famtoken)) {
					if (!mval[famtoken].IsArray()) {throw (ChException( "Invalid array after '"+std::string(token)+"'"));}
					for (rapidjson::SizeType i = 0; i < mval[famtoken].Size(); i++)
					{
						double mprob=0;

						token = "probability";
						if (mval[famtoken][i].HasMember(token)) {
							if (!mval[famtoken][i][token].IsNumber()) {throw (ChException( "Invalid number (range from 0 to 1) after '"+std::string(token)+"'"));}
							mprob = mval[famtoken][i][token].GetDouble();
						} else 
							{throw (ChException( "Missing 'probability' field in a family of ChRandomShapeCreatorFromFamilies"));}
						
						// Create the family using the parser, and add it to the generator using the providen probability
						mcreator->AddFamily(ParserChRandomShapeGenerator::ParseCreate(mval[famtoken][i], msystem), mprob);
					}
				}
			}

			// All classes inherited be ChRandomShapeCreator have this in common:
			if(parsed_creator)
			{
				token = "add_collision_shape";
				if (mval.HasMember(token)) {
					if (!mval[token].IsBool()) {throw (ChException( "Invalid true/false flag after '"+std::string(token)+"'"));}
					parsed_creator->SetAddCollisionShape( mval[token].GetBool() );
				}
				token = "add_visualization_asset";
				if (mval.HasMember(token)) {
					if (!mval[token].IsBool()) {throw (ChException( "Invalid true/false flag after '"+std::string(token)+"'"));}
					parsed_creator->SetAddVisualizationAsset( mval[token].GetBool() );
				}

				// Parse custom electric properties etc.
				ElectricParticleProperty::enum_material_type  parsed_mattype = ElectricParticleProperty::e_mat_other;
				double	parsed_conductivity = -1.0;
				ChColor parsed_color = ChColor(-1,0,0);

				token = "material_type";
				if (mval.HasMember(token)) {
					if (!mval[token].IsString()) {throw (ChException( "Invalid string after '"+std::string(token)+"'"));}
					if (mval[token].GetString()==std::string("plastic")) {parsed_mattype = ElectricParticleProperty::e_mat_plastic; parsed_conductivity = 0;}
					if (mval[token].GetString()==std::string("metal"))	 {parsed_mattype = ElectricParticleProperty::e_mat_metal;   parsed_conductivity = 6428000;}
					if (mval[token].GetString()==std::string("other"))	 {parsed_mattype = ElectricParticleProperty::e_mat_other;   parsed_conductivity = 0;}
					if (parsed_conductivity ==-1) {throw (ChException( "Invalid material_type, only \"plastic\"|\"metal\"|\"other\" are supported."));}
				}
				token = "conductivity";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					parsed_conductivity = mval[token].GetDouble();
				}
				token = "color";
				if (mval.HasMember(token)) {
					if (!mval[token].IsArray()) {throw (ChException( "Invalid array after '"+std::string(token)+"'"));}
					if (mval[token].Size() != 3)  {throw (ChException( "Invalid color array size: must have 3 numbers (RGB, in 0..1 range)."));}	
					if (!mval[token][rapidjson::SizeType(0)].IsNumber()) {throw (ChException( "Invalid number for R color componenent (must be in 0..1 range)."));}						
						parsed_color.R = (float)(mval[token][rapidjson::SizeType(0)].GetDouble());
					if (!mval[token][rapidjson::SizeType(1)].IsNumber()) {throw (ChException( "Invalid number for G color componenent (must be in 0..1 range)."));}					
						parsed_color.G = (float)(mval[token][rapidjson::SizeType(1)].GetDouble());
					if (!mval[token][rapidjson::SizeType(2)].IsNumber()) {throw (ChException( "Invalid number for B color componenent (must be in 0..1 range)."));}					
						parsed_color.B = (float)(mval[token][rapidjson::SizeType(2)].GetDouble());
				}

				// Attach custom assets only if the user provided some settings 
				if (parsed_conductivity != -1)
				{
							 // Define a callback to be exectuted at each creation of a metal particle:
					class MyCreator_custom : public ChCallbackPostCreation
					{
						// Here do custom stuff on the just-created particle:
						public: virtual void PostCreation(ChSharedPtr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator)
						{
							  // Attach some optional visualization stuff
							ChSharedPtr<ChColorAsset> mvisual(new ChColorAsset);
							mvisual->SetColor(this->def_color);
							mbody->AddAsset(mvisual);

							  // Attach a custom asset. It will hold electrical properties
							ChSharedPtr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty); 
							
							electric_asset->material_type = this->def_mattype;
							electric_asset->conductivity  = this->def_conductivity;
							electric_asset->birthdate	  = this->systemreference->GetChTime();
							
							ChVector<> Cradii; // use equivalent-inertia ellipsoid to get characteristic size:
							ChVector<> Ine = mbody->GetInertiaXX();
							Cradii.x = sqrt((5./(2.*mbody->GetMass()))*(Ine.y+Ine.z-Ine.x));
							Cradii.y = sqrt((5./(2.*mbody->GetMass()))*(Ine.x+Ine.z-Ine.y));
							Cradii.z = sqrt((5./(2.*mbody->GetMass()))*(Ine.x+Ine.y-Ine.z));
							electric_asset->Cdim         = Cradii*2.;

									// in case the electric_asset is later processed for data output, maybe usefult to initialize it too 
							electric_asset->fraction	  = ElectricParticleProperty::e_fraction_others;
							if (dynamic_cast<ChRandomShapeCreatorSpheres*>(&mcreator))
								electric_asset->fraction	  = ElectricParticleProperty::e_fraction_sphere;
							if (dynamic_cast<ChRandomShapeCreatorBoxes*>(&mcreator))
								electric_asset->fraction	  = ElectricParticleProperty::e_fraction_box;
							if (dynamic_cast<ChRandomShapeCreatorCylinders*>(&mcreator))
								electric_asset->fraction	  = ElectricParticleProperty::e_fraction_cylinder;
							if (dynamic_cast<ChRandomShapeCreatorConvexHulls*>(&mcreator))
								electric_asset->fraction	  = ElectricParticleProperty::e_fraction_convexhull;
							if (dynamic_cast<ChRandomShapeCreatorShavings*>(&mcreator))
								electric_asset->fraction	  = ElectricParticleProperty::e_fraction_shaving;
							
							// Finally add the asset to the newly created particle!
							mbody->AddAsset(electric_asset);
						}

						// here put custom data that might be needed by the callback:
						ChSystem* systemreference;
						ElectricParticleProperty::enum_material_type  def_mattype;
						double	def_conductivity;
						ChColor def_color;
					};

					// Create an object of the above callback class, and add it to the just parsed creator:
					MyCreator_custom* callback_custom = new MyCreator_custom;
					callback_custom->systemreference	= &msystem;
					callback_custom->def_mattype		= parsed_mattype;
					callback_custom->def_conductivity	= parsed_conductivity;
					callback_custom->def_color			= parsed_color;
					parsed_creator->SetCallbackPostCreation(callback_custom);

				} // end of custom properties stuff

			}

		}
		if (parsed_creator.IsNull()) {throw (ChException( "Cannot create the specified particle creator (check after 'type': .. )"));}

		return parsed_creator;
	}
	
};


#endif

