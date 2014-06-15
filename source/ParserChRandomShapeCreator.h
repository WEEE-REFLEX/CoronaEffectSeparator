#ifndef PARSERCHRANDOMSHAPECREATOR_H
#define PARSERCHRANDOMSHAPECREATOR_H

#include "ChParticleEmitter.h"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"
#include <fstream>

#include "ParserChDistribution.h"

using namespace particlefactory;


/// Class for parsing data of ChRandomShapeGenerator
class ParserChRandomShapeGenerator 
{ 
public:
	
		/// Just a simple static function here, call as ParserChRandomShapeGenerator::Parse(...)
	static ChSharedPtr<ChRandomShapeCreator> ParseCreate (rapidjson::Value& mval)
	{
		if (!mval.IsObject()) {throw (ChException( "Invalid shape creator object."));}

		ChSharedPtr<ChRandomShapeCreator> createdcreator;

		char* token;
		
		token = "type";
		if (mval.HasMember(token)) 
		{
			if (!mval[token].IsString()) {throw (ChException( "Invalid string after '"+std::string(token)+"'"));}

			std::string mtype = mval[token].GetString();

			if (mtype == std::string("ChRandomShapeGeneratorSpheres") )
			{
				GetLog() << "Parsing a ChRandomShapeCreatorSpheres generator!\n";

				// Create a sphere generator!
				ChSharedPtr<ChRandomShapeCreatorSpheres> mcreator(new ChRandomShapeCreatorSpheres);
				createdcreator = mcreator;

				token = "diameter_distribution";
				if (mval.HasMember(token)) {
					if (!mval.IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetDiameterDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "density_distribution";
				if (mval.HasMember(token)) {
					if (!mval.IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetDensityDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
			}

			if (mtype == std::string("ChRandomShapeCreatorBoxes") )
			{
				GetLog() << "Parsing a ChRandomShapeCreatorBoxes generator!\n";

				// Create a boxes generator!
				ChSharedPtr<ChRandomShapeCreatorBoxes> mcreator(new ChRandomShapeCreatorBoxes);
				createdcreator = mcreator;

				token = "Xsize_distribution";
				if (mval.HasMember(token)) {
					if (!mval.IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetXsizeDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "Ysize_distribution";
				if (mval.HasMember(token)) {
					if (!mval.IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetYsizeDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "Zsize_distribution";
				if (mval.HasMember(token)) {
					if (!mval.IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetZsizeDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
				token = "density_distribution";
				if (mval.HasMember(token)) {
					if (!mval.IsObject()) {throw (ChException( "Invalid distribution after '"+std::string(token)+"'"));}
					mcreator->SetDensityDistribution( ParserChDistribution::ParseCreate(mval[token]) );
				}
			}

			if (mtype == std::string("ChRandomShapeCreatorFromFamilies") )
			{
				GetLog() << "Parsing a ChRandomShapeCreatorFromFamilies generator!\n";

				// Create a family generator!
				ChSharedPtr<ChRandomShapeCreatorFromFamilies> mcreator(new ChRandomShapeCreatorFromFamilies);
				createdcreator = mcreator;

				token = "families";
				if (mval.HasMember(token)) {
					if (!mval.IsArray()) {throw (ChException( "Invalid array after '"+std::string(token)+"'"));}
					for (rapidjson::SizeType i = 0; i < mval.Size(); i++)
					{
						double mprob=0;

						token = "probability";
						if (mval[i].HasMember(token)) {
							if (!mval[i][token].IsNumber()) {throw (ChException( "Invalid number (range from 0 to 1) after '"+std::string(token)+"'"));}
							mprob = mval[i][token].GetDouble();
						} else 
							{throw (ChException( "Missing 'probability' field in a family of ChRandomShapeCreatorFromFamilies"));}
						
						// Create the family using the parser, and add it to the generator using the providen probability
						mcreator->AddFamily(ParserChRandomShapeGenerator::ParseCreate(mval[i]), mprob);
					}
				}
			}


			if(createdcreator)
			{
				token = "add_collision_shape";
				if (mval.HasMember(token)) {
					if (!mval[token].IsBool()) {throw (ChException( "Invalid true/false flag after '"+std::string(token)+"'"));}
					createdcreator->SetAddCollisionShape( mval[token].GetBool() );
				}
				token = "add_visualization_asset";
				if (mval.HasMember(token)) {
					if (!mval[token].IsBool()) {throw (ChException( "Invalid true/false flag after '"+std::string(token)+"'"));}
					createdcreator->SetAddVisualizationAsset( mval[token].GetBool() );
				}
			}

		}
		if (createdcreator.IsNull()) {throw (ChException( "Cannot create the specified particle creator (check after 'type': .. )"));}

		return createdcreator;
	}
	
};


#endif

