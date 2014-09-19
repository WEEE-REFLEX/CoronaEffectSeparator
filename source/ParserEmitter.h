#ifndef PARSEREMITTER_H
#define PARSEREMITTER_H

#include "particlefactory/ChParticleEmitter.h"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"
#include <fstream>
#include "ParserChRandomShapeCreator.h"

using namespace chrono;
using namespace chrono::particlefactory;


/// Class for parsing data of the CES emitter
class ParserEmitter 
{ 
public:
	
		/// Just a simple static function here, call as ParserEmitter::Parse(...)
	static void Parse (ChParticleEmitter& emitter, 
					   ChSystem& msystem,
					   ChSharedPtr<ChRandomParticlePositionRectangleOutlet> emitter_positions,
					   ChSharedPtr<ChRandomParticleAlignment> emitter_rotations,
					   rapidjson::Value& mval)
	{
		char* token;
		if (!mval.IsObject()) {throw (ChException( "Invalid particle emitter object"));}

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
		token = "flow_control_mode";
		if (mval.HasMember(token)) {
			if (!mval[token].IsString()) {throw (ChException( "Invalid string after '"+std::string(token)+"'"));}
			char buffer [200];
			strncpy(buffer, mval[token].GetString(), mval[token].GetStringLength());
			buffer[mval[token].GetStringLength()]=0;
			if (strcmp(buffer,"FLOW_PARTICLESPERSECOND")==0)
			{
				emitter.SetFlowControlMode(ChParticleEmitter::FLOW_PARTICLESPERSECOND);
			}
			if (strcmp(buffer,"FLOW_MASSPERSECOND")==0)
			{
				emitter.SetFlowControlMode(ChParticleEmitter::FLOW_MASSPERSECOND);
			}
		}
		token = "particles_per_second";
		if (mval.HasMember(token)) {
			if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
			emitter.ParticlesPerSecond() = mval[token].GetDouble();
		}
		token = "mass_per_second";
		if (mval.HasMember(token)) {
			if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
			emitter.MassPerSecond() = mval[token].GetDouble();
		}
		token = "use_particle_reservoir";
		if (mval.HasMember(token)) {
			if (!mval[token].IsBool()) {throw (ChException( "Invalid true/false flag after '"+std::string(token)+"'"));}
			emitter.SetUseParticleReservoir( mval[token].GetBool() );
		}
		token = "use_mass_reservoir";
		if (mval.HasMember(token)) {
			if (!mval[token].IsBool()) {throw (ChException( "Invalid true/false flag after '"+std::string(token)+"'"));}
			emitter.SetUseMassReservoir( mval[token].GetBool() );
		}
		token = "particle_reservoir";
		if (mval.HasMember(token)) {
			if (!mval[token].IsInt()) {throw (ChException( "Invalid integer after '"+std::string(token)+"'"));}
			emitter.ParticleReservoirAmount() = mval[token].GetInt();
		}
		token = "mass_reservoir";
		if (mval.HasMember(token)) {
			if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
			emitter.MassReservoirAmount() = mval[token].GetDouble();
		}
		token = "particle_creator";
		if (mval.HasMember(token)) 
		{
			// Parse the random creator of the emitter
			emitter.SetParticleCreator( ParserChRandomShapeGenerator::ParseCreate(mval[token], msystem ) );
		}
	}
	
};


#endif

