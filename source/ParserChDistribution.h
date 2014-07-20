#ifndef PARSERCHDISTRIBUTION_H
#define PARSERCHDISTRIBUTION_H

#include "core/ChDistribution.h"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"
#include <fstream>


using namespace chrono;


/// Class for parsing data of ParserChDistribution
class ParserChDistribution
{ 
public:
	
		/// Just a simple static function here, call as ParserChRandomShapeGenerator::Parse(...)
	static ChSmartPtr<ChDistribution> ParseCreate (rapidjson::Value& mval)
	{
		if (!mval.IsObject()) {throw (ChException( "Invalid distribution object."));}

		ChSmartPtr<ChDistribution> createddistr;

		char* token;

		token = "type";
		if (mval.HasMember(token)) 
		{
			if (!mval[token].IsString()) {throw (ChException( "Invalid string after '"+std::string(token)+"'"));}

			std::string mtype = mval[token].GetString();

			if (mtype == std::string("ChConstantDistribution") )
			{
				double value=0;

				token = "value";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					value = mval[token].GetDouble();
				}

				// Create the distribution!
				ChSmartPtr<ChConstantDistribution> mdistr(new ChConstantDistribution(value));
				createddistr = mdistr;
			}

			if (mtype == std::string("ChMinMaxDistribution") )
			{
				double mmin=0;
				double mmax=1;

				token = "min";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					mmin = mval[token].GetDouble();
				}
				token = "max";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					mmax = mval[token].GetDouble();
				}

				// Create the distribution!
				ChSmartPtr<ChMinMaxDistribution> mdistr(new ChMinMaxDistribution(mmin,mmax));
				createddistr = mdistr;
			}

			if (mtype == std::string("ChNormalDistribution") )
			{
				double variance=1;
				double mean=0;

				token = "variance";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					variance = mval[token].GetDouble();
				}
				token = "mean";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					mean = mval[token].GetDouble();
				}

				// Create the distribution!
				ChSmartPtr<ChNormalDistribution> mdistr(new ChNormalDistribution(variance,mean));
				createddistr = mdistr;
			}


			if (mtype == std::string("ChWeibullDistribution") )
			{
				double lambda=1;
				double k=1;

				token = "lambda";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					lambda = mval[token].GetDouble();
				}
				token = "k";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					k = mval[token].GetDouble();
				}

				// Create the distribution!
				ChSmartPtr<ChWeibullDistribution> mdistr(new ChWeibullDistribution(lambda,k));
				createddistr = mdistr;
			}


			if (mtype == std::string("ChZhangDistribution") )
			{
				double average=1;
				double mmin=1;

				token = "average";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					average = mval[token].GetDouble();
				}
				token = "min";
				if (mval.HasMember(token)) {
					if (!mval[token].IsNumber()) {throw (ChException( "Invalid number after '"+std::string(token)+"'"));}
					mmin = mval[token].GetDouble();
				}

				// Create the distribution!
				ChSmartPtr<ChZhangDistribution> mdistr(new ChZhangDistribution(average,mmin));
				createddistr = mdistr;
			}


			if (mtype == std::string("ChContinuumDistribution") )
			{
				GetLog() << "Parsing a ChContinuumDistribution!\n";

				std::vector< double > mx;
				std::vector< double > my;

				token = "x";
				if (mval.HasMember(token)) {
					if (!mval[token].IsArray()) {throw (ChException( "Invalid x array in ChContinuumDistribution."));}
					for (rapidjson::SizeType i = 0; i < mval[token].Size(); i++)
					{
						if (!mval[token][i].IsNumber()) {throw (ChException( "Invalid number in x array of ChContinuumDistribution."+std::string(token)+"'"));}						
						mx.push_back(mval[token][i].GetDouble());
					}
				}
				token = "y";
				if (mval.HasMember(token)) {
					if (!mval[token].IsArray()) {throw (ChException( "Invalid y array in ChContinuumDistribution."));}
					for (rapidjson::SizeType i = 0; i < mval[token].Size(); i++)
					{
						if (!mval[token][i].IsNumber()) {throw (ChException( "Invalid number in y array of ChContinuumDistribution."+std::string(token)+"'"));}						
						my.push_back(mval[token][i].GetDouble());
					}
				}
				if (mx.size() == 0) {throw (ChException( "Error, empty x array in ChContinuumDistribution"));}
				if (my.size() == 0) {throw (ChException( "Error, empty y array in ChContinuumDistribution"));}
				if (mx.size() != my.size()) {throw (ChException( "Error, x and y arrays must have same length, in ChContinuumDistribution. "));}

				// Create the distribution!
				ChMatrixDynamic<> mmx(1, mx.size());
				ChMatrixDynamic<> mmy(1, my.size());
				for (unsigned int i=0;i<mx.size();++i)
				{
					mmx(0,i)=mx[i];
					mmy(0,i)=my[i];
				}
				ChSmartPtr<ChContinuumDistribution> mdistr(new ChContinuumDistribution(mmx,mmy));
				createddistr = mdistr;
			}

			if (mtype == std::string("ChDiscreteDistribution") )
			{
				GetLog() << "Parsing a ChDiscreteDistribution!\n";

				std::vector< double > mx;
				std::vector< double > my;

				token = "x";
				if (mval.HasMember(token)) {
					if (!mval[token].IsArray()) {throw (ChException( "Invalid x array in ChDiscreteDistribution."));}
					for (rapidjson::SizeType i = 0; i < mval[token].Size(); i++)
					{
						if (!mval[token][i].IsNumber()) {throw (ChException( "Invalid number in x array of ChDiscreteDistribution."));}						
						mx.push_back(mval[token][i].GetDouble());
					}
				}
				token = "y";
				if (mval.HasMember(token)) {
					if (!mval[token].IsArray()) {throw (ChException( "Invalid y array in ChDiscreteDistribution."));}
					for (rapidjson::SizeType i = 0; i < mval[token].Size(); i++)
					{
						if (!mval[token][i].IsNumber()) {throw (ChException( "Invalid number in y array of ChDiscreteDistribution."));}						
						my.push_back(mval[token][i].GetDouble());
					}
				}
				if (mx.size() == 0) {throw (ChException( "Error, empty x array in ChDiscreteDistribution"));}
				if (my.size() == 0) {throw (ChException( "Error, empty y array in ChDiscreteDistribution"));}
				if (mx.size() != my.size()) {throw (ChException( "Error, x and y arrays must have same length, in ChDiscreteDistribution. "));}

				// Create the distribution!
				ChMatrixDynamic<> mmx(1, mx.size());
				ChMatrixDynamic<> mmy(1, my.size());
				for (unsigned int i=0;i<mx.size();++i)
				{
					mmx(0,i)=mx[i];
					mmy(0,i)=my[i];
				}
				ChSmartPtr<ChDiscreteDistribution> mdistr(new ChDiscreteDistribution(mmx,mmy));
				createddistr = mdistr;
			}

			
			// Add parsing code of future distributions here....


		}
		if (createddistr.IsNull()) {throw (ChException( "Cannot create the specified distribution (unknown 'type': ...? )"));}

		return createddistr;
	}
	
};


#endif

