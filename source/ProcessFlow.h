#ifndef PROCESSFLOW_H
#define PROCESSFLOW_H

#include "ElectricParticleProperty.h"
#include "particlefactory/ChParticleProcessEvent.h"


namespace chrono {
namespace particlefactory {




	/// Processed particle will increment 2 NxM matrix mass counters,
	/// one for plastic material and another for metal material in CES system,
	/// so that a statistical distribution of flow over a uv surface
	/// can be obtained. Note that the matrix maps the 0...1 of the u (matrix rows)
	/// and the 0..1 of the v (matrix columns) of the uv surface.
	/// Note: For the moment, this supports only the case of trigger of type
	/// ChParticleEventFlowInRectangle.
class ProcessFlow : public ChParticleProcessEvent
{
public:
	ProcessFlow(int u_sects = 10, int v_sects = 10)
	{
		mmass_plastic.Reset(u_sects, v_sects);
		mmass_metal.Reset(u_sects, v_sects);
	}

		/// Remove the particle from the system. 
	virtual void ParticleProcessEvent(ChSharedPtr<ChBody> mbody, 
									  ChSystem& msystem, 
									  ChSharedPtr<ChParticleEventTrigger> mprocessor ) 
	{
		assert(mprocessor.IsType<ChParticleEventFlowInRectangle>());
		if (mprocessor.IsType<ChParticleEventFlowInRectangle>())
		{
			ChSharedPtr<ChParticleEventFlowInRectangle> mrectangleprocessor = mprocessor.DynamicCastTo<ChParticleEventFlowInRectangle>();
			// compute the row and colum of the matrix

			int irow = floor(mmass_plastic.GetRows() * mrectangleprocessor->last_intersectionUV.x);
			if (irow >= mmass_plastic.GetRows()) irow = mmass_plastic.GetRows()-1;
			int icol = floor(mmass_plastic.GetColumns() * mrectangleprocessor->last_intersectionUV.y);
			if (icol >= mmass_plastic.GetColumns()) icol = mmass_plastic.GetColumns()-1;

			// Fetch the ElectricParticleProperty asset from the list
			for (unsigned int na= 0; na< mbody->GetAssets().size(); na++)
			{
				ChSharedPtr<ChAsset> myasset = mbody->GetAssetN(na);
				if (myasset.IsType<ElectricParticleProperty>())
				{
					// ok, its a particle!
					ChSharedPtr<ElectricParticleProperty> electricproperties = myasset.DynamicCastTo<ElectricParticleProperty>();

					if (electricproperties->material_type == ElectricParticleProperty::e_mat_plastic)
					{
						this->mmass_plastic(irow,icol) += mbody->GetMass();
					}
					if (electricproperties->material_type == ElectricParticleProperty::e_mat_metal)
					{
						this->mmass_metal(irow,icol) += mbody->GetMass();
					}

				}
			}
			
		}
	}

	ChMatrixDynamic<> mmass_plastic;
	ChMatrixDynamic<> mmass_metal;
};





} // end of namespace particlefactory
} // end of namespace chrono


#endif  
