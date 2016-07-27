#ifndef PROCESSFLOW_H
#define PROCESSFLOW_H

#include "ElectricParticleAssets.h"
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
    virtual ~ProcessFlow() { }

    explicit ProcessFlow(int u_sects = 100, int v_sects = 100)
	{
		mmass_plastic.Reset(u_sects, v_sects);
		mmass_metal.Reset(u_sects, v_sects);
	}

		/// Remove the particle from the system. 
    void ParticleProcessEvent(std::shared_ptr<ChBody> mbody, 
									  ChSystem& msystem, 
									  std::shared_ptr<ChParticleEventTrigger> mprocessor ) override
	{
		if (auto mrectangleprocessor = std::dynamic_pointer_cast<ChParticleEventFlowInRectangle>(mprocessor))
		{
			// compute the row and colum of the matrix
			int irow = (int)floor(mmass_plastic.GetRows() * mrectangleprocessor->last_intersectionUV.x);
			if (irow >= mmass_plastic.GetRows()) irow = mmass_plastic.GetRows()-1;
			int icol = (int)floor(mmass_plastic.GetColumns() * mrectangleprocessor->last_intersectionUV.y);
			if (icol >= mmass_plastic.GetColumns()) icol = mmass_plastic.GetColumns()-1;

			// Fetch the ElectricParticleAsset asset from the list
			for (size_t na= 0; na< mbody->GetAssets().size(); na++)
			{
				std::shared_ptr<ChAsset> myasset = mbody->GetAssetN(na);
				if (auto electricproperties = std::dynamic_pointer_cast<ElectricParticleAsset>(myasset))
				{
					// ok, its a particle!

					if (electricproperties->GetMaterial() == ElectricParticleAsset::material_type::e_mat_plastic)
					{
						this->mmass_plastic(irow,icol) += mbody->GetMass();
					}
					if (electricproperties->GetMaterial() == ElectricParticleAsset::material_type::e_mat_metal)
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
