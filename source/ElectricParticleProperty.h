#ifndef ELECTRICPARTICLEPROPERTY_H
#define ELECTRICPARTICLEPROPERTY_H

using namespace chrono;

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

	enum enum_fraction_type
	{
		e_fraction_box,
		e_fraction_cylinder,
		e_fraction_sphere,
		e_fraction_convexhull,
		e_fraction_shaving,
		e_fraction_others
	} fraction;

	enum enum_material_type
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


#endif

