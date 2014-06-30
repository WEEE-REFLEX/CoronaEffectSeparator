#ifndef ELECTRICFORCESCES_H
#define ELECTRICFORCESCES_H

#include "physics/ChSystem.h"
#include "ElectricParticleProperty.h"
#include <fstream>

class ElectricForcesCES 
{ 
public:

	// data for this type of asset 
	double drumdiameter;
	double drum_width;
	double electrodediameter;
	double U; // supplied high-voltage [v]
	double L; //certer distance of rotating roll electrode and electrostatic pole *****ida
	double alpha; //angle of horizontal line and electrodes center line *****ida
	double epsilon; // dielectric constant [F/m] *****ida 
	double epsilonO; //vacuum permeability
	double epsilonR; //relative permeability
	double eta; // Air drag coefficent [N*s/m^2]
	double ro;  //fluid density (air) [Kg/m^3]

private:
	double h1;	//analytical parameter****ida
	double h2;	//analytical parameter****ida
	double j;	//analytical parameter****ida
	double f;	//analytical parameter****ida

public:
		///
		/// Default constructor with initialization of drum force parameters
		///
	ElectricForcesCES()
	{
		drumdiameter = 0.320;		//diameter of drum
		drum_width=0.3;				//used to switch of electric forces out of drum
		electrodediameter = 0.038;	//for force computations
		U = -35000;					//supplied high-voltage [v]
		L = 0.267;					//certer distance of rotating roll electrode and electrostatic pole *****ida
		alpha = (CH_C_PI/180)*30;	//angle of horizontal line and electrodes center line *****ida
		epsilon  = 8.85941e-12;		//dielectric constant [F/m] *****ida 
		epsilonO = 8.854187e-12;	//vacuum permettivity
		epsilonR = 2.5;				//relative permettivity
		eta = 0.0000181;			// Air drag coefficent [N*s/m^2]
		ro=1.225;					// Air density (air) [Kg/m^3]
	}


		///
		/// Function that defines the forces on the debris ****ida
		///
	void apply_forces (	    ChSystem* msystem,		 // contains all bodies
							ChCoordsys<>& drum_csys, // pos and rotation of drum 
							double drumspeed,		 // speed of drum
							int totframes)		
	{
			// Compute parameters on-the-fly (some parameters like L or U might have changed meanwhile..)
		h1 = (pow(L,2)+pow((drumdiameter/2),2)-((electrodediameter/2),2))/(2*L); //analytical parameter****ida
		h2 = (pow(L,2)-pow((drumdiameter/2),2)+((electrodediameter/2),2))/(2*L);//analytical parameter****ida
		j = sqrt(pow(h1,2)-pow((drumdiameter/2),2));//analytical parameter****ida
		f = U/log(((h1+j-(drumdiameter/2))*(h2+j-(electrodediameter/2)))/((drumdiameter/2)+j-h1)*((electrodediameter/2)+j-h2));//analytical parameter****ida

			// Loop on all bodies:
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
				double Emax = -11.818*U-514.87;

	          
				
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
						electricproperties->chargeM = (2./3.)*pow(CH_C_PI,3)*epsilon*pow(average_rad,2)*E;
						electricproperties->chargeM *= (1.0 - 0.3*ChRandom() );
						
						}
					}

					
					ChVector<> ElectricForce = electricproperties->ElectricForce;

					electricproperties->ElectricForce = 0.832 * electricproperties->chargeM * vE;
//GetLog() << "ElectricForce" << ElectricForce << "\n";
					// switch off electric forces if too out-of-plane
					if ((mrelpos.z > drum_width*0.5) || (mrelpos.z < -drum_width*0.5))
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
							electricproperties->chargeP = 3*CH_C_PI*epsilonO*pow(2*average_rad,2)*Emax*(epsilonR/(epsilonR+2)); // charge
							electricproperties->chargeP *= (1.0 - 0.3*ChRandom() );
						}
					} //15000000,750000,450000
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
					if ((mrelpos.z > drum_width*0.5) || (mrelpos.z < -drum_width*0.5))
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

	
};


#endif

