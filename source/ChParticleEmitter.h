//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: A.Tasora

#ifndef CHPARTICLEEMITTER_H
#define CHPARTICLEEMITTER_H


#include "core/ChMathematics.h"
#include "core/ChVector.h"
#include "core/ChMatrix.h"
#include "core/ChDistribution.h"
#include "core/ChSmartpointers.h"
#include "geometry/ChCSphere.h"
#include "geometry/ChCBox.h"
#include "geometry/ChCCylinder.h"
#include "physics/CHsystem.h"
#include "physics/ChBodyEasy.h"



	/// BASE class for generators of random ChBody shapes
class ChRandomShapeCreator : public ChShared
{
public:
	ChRandomShapeCreator() {}

			/// Function that creates a random ChBody particle each
			/// time it is called. 
			/// Note: it must be implemented by children classes!
	virtual ChSharedPtr<ChBody> RandomGenerate(ChCoordsys<> mcoords) = 0;
};



	/// Class for generating spheres with variable radius
	/// and density. By default uses constant distributions 
	/// (all spheres with default radius 0.01) but you can provide your distributions.
class ChRandomShapeCreatorSpheres : public ChRandomShapeCreator
{
public:
	ChRandomShapeCreatorSpheres() 
	{
		// defaults
		radius  = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(0.01));
		density = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1000));
	}

			/// Function that creates a random ChBody particle each
			/// time it is called.
	virtual ChSharedPtr<ChBody> RandomGenerate(ChCoordsys<> mcoords) 
	{
		ChSharedPtr<ChBodyEasySphere> mbody(new ChBodyEasySphere(radius->GetRandom(), density->GetRandom(),true));
		mbody->SetCoord(mcoords);
		return mbody;
	};

			/// Set the statistical distribution for the random radius.
	void SetRadiusDistribution(ChSmartPtr<ChDistribution> mdistr) {radius = mdistr;}

			/// Set the statistical distribution for the random density.
	void SetDensityDistribution(ChSmartPtr<ChDistribution> mdistr) {density = mdistr;}

private:
	ChSmartPtr<ChDistribution> radius;
	ChSmartPtr<ChDistribution> density;
};


	/// Class for generating boxes with variable sizes
	/// and density. By default uses constant distributions 
	/// (all boxes with default fixed sizes) but you can provide your distributions.
class ChRandomShapeCreatorBoxes : public ChRandomShapeCreator
{
public:
	ChRandomShapeCreatorBoxes() 
	{
		// defaults
		x_size  = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(0.01));
		y_size  = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(0.01));
		z_size  = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(0.01));
		density = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1000));
	}

			/// Function that creates a random ChBody particle each
			/// time it is called.
	virtual ChSharedPtr<ChBody> RandomGenerate(ChCoordsys<> mcoords) 
	{
		ChSharedPtr<ChBodyEasyBox> mbody(new ChBodyEasyBox(x_size->GetRandom(), y_size->GetRandom(), z_size->GetRandom(), density->GetRandom(), true));
		mbody->SetCoord(mcoords);
		return mbody;
	};

			/// Set the statistical distribution for the x size.
	void SetXsizeDistribution(ChSmartPtr<ChDistribution> mdistr) {x_size = mdistr;}
			/// Set the statistical distribution for the y size.
	void SetYsizeDistribution(ChSmartPtr<ChDistribution> mdistr) {y_size = mdistr;}
			/// Set the statistical distribution for the z size.
	void SetZsizeDistribution(ChSmartPtr<ChDistribution> mdistr) {z_size = mdistr;}

			/// Set the statistical distribution for the random density.
	void SetDensityDistribution(ChSmartPtr<ChDistribution> mdistr) {density = mdistr;}

private:
	ChSmartPtr<ChDistribution> x_size;
	ChSmartPtr<ChDistribution> y_size;
	ChSmartPtr<ChDistribution> z_size;
	ChSmartPtr<ChDistribution> density;
};

	/// Class for generating cylinders with variable radii
	/// and length. By default uses constant distributions 
	/// (all cylinders are equal) but you can provide your distributions.
class ChRandomShapeCreatorCylinders : public ChRandomShapeCreator
{
public:
	ChRandomShapeCreatorCylinders() 
	{
		// defaults
		radius         = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(0.01));
		lenght_factor  = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(2.0));
		density = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1000));
	}

			/// Function that creates a random ChBody particle each
			/// time it is called.
	virtual ChSharedPtr<ChBody> RandomGenerate(ChCoordsys<> mcoords) 
	{
		double rad    = radius->GetRandom();
		double height = lenght_factor->GetRandom() * rad;
		ChSharedPtr<ChBodyEasyCylinder> mbody(new ChBodyEasyCylinder(rad,height, density->GetRandom(), true));
		mbody->SetCoord(mcoords);
		return mbody;
	};

			/// Set the statistical distribution for the radius.
	void SetRadiusDistribution(ChSmartPtr<ChDistribution> mdistr) {radius = mdistr;}
			/// Set the statistical distribution for the lenght ratio (lenght = radius*length_factor).
	void SetLenghtFactorDistribution(ChSmartPtr<ChDistribution> mdistr) {lenght_factor = mdistr;}

			/// Set the statistical distribution for the random density.
	void SetDensityDistribution(ChSmartPtr<ChDistribution> mdistr) {density = mdistr;}

private:
	ChSmartPtr<ChDistribution> radius;
	ChSmartPtr<ChDistribution> lenght_factor;
	ChSmartPtr<ChDistribution> density;
};

/*
	/// Class for generating spheres from different families, 
	/// each with given probability
class ChRandomShapeCreatorFromSamples : public ChRandomShapeCreator
{
public:
	ChRandomShapeCreatorFromSamples() 
		{
			// defaults
			Reset();
		}

			/// Function that creates a random ChBody particle each
			/// time it is called.
	virtual ChSharedPtr<ChBody> RandomGenerate(ChCoordsys<> mcoords) 
		{
			if (particle_samples.size() ==0) 
				throw ChException("Error, cannot randomize particles from a zero length vector of samples");

			ChSharedPtr<ChBody> sample = particle_samples[0];
			double rand = ::chrono::ChRandom();
			for (int i=0; i <  cumulative_probability.size(); ++i)
			{
				if ( rand < cumulative_probability[i] )
				{
					sample = particle_samples[i];
					break;
				}
			}
			ChSharedPtr<ChBody> newbody(new ChBody);
			newbody->Copy(sample.get_ptr());

			newbody->SetCoord(mcoords);
			return sample;
		};
	
			/// Call this BEFORE adding a set of samples via AddSample()
	void Reset() 
		{
			probability.clear();
			cumulative_probability.clear();
			sum = 0;
			particle_samples.clear();
		}
			/// Call this multiple times to add a set of samples.
			/// Each sample is a body with a given probability. 
			/// Finally, use Setup() after you used	AddSample N times	
			/// The sum of probabilities should be 1; otherwise will be normalized.
	void AddSample(ChSharedPtr<ChBody> msample, double mprobability)
		{
			probability.push_back(mprobability);
			sum += mprobability;
			cumulative_probability.push_back(sum);
			particle_samples.push_back(msample);
		}
			/// Call this when you finished adding samples via AddSample()
	void Setup()
		{
			if (probability.size() ==0) 
				return;
			// normalize if integral of atomic probabilities not unitary
			double scale = 1.0/sum;
			for (int i= 0; i< probability.size(); ++i)
			{
				probability[i] *= scale;
				cumulative_probability[i]*= scale;
			}
		}
private:
	std::vector<double> probability;
	std::vector<double> cumulative_probability;
	double sum;
	std::vector< ChSharedPtr<ChBody> > particle_samples;
};
*/




/////////////////////////////////////////////////////////////////////////////////////////////////


	/// BASE class for generators of random particle alignment.
	/// By default it simply always returns quaternion {1,0,0,0} (no rotation) so it
	/// it is up to sub-classes to implement more sophisticated randomizations.
class ChRandomParticleAlignment : public ChShared
{
public:
	ChRandomParticleAlignment() {}

			/// Function that creates a random alignment (as a rotation quaternion) each
			/// time it is called. 
			/// Note: it must be implemented by children classes!
	virtual ChQuaternion<> RandomAlignment() {return QUNIT;}
};

	/// Class for generator of random particle alignment.
	/// The S3 space is not uniformily sampled - this is a quick approximation anyway.
class ChRandomParticleAlignmentUniform : public ChRandomParticleAlignment
{
public:
	ChRandomParticleAlignmentUniform() {}

			/// Function that creates a random alignment (as a rotation quaternion) each
			/// time it is called. The alignment is a random polar rotation.
	virtual ChQuaternion<> RandomAlignment()
		{
			ChQuaternion<> mq(	1.-ChRandom()*2., 
								1.-ChRandom()*2., 
								1.-ChRandom()*2.,
								1.-ChRandom()*2.); 
			return mq.GetNormalized();
		}
};


/////////////////////////////////////////////////////////////////////////////////////////////////


	/// BASE class for generators of random particle positions.
	/// By default it simply always returns Pxyz={0,0,0}, so it
	/// it is up to sub-classes to implement more sophisticated randomizations.
class ChRandomParticlePosition : public ChShared
{
public:
	ChRandomParticlePosition() {}

			/// Function that creates a random position each
			/// time it is called. 
			/// Note: it must be implemented by children classes!
	virtual ChVector<> RandomPosition() {return VNULL;}
};

	/// Class for generator of random particle positions 
	/// scattered over a rectangle outlet in 3D space	
class ChRandomParticlePositionRectangleOutlet : public ChRandomParticlePosition
{
public:	
	ChRandomParticlePositionRectangleOutlet() 
	{
		// defaults
		outlet = CSYSNORM;
		width  = 0.1;
		height = 0.1;
	}

			/// Function that creates a random position each
			/// time it is called. 
	virtual ChVector<> RandomPosition() 
		{
			ChVector<> localp = ChVector<>(ChRandom()*width,ChRandom()*height, 0);
			return outlet.TrasformLocalToParent(localp);
		}
			/// Access the coordinate system of the rectangular outlet. 
			/// The outlet width is on the X direction of this csys, and the
			/// outled height is on the Y direction of this csys.
	ChCoordsys<>& Outlet() {return outlet;}
		
			/// Access the width of the rectangular outlet, that is on the X axis of the coordinate
	double& OutletWidth() {return width;}

			/// Access the height of the rectangular outlet, that is on the Y axis of the coordinate
	double& OutletHeight() {return height;}

private:
	ChCoordsys<> outlet;
	double width;
	double height;
};



	/// Class for generator of random particle positions 
	/// scattered over a parametric surface
class ChRandomParticlePositionOnGeometry : public ChRandomParticlePosition
{
public:
	ChRandomParticlePositionOnGeometry() 
		{
			// defaults
			geometry = ChSmartPtr<ChGeometry>(new ChBox(VNULL, ChMatrix33<>(QUNIT), ChVector<>(0.1,0.1,0.1)));
		}

			/// Function that creates a random position each
			/// time it is called.
	virtual ChVector<> RandomPosition() 
		{ 
			ChVector<> mpos;
			geometry->Evaluate(mpos, ChRandom(), ChRandom());
			return mpos; 
		}

			/// Set the parametric surface used for this outlet. 
			/// The surface will be sampled uniformly over its U,V parametric
			/// coordinaters. In cas of lines, oly U is used, in case of parametric volumes, U,V,W.
	void SetGeometry(ChSmartPtr<ChGeometry> mgeometry) {this->geometry = mgeometry;}

private:
	ChSmartPtr<ChGeometry> geometry;
};



/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////


	
	/// Class for emitters of particles, with random positions,
	/// rotations, and random shapes. You can setup a variety of 
	/// different emitters by assembling different types of 
	/// items inherited by classes like ChRandomShapeCreator,
	/// ChRandomParticlePosition, etc.
class ChParticleEmitter
{
public:
	ChParticleEmitter() 
		{
			// defaults:
			particles_per_second = 100;
			particle_creator     = ChSharedPtr<ChRandomShapeCreatorSpheres>(new ChRandomShapeCreatorSpheres);
			particle_positioner  = ChSharedPtr<ChRandomParticlePositionRectangleOutlet>(new ChRandomParticlePositionRectangleOutlet);
			particle_aligner     = ChSharedPtr<ChRandomParticleAlignmentUniform> (new ChRandomParticleAlignmentUniform);
			creation_callback	 = 0;
		}

			/// Function that creates random particles with random shape, position
			/// and alignment each time it is called. 
			/// Typically, one calls this function once per timestep.
	void EmitParticles(ChSystem& msystem, double dt)
		{
			// get n.of particles to generate in this dt timestep, with floor roundoff
			int particles_per_step = (int)floor(dt*particles_per_second);
			// since int->double roundoff, adjust to have correct flow rate on large n. of steps
			if ((dt*particles_per_second - floor(dt*particles_per_second)) > ChRandom())
				particles_per_step++;

			// create the particles for this timestep
			for (int i = 0; i < particles_per_step; ++i)
			{
				ChCoordsys<> mcoords;
				mcoords.pos = particle_positioner->RandomPosition();
				mcoords.rot = particle_aligner->RandomAlignment();

				ChSharedPtr<ChBody> mbody = particle_creator->RandomGenerate(mcoords);

				if (this->creation_callback)
					this->creation_callback->PostCreation(mbody, mcoords);

				msystem.Add(mbody);
			}
		}

			/// Inherit form this class and pass an object to the PostCreation() function
			/// to have the callback executed per each created particle.
	class ChPostCreationCallback
	{
	public:
			/// Implement this function if you want to provide the post creation callback.
		virtual void PostCreation(ChSharedPtr<ChBody> mbody, ChCoordsys<> mcoords) = 0;
	};
			/// Pass an object from a ChPostCreationCallback-inherited class if you want to 
			/// set additional stuff on each created particle (ex.set some random asset, set some random material, or such)
	void SetPostCreationCallback(ChPostCreationCallback* mcallback) {this->creation_callback = mcallback;}

			/// Set the particle creator, that is an object whose class is
			/// inherited from ChRandomShapeCreator
	void SetParticleCreator   (ChSharedPtr<ChRandomShapeCreator> mc) {particle_creator = mc;}
		
			/// Set the particle positioner, that generates different positions for each particle
	void SetParticlePositioner(ChSharedPtr<ChRandomParticlePosition> mc) {particle_positioner = mc;}

			/// Set the particle aligner, that generates different rotations for each particle
	void SetParticleAligner   (ChSharedPtr<ChRandomParticleAlignment> mc) {particle_aligner = mc;}

			/// Access the flow rate, measured as n.of particles per second.
	double& ParticlesPerSecond() {return particles_per_second;}

private:
	double particles_per_second;
	ChSharedPtr<ChRandomShapeCreator>	   particle_creator;
	ChSharedPtr<ChRandomParticlePosition>  particle_positioner;
	ChSharedPtr<ChRandomParticleAlignment> particle_aligner;
	ChPostCreationCallback* creation_callback;
};


#endif  // END of ChMath.h 
