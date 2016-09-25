#ifndef ELECTROSTATICCORONASEPARATOR_H
#define ELECTROSTATICCORONASEPARATOR_H

#include "physics/ChSystem.h"
#include <fstream>
#include "physics/ChBodyEasy.h"
#include "physics/ChConveyor.h"
#include "physics/ChBodyAuxRef.h"
#include "core/ChFileutils.h"
#include "chrono_irrlicht/CHirrApp.h"
#include "core/ChRealtimeStep.h"
#include "core/ChMath.h"
#include "core/ChDistribution.h"
#include "collision/ChCCollisionSystemBullet.h"
#include "particlefactory/ChParticleEmitter.h"
#include "particlefactory/ChParticleProcessor.h"

//#include "chrono_python/ChPython.h"
#include "chrono_postprocess/ChPovRay.h"

//#include "rapidjson/document.h"
//#include "rapidjson/prettywriter.h"
//#include "rapidjson/filereadstream.h"
//#include "rapidjson/filewritestream.h"

#include "UserInterfaceEventReceiver.h"
#include "ProcessFlow.h"
#include "ElectricParticleAssets.h"
//#include "ParserEmitter.h"
//#include "ParserElectricForcesCES.h"
//#include "ProcessFlow.h"


using namespace chrono;
using namespace particlefactory;




/// Utility function that plots a matrix over a rectangle
static void drawDistribution(irr::video::IVideoDriver* driver,
                             const ChMatrix<>& Z, // distribution matrix
                             const ChCoordsys<>& mpos, // center coordinates of the rectangle that measures flow
                             double x_size, double y_size, // size of the rectangle
                             irr::video::SColor mcol = irr::video::SColor(50, 80, 110, 110),
                             bool use_Zbuffer = false
);

template <typename asset_type>
static bool GetAsset(const std::shared_ptr<ChBody>& body_iter, std::shared_ptr<asset_type>& desired_asset);

class ElectrostaticCoronaSeparator 
{ 

private:

    class ParticleBucket
    {
    private:
        ChCoordsys<double> plane_csys {{0,0,0},{1,0,0,0}};
        double x_semi_width = 1;
        double z_semi_width = 1;

        
    public:
        size_t particle_caught = 0;

        ParticleBucket(const ChCoordsys<double> csys, double x_width, double z_width): plane_csys(csys), x_semi_width(x_width/2), z_semi_width(z_width/2) {}
        ~ParticleBucket(){}

        bool IsCaught(const ChBody& body, bool count_catches = true)
        {
            auto local_pos = plane_csys.TransformPointParentToLocal(body.GetCoord().pos);

            if (local_pos(1) < 0 &&
                local_pos(0) < x_semi_width && local_pos(0) > -x_semi_width &&
                local_pos(2) < z_semi_width && local_pos(2) > -z_semi_width
                )
            {
                if (count_catches)
                    ++particle_caught;
                return true;
            }


            return false;
        }

        void CatchIf(ChSystem& mysystem, bool f(ChBody&) )
        {
            for (auto body_sel = 0; body_sel < mysystem.Get_bodylist()->size(); ++body_sel)
            {
                auto body = (*mysystem.Get_bodylist())[body_sel];

                if (IsCaught(*body, true) && f(*body))
                {
                    mysystem.Remove(body);
                }

            }
        }

        void SetBucket(const ChVector<>& plane_center_position, const ChQuaternion<>& plane_orientation = ChQuaternion<>(1,0,0,0))
        {
            plane_csys.pos = plane_center_position;
            plane_csys.rot = plane_orientation;
        }

        void SetWidth(double x_width, double z_width)
        {
            x_semi_width = x_width / 2;
            z_semi_width = z_width / 2;
        }

        ChCoordsys<>& GetBucketCoordsys() { return plane_csys; }

    };


    double h1 = 0;
    double h2 = 0;
    double j = 0;
    double f = 0;

public:

	// data for this type of asset 
	double drum_diameter = 0.320;
	double drum_width = 0.3;
	double electrode_diameter = 0.038;

	// electric features
	double U = -35000; // supplied high-voltage [v]
	double L = 0.267; //certer distance of rotating roll electrode and electrostatic pole *****ida
	double alpha = (CH_C_PI / 180) * 30; //angle of horizontal line and electrodes center line *****ida
	const double epsilon = 8.85941e-12; // dielectric constant [F/m] *****ida 
	const double epsilonO = 8.854187e-12; //vacuum permeability
	const double epsilonR = 2.5; //relative permeability
	const double eta = 0.0000181; // Air drag coefficent [N*s/m^2]
	const double ro = 1.225;  //fluid density (air) [Kg/m^3]


    ChParticleEmitter emitter;
    std::shared_ptr<ChRandomParticlePositionRectangleOutlet> emitter_positions;
    std::shared_ptr<ChRandomParticleAlignment> emitter_rotations;

    //double drumspeed_rpm = 44.8; // [rpm]
    const double drumspeed_rpm_max = 100;
    double drumspeed_rpm = 70.4; // [rpm]
    double drumspeed_rads = drumspeed_rpm*(CH_C_2PI / 60.0); //[rad/s]

    std::shared_ptr<ChFunction_Const> drum_speed_function;

    std::shared_ptr<ChMaterialSurface> surface_particles;

    double max_particle_age = 2;

    double xnozzlesize = 0.182; //**from CAD, nozzle z width
    double znozzlesize = 0.1; //**from CAD, nozzle x width;

    double flowmeter_xmin = 0.28;
    double flowmeter_xmax = flowmeter_xmin + 0.3;
    double flowmeter_width = 0.2;
    double flowmeter_y = -0.1;
    int    flowmeter_bins = 25;

    double particle_flow = 1000; // this is redundant with the one in particle_emitter

    // Coordinate systems with position and rotation of important items in the 
    // separator.

    const double conveyor_thick = 0.01;
    const double conveyor_length = 0.6;

    ChCoordsys<> drum_csys;
    ChCoordsys<> nozzle_csys;

    // set as true for saving log files each n frames
    bool save_dataset = false;
    bool save_irrlicht_screenshots = false;
    bool save_POV_screenshots = false;
    int saveEachNframes = 3;


    int totframes = 0;
    bool init_particle_speed = true;
    double ECSforces_scalefactor = 1000;
    double particle_magnification = 3; // for larger visualization of particle
    std::string results_file = "output/results.txt";
    double Tmax = 5;
    bool splitters_collide = true;

    std::list<ParticleBucket> buckets;

    std::vector<std::shared_ptr<ChBody>> scanned_particles;


    ElectrostaticCoronaSeparator(ChSystem& mphysicalSystem);


    ///
	/// Function that defines the forces on the debris ****ida
	///
    void apply_forces(ChSystem* msystem);

    /// Acquire particles information based on text file given by the spectrophotometric camera.
    bool LoadParticleScan(const char* filename);

    /// Creates random bodies according to the last scan.
    void create_debris_particlescan(double particles_second,
                                    ChSystem& mysystem,
                                    irrlicht::ChIrrApp* irr_application);

    void create_debris_particlescan_original(double dt, double particles_second, ChSystem& mysystem, irrlicht::ChIrrApp* irr_application);

    /// Function that deletes old debris (to avoid infinite creation that fills memory)
    static void purge_debris_byage(ChSystem& mysystem, double max_age = 5.0);
    static void purge_debris_byposition(ChSystem& mysystem, ChVector<> min_position, ChVector<> max_position);
    /// Function for drawing forces
    static void DrawForces(irrlicht::ChIrrApp& application, double scalefactor = 0);

    /// Function to update trajectories. Must be
    /// called at each timestep
    static void UpdateTrajectories(irrlicht::ChIrrApp& application, bool only_those_on_drum = true);

    /// Function to draw trajectories. Must be
    /// called at each timestep
    static void DrawTrajectories(irrlicht::ChIrrApp& application, bool only_those_on_drum = true);

    /// Add bodies belonging to ECS to the \c system and their visual assets to \c application
    int Setup(ChSystem& system, irrlicht::ChIrrApp* application);

    /// Main function of the simulator.
    /// Initialize the simulation, and
    /// Performs the simulation
    /// by running the loop of time integration
    int RunSimulation(irrlicht::ChIrrApp& application);

    static ElectricParticleAsset::material_type getElectricParticleProperty_material_type(int matID);

    void SetDrumSpeed(double speed_rpm)
    {
        drumspeed_rpm = speed_rpm;
        drumspeed_rads = drumspeed_rpm*CH_C_2PI / 60;
    }

    double GetDrumSpeed() const { return drumspeed_rpm; }



};


#endif

