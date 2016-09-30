#include "ElectrostaticCoronaSeparator.h"
#include <typeinfo>
#include <string>
#include "SerialComm.h"

void drawDistribution(irr::video::IVideoDriver* driver,
                      const ChMatrix<>& Z,
                      const ChCoordsys<>& mpos,
                      double x_size, double y_size,
                      irr::video::SColor mcol,
                      bool use_Zbuffer = true)
{
    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = true;
    mattransp.Lighting = false;
    driver->setMaterial(mattransp);

    chrono::ChVector<> V1a(-x_size * 0.5, y_size * 0.5, 0);
    chrono::ChVector<> V2a(x_size * 0.5, y_size * 0.5, 0);
    irrlicht::ChIrrTools::drawSegment(driver, mpos.TransformLocalToParent(V1a), mpos.TransformLocalToParent(V2a), mcol, use_Zbuffer);
    chrono::ChVector<> V1b(-x_size * 0.5, -y_size * 0.5, 0);
    chrono::ChVector<> V2b(x_size * 0.5, -y_size * 0.5, 0);
    irrlicht::ChIrrTools::drawSegment(driver, mpos.TransformLocalToParent(V1b), mpos.TransformLocalToParent(V2b), mcol, use_Zbuffer);
    chrono::ChVector<> V1c(x_size * 0.5, y_size * 0.5, 0);
    chrono::ChVector<> V2c(x_size * 0.5, -y_size * 0.5, 0);
    irrlicht::ChIrrTools::drawSegment(driver, mpos.TransformLocalToParent(V1c), mpos.TransformLocalToParent(V2c), mcol, use_Zbuffer);
    chrono::ChVector<> V1d(-x_size * 0.5, y_size * 0.5, 0);
    chrono::ChVector<> V2d(-x_size * 0.5, -y_size * 0.5, 0);
    irrlicht::ChIrrTools::drawSegment(driver, mpos.TransformLocalToParent(V1d), mpos.TransformLocalToParent(V2d), mcol, use_Zbuffer);

    for (int iy = 0; iy < Z.GetColumns(); ++iy)
    {
        double mystep = y_size / Z.GetColumns();
        double my = -0.5 * y_size + iy * mystep + 0.5 * mystep;
        for (int ix = 0; ix < Z.GetRows(); ++ix)
        {
            double mxstep = x_size / Z.GetRows();
            double mx = -0.5 * x_size + ix * mxstep + 0.5 * mxstep;
            if (ix > 0)
            {
                chrono::ChVector<> Vx1(mx - mxstep, my, Z(ix - 1, iy));
                chrono::ChVector<> Vx2(mx, my, Z(ix, iy));
                irrlicht::ChIrrTools::drawSegment(driver, mpos.TransformLocalToParent(Vx1), mpos.TransformLocalToParent(Vx2), mcol, use_Zbuffer);
            }
            if (iy > 0)
            {
                chrono::ChVector<> Vy1(mx, my - mystep, Z(ix, iy - 1));
                chrono::ChVector<> Vy2(mx, my, Z(ix, iy));
                irrlicht::ChIrrTools::drawSegment(driver, mpos.TransformLocalToParent(Vy1), mpos.TransformLocalToParent(Vy2), mcol, use_Zbuffer);
            }
        }
    }
}

template <typename asset_type = ElectricParticleAsset>
bool GetAsset(const std::shared_ptr<ChBody>& body, std::shared_ptr<asset_type>& desired_asset)
{
    for (auto asset_iter = body->GetAssets().begin(); asset_iter != body->GetAssets().end(); ++asset_iter)
    {
        auto desired_asset_ptr = std::dynamic_pointer_cast<asset_type>(*asset_iter);
        if (desired_asset_ptr)
        {
            desired_asset = desired_asset_ptr;
            return true;
        }
    }
    return false;
}


ElectrostaticCoronaSeparator::ElectrostaticCoronaSeparator(ChSystem& mphysicalSystem)
{
    // Initialize splitter positions
    splitters[0].pos_x = 0.1;
    splitters[1].pos_x = 0.2;
    splitters[2].pos_x = 0.3;
    splitters[0].pos_y = 0;
    splitters[1].pos_y = 0;
    splitters[2].pos_y = 0;


    // Set small collision envelopes for objects that will be created from now on..
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.001); //0.002
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0005); //0.0008
    // Set contact breaking/merging tolerance of Bullet:
    collision::ChCollisionSystemBullet::SetContactBreakingThreshold(0.001);

    // Important! dt is small, and particles are small, so it's better to keep this small...
    mphysicalSystem.SetMaxPenetrationRecoverySpeed(0.15);// not needed in INT_TASORA, only for INT_ANITESCU
    mphysicalSystem.SetMinBounceSpeed(0.1);
}

void ElectrostaticCoronaSeparator::apply_forces(ChSystem* msystem)
{
    // Compute parameters on-the-fly (some parameters like L or U might have changed meanwhile..)
    h1 = (pow(L, 2) + pow((drum_diameter / 2), 2) - ((electrode_diameter / 2), 2)) / (2 * L); //analytical parameter****ida
    h2 = (pow(L, 2) - pow((drum_diameter / 2), 2) + ((electrode_diameter / 2), 2)) / (2 * L);//analytical parameter****ida
    j = sqrt(pow(h1, 2) - pow((drum_diameter / 2), 2));//analytical parameter****ida
    f = U / log(((h1 + j - (drum_diameter / 2)) * (h2 + j - (electrode_diameter / 2))) / ((drum_diameter / 2) + j - h1) * ((electrode_diameter / 2) + j - h2));//analytical parameter****ida

    // Loop on all bodies:
    for (auto body_iter = msystem->IterBeginBodies(); body_iter != msystem->IterEndBodies(); ++body_iter)
    {
        // Do the computation of forces only on bodies that had the 'ElectricParticleAsset' attached
        std::shared_ptr<ElectricParticleAsset> electric_asset;
        if (!GetAsset(*body_iter, electric_asset))
        {
            continue;
        }

        //ChVector<> diam = electric_asset->Cdim;
        //double sigma = electric_asset->conductivity;

        // Remember to reset 'user forces accumulators':
        (*body_iter)->Empty_forces_accumulators();

        // initialize speed of air (steady, if outside fan stream): 
        //ChVector<> abs_wind(0, 0, 0);

        // calculate the position of body COG with respect to the drum COG:
        // TODO: adapt the notation
        //ChVector<> mrelpos = drum_csys.TransformParentToLocal((*body_iter)->GetPos());
        ChVector<> mrelpos = drum_csys.TransformParentToLocal((*body_iter)->GetPos());
        double distx = mrelpos.x;
        double disty = mrelpos.y;
        ChVector<> velocity = (*body_iter)->GetPos_dt();

        //// UNUSED variables
        //double velocityx = velocity.x;
        //double velocityy = velocity.y;
        //double velocityz = velocity.z;

        //double velocity_norm_sq = velocity.Length2();
        ////ChQuaternion<> rot_velocity=abody->GetRot_dt;

        //// Polar coordinates of particles respect to the axis of the rotor, may be useful later **ALEX
        //double distance = pow(distx * distx + disty * disty, 0.5);
        //double phi = atan2(disty, distx);
        //double phi2 = atan2(-velocity.y, velocity.x);


        //
        // STOKES FORCES
        //

        // Approximate to sphere radius. Ida: this can be improved, by having Stokes forces for three Cdim x y z values maybe
        double average_rad = 0.5 * electric_asset->Cdim.Length();
        ChVector<> StokesForce = electric_asset->StokesForce;
        electric_asset->StokesForce = (-6 * CH_C_PI * eta * average_rad) * velocity;
        (*body_iter)->Accumulate_force(StokesForce, (*body_iter)->GetPos(), false);


        // Electric field
        double x_1 = distx * cos(alpha) + disty * sin(alpha);
        double y_1 = disty * cos(alpha) - distx * sin(alpha);

        double Ex = (((j - h1 + x_1) / (pow((j - h1 + x_1), 2) + pow(y_1, 2)) + ((j + h1 - x_1) / (pow((j + h1 - x_1), 2) + pow(y_1, 2))) * f));
        double Ey = ((y_1 / (pow((j - h1 + x_1), 2) + pow(y_1, 2)) - (y_1 / (pow((j + h1 - x_1), 2) + pow(y_1, 2))) * f));
        double Ez = 0;

        ChVector<> vE(Ex, Ey, Ez);
        double E = vE.Length();
        double Emax = -11.818 * U - 514.87;


        // Compute forces based on material type
        switch (electric_asset->GetMaterial())
        {
            case ElectricParticleAsset::material_type::e_mat_copper:
            case ElectricParticleAsset::material_type::e_mat_metal:
            {
                // charge the particle? (contact w. drum)
                if ((distx > 0) && (disty > 0))
                {
                    if (electric_asset->chargeM == 0)
                    {
                        electric_asset->chargeM = (2. / 3.) * pow(CH_C_PI, 3) * epsilon * pow(average_rad, 2) * E;
                        electric_asset->chargeM *= (1.0 - 0.3 * ChRandom());
                    }
                }


                ChVector<> ElectricForce = electric_asset->ElectricForce;

                electric_asset->ElectricForce = 0.832 * electric_asset->chargeM * vE;
                //GetLog() << "ElectricForce" << ElectricForce << "\n";
                // switch off electric forces if too out-of-plane
                if ((mrelpos.z > drum_width * 0.5) || (mrelpos.z < -drum_width * 0.5))
                    ElectricForce = 0;

                (*body_iter)->Accumulate_force(ElectricForce, (*body_iter)->GetPos(), false);
            }
            break;

            case ElectricParticleAsset::material_type::e_mat_pcb6:
            case ElectricParticleAsset::material_type::e_mat_pcb7:
            case ElectricParticleAsset::material_type::e_mat_pcb8:
            case ElectricParticleAsset::material_type::e_mat_pcb9:
            case ElectricParticleAsset::material_type::e_mat_pcb10:
            case ElectricParticleAsset::material_type::e_mat_plastic:
            {
                // charge the particle? (contact w. drum)
                if ((distx > 0.04) && (disty > 0))
                {
                    if (electric_asset->chargeP == 0)
                    {
                        electric_asset->chargeP = 3 * CH_C_PI * epsilonO * pow(2 * average_rad, 2) * Emax * (epsilonR / (epsilonR + 2)); // charge
                        electric_asset->chargeP *= (1.0 - 0.3 * ChRandom());
                    }
                } //15000000,750000,450000
                  // discharge the particle? (contact w. blade)
                if (distx < -(drum_diameter * 0.5 - 0.009) && (disty > -(drum_diameter * 0.5 + 0.009)) || sqrt(pow(distx, 2) + pow(disty, 2)) > (1.03 * drum_diameter * 0.5))
                {
                    electric_asset->chargeP = 0; // charge
                }

                ChVector<> ElectricImageForce = electric_asset->ElectricImageForce;


                electric_asset->ElectricImageForce.x = -((pow(electric_asset->chargeP, 2)) / (4 * CH_C_PI * epsilon * pow((2 * average_rad), 2)) * cos(atan2(disty, distx)));
                electric_asset->ElectricImageForce.y = -((pow(electric_asset->chargeP, 2)) / (4 * CH_C_PI * epsilon * pow((2 * average_rad), 2)) * sin(atan2(disty, distx)));
                electric_asset->ElectricImageForce.z = 0;


                // switch off electric forces if too out-of-plane
                if ((mrelpos.z > drum_width * 0.5) || (mrelpos.z < -drum_width * 0.5))
                    ElectricImageForce = 0;


                (*body_iter)->Accumulate_force(ElectricImageForce, (*body_iter)->GetPos(), false);
            }
            break;

            default:

                break;
        }

        //
        // AERODYNAMIC FORCES
        //

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

    } // end for() loop on all bodies
}


/// It creates a list of particles with properties given by the \c filename content;
/// \c filename should have the informations needed stored in this order:
/// material_ID, particleID, thickness, mass, main inertias {x3}, number of convex hull points, x coordinates of convex hull points, y coordinates of convex hull points
/// The units are [mm], [mm^2], [mm^2 mg]
bool ElectrostaticCoronaSeparator::LoadParticleScan(std::string filename)
{
    std::ifstream particlefile;
    particlefile.open(filename, std::ifstream::in);

    std::cout << "File open? " << particlefile.is_open() << std::endl;


    std::string line;
    while (std::getline(particlefile, line))
    {
        std::istringstream iss(line);
        std::string field;
        int mat_ID, particle_ID, convex_hull_Npoint;
        double thickness, particle_mass, I1, I2, I3;
        std::vector<ChVector<double>> convex_hull_points;

        try
        {
            getline(iss, field, ',');
            if (field == "") // check for empty lines
                break;
            static_cast<std::stringstream>(field) >> mat_ID;
            getline(iss, field, ',');
            static_cast<std::stringstream>(field) >> particle_ID;
            getline(iss, field, ',');
            static_cast<std::stringstream>(field) >> thickness;
            getline(iss, field, ',');
            static_cast<std::stringstream>(field) >> particle_mass;
            getline(iss, field, ',');
            static_cast<std::stringstream>(field) >> I1;
            getline(iss, field, ',');
            static_cast<std::stringstream>(field) >> I2;
            getline(iss, field, ',');
            static_cast<std::stringstream>(field) >> I3;
            getline(iss, field, ',');
            static_cast<std::stringstream>(field) >> convex_hull_Npoint;

            convex_hull_points.resize(2 * convex_hull_Npoint);

            // unit conversion
            particle_mass *= 1e-6;
            thickness *= 1e-3;
            I1 *= 1e-12;
            I2 *= 1e-12;
            I3 *= 1e-12;

            // acquire x coordinates of convex hull vertices and give thickness to the body
            for (auto convex_hull_point_sel = 0; convex_hull_point_sel < convex_hull_Npoint; ++convex_hull_point_sel)
            {
                getline(iss, field, ',');
                static_cast<std::stringstream>(field) >> convex_hull_points[convex_hull_point_sel](0);
                convex_hull_points[convex_hull_point_sel](0) /= 1000;

                // the points are given on a flat surface (2D); they will be duplicated and shifted by 'thickness' in order to give thick objects
                convex_hull_points[convex_hull_point_sel + convex_hull_Npoint](0) = convex_hull_points[convex_hull_point_sel](0);

                convex_hull_points[convex_hull_point_sel](1) = 0;
                convex_hull_points[convex_hull_point_sel + convex_hull_Npoint](1) = thickness;
            }

            // acquire y coordinates of convex hull vertices
            for (auto convex_hull_point_sel = 0; convex_hull_point_sel < convex_hull_Npoint; ++convex_hull_point_sel)
            {
                getline(iss, field, ',');
                static_cast<std::stringstream>(field) >> convex_hull_points[convex_hull_point_sel](2);
                convex_hull_points[convex_hull_point_sel](2) /= 1000;

                convex_hull_points[convex_hull_point_sel + convex_hull_Npoint](2) = convex_hull_points[convex_hull_point_sel](2);
            }

            // create the electric asset
            auto basic_elec_asset = std::make_shared<ElectricParticleAsset>(getElectricParticleProperty_material_type(mat_ID));
            basic_elec_asset->particleID = particle_ID;

            // create body
            auto convex_hull_temp = std::make_shared<ChBodyEasyConvexHull>(convex_hull_points, basic_elec_asset->GetDensity(), true, true);
            convex_hull_temp->SetMass(particle_mass);
            convex_hull_temp->SetInertiaXX(ChVector<>(I1, I2, I3)); // the inertia is overwritten with the input value
            convex_hull_temp->AddAsset(basic_elec_asset);

            scanned_particles.push_back(convex_hull_temp);


        }
        catch (...)
        {
            std::cout << "Misformatted input file: " << filename << std::endl;
            return false;
        }
    }

    return true;
}

void ElectrostaticCoronaSeparator::create_debris_particlescan(double particles_second, ChSystem& mysystem, irrlicht::ChIrrApp* irr_application)
{
    assert(scanned_particles.size() > 0);

    auto mmaterial = std::make_shared<ChMaterialSurface>();
    mmaterial->SetFriction(0.4f);
    mmaterial->SetRollingFriction(0);
    mmaterial->SetSpinningFriction(0);
    mmaterial->SetRestitution(0);


    double xnozzle = -conveyor_length*0.25;
    double ynozzle = drum_diameter / 2 + 0.1;

    //TODO: is it so critical the number of particles that has to be set with this bizarre method?
    double exact_particles_dt = mysystem.GetStep() * particles_second;
    double particles_dt = floor(exact_particles_dt);
    double remaind = exact_particles_dt - particles_dt;
    if (remaind > ChRandom()) particles_dt += 1;

    // the routine creates clones of the particles whose informations are acquired by the particle scan
    for (auto i = 0; i < particles_dt; i++)
    {
        // Choose a particle to be cloned
        // the copy constructor of ChBody:
        // - copies all the member variables
        // - DOES share the assets
        // - does NOT copy or share the collision model
        int selected_particle = floor(ChRandom() * scanned_particles.size());
        auto mrigidBody = std::make_shared<ChBody>(*scanned_particles[selected_particle]); // member variables copied
        mrigidBody->GetCollisionModel()->AddCopyOfAnotherModel(scanned_particles[selected_particle]->GetCollisionModel()); // collision model shared

        // Purge the particle from the 'master' assets of the 'scanned_particle' and link a new one.
        // the new body still SHARES the assets with the 'master' particle!
        std::shared_ptr<ElectricParticleAsset> elec_asset_old;
        GetAsset(mrigidBody, elec_asset_old); // the pointer still points to the same asset of the master particle
        auto elec_asset = std::make_shared<ElectricParticleAsset>(*elec_asset_old);
        for (auto iter = mrigidBody->GetAssets().begin(); iter != mrigidBody->GetAssets().end(); ++iter) //TODO: purge the old ElectricParticleAsset shared pointer in other way
        {
            if (std::dynamic_pointer_cast<ElectricParticleAsset>(*iter))
            {
                iter = mrigidBody->GetAssets().erase(iter);
                if (iter == mrigidBody->GetAssets().end())
                    break;
            }
        }
        elec_asset->UpdateCharacteristicSize(*mrigidBody);
        elec_asset->birthdate = mysystem.GetChTime();
        mrigidBody->AddAsset(elec_asset);

        mrigidBody->AddAsset(elec_asset->GetDefaultColorAsset()); //share the default asset with all the bodies of the same material

        mrigidBody->SetPos(ChVector<>((ChRandom() - 0.5) * xnozzlesize + xnozzle, ynozzle + i * 0.005, (ChRandom() - 0.5) * znozzlesize));
        std::srand(std::time(0));
        ChVector<double> vect = { static_cast<double>(std::rand()) / RAND_MAX, static_cast<double>(std::rand()) / RAND_MAX, static_cast<double>(std::rand()) / RAND_MAX }; vect.Normalize();
        double alpha = static_cast<double>(std::rand()) / RAND_MAX*CH_C_PI_2;
        ChQuaternion<double> q = { cos(alpha),vect(0)*sin(alpha),vect(1)*sin(alpha),vect(2)*sin(alpha) };


        mrigidBody->SetRot(q);
        mrigidBody->SetMaterialSurface(mmaterial);

        auto traj_asset = std::make_shared<ParticleTrajectory>();
        mrigidBody->AddAsset(traj_asset);

        mysystem.AddBody(mrigidBody);

        // If Irrlicht is used, setup also the visualization proxy:
        if (irr_application)
        {
            irr_application->AssetBind(mrigidBody);
            irr_application->AssetUpdate(mrigidBody);
        }

        // This is an optional hack that largely affects the stability of the
        // simulation. 
        // In fact, if particles happen to spin too fast, the collision detection
        // starts to be very slow, and maybe also inaccurate; also, the time integration
        // could diverge. To get rid of this problem wihtout reducing too much the timestep, 
        // one can enable a limit on angular velocity and/or linear velocity. NOTE that 
        // this achieves greater stability at the cost of lower realism of the simulation, 
        // so it should not be abused. ***ALEX

        bool do_velocity_clamping = true;

        if (mrigidBody && do_velocity_clamping)
        {
            mrigidBody->SetLimitSpeed(true);
            mrigidBody->SetMaxSpeed(100);
            mrigidBody->SetMaxWvel(250);
        }


    }

}





void ElectrostaticCoronaSeparator::purge_debris_byage(ChSystem& mysystem, double max_age)
{
    for (auto body_sel = 0; body_sel < mysystem.Get_bodylist()->size(); ++body_sel)
    {
        auto body = (*mysystem.Get_bodylist())[body_sel];
        std::shared_ptr<ElectricParticleAsset> elec_ass;
        if (GetAsset(body, elec_ass))
        {
            double particle_age = mysystem.GetChTime() - elec_ass->birthdate;
            if (particle_age > max_age)
            {
                mysystem.Remove(body);
            }
        }
    }
}

void ElectrostaticCoronaSeparator::purge_debris_byposition(ChSystem& mysystem, ChVector<> min_position, ChVector<> max_position)
{
    for (auto body_sel = 0; body_sel < mysystem.Get_bodylist()->size(); ++body_sel)
    {
        auto body = (*mysystem.Get_bodylist())[body_sel];

        if (body->GetPos()(0) < min_position(0) || body->GetPos()(1) < min_position(1) || body->GetPos()(2) < min_position(2) ||
            body->GetPos()(0) > max_position(0) || body->GetPos()(1) > max_position(1) || body->GetPos()(2) > max_position(2))
        {
            mysystem.Remove(body);
        }

    }
}

void ElectrostaticCoronaSeparator::drawForces(irrlicht::ChIrrApp& application, double scalefactor)
{
    for (auto body_iter = application.GetSystem()->IterBeginBodies(); body_iter != application.GetSystem()->IterEndBodies(); ++body_iter)
    {
        std::shared_ptr<ElectricParticleAsset> elec_ass;
        if (GetAsset(*body_iter, elec_ass))
        {
            ChVector<> custom_force = (*body_iter)->Get_accumulated_force();
            double val = custom_force.Length() / 1e-4;
            custom_force *= scalefactor ? scalefactor : application.GetSymbolscale();
            int r = static_cast<int>(255 * std::min(std::max(0.0, 1.5 - abs(1 - 4 * (val - 0.5))), 1.0));
            int g = static_cast<int>(255 * std::min(std::max(0.0, 1.5 - abs(1 - 4 * (val - 0.25))), 1.0));
            int b = static_cast<int>(255 * std::min(std::max(0.0, 1.5 - abs(1 - 4 * val)), 1.0));
            irrlicht::ChIrrTools::drawSegment(application.GetVideoDriver(),
                                              (*body_iter)->GetPos(),
                                              (*body_iter)->GetPos() + custom_force, irr::video::SColor(255, 0, 0, 255));
        }
    }
}

void ElectrostaticCoronaSeparator::updateTrajectories(irrlicht::ChIrrApp& application, bool only_those_on_drum)
{
    for (auto body_iter = application.GetSystem()->IterBeginBodies(); body_iter != application.GetSystem()->IterEndBodies(); ++body_iter)
    {
        std::shared_ptr<ParticleTrajectory> traj_ass;
        if (GetAsset(*body_iter, traj_ass))
        {
            traj_ass->push_back((*body_iter)->GetPos(), (*body_iter)->GetPos_dt());
        }
    }
}

void ElectrostaticCoronaSeparator::drawTrajectories(irrlicht::ChIrrApp& application, bool only_those_on_drum)
{
    for (auto body_iter = application.GetSystem()->IterBeginBodies(); body_iter != application.GetSystem()->IterEndBodies(); ++body_iter)
    {
        std::shared_ptr<ParticleTrajectory> traj_ass;
        if (GetAsset(*body_iter, traj_ass) && (traj_ass->positions.size() > 1))
        {
            auto iteratorA = traj_ass->positions.begin();
            auto iteratorB = iteratorA; ++iteratorB;
            auto iterator_speed = traj_ass->speeds.begin();

            while (iteratorB != traj_ass->positions.end())
            {
                double scalarspeed = iterator_speed->Length();
                double normalizedspeed = scalarspeed / 5.0;
                int r = static_cast<int>(255*std::min(std::max(0.0, 1.5 - abs(1 - 4 * (normalizedspeed - 0.5))), 1.0));
                int g = static_cast<int>(255*std::min(std::max(0.0, 1.5 - abs(1 - 4 * (normalizedspeed - 0.25))), 1.0));
                int b = static_cast<int>(255*std::min(std::max(0.0, 1.5 - abs(1 - 4 * normalizedspeed)), 1.0));
                irr::video::SColor mcol(r, g, b, 255);

                irrlicht::ChIrrTools::drawSegment(application.GetVideoDriver(), *iteratorA, *iteratorB, mcol);

                ++iteratorA;
                ++iteratorB;
                ++iterator_speed;

            }
        }
    }
}


int ElectrostaticCoronaSeparator::Setup(ChSystem& system, irrlicht::ChIrrApp* application)
{
    auto mvisual_orange = std::make_shared<ChColorAsset>();
    mvisual_orange->SetColor(ChColor(0.9f, 0.4f, 0.2f));

    auto mvisual_grey = std::make_shared<ChColorAsset>();
    mvisual_grey->SetColor(ChColor(0.81f, 0.85f, 0.88f));

    auto mvisual_bluegrey = std::make_shared<ChColorAsset>();
    mvisual_bluegrey->SetColor(ChColor(0.33f, 0.69f, 0.97f));

    auto mtexture = std::make_shared<ChTexture>();
    mtexture->SetTextureFilename("cyltext.jpg");

    // DRUM: rotating cylinder with electrostatic charge
    auto mrigidBodyDrum = std::make_shared<ChBodyEasyCylinder>(drum_diameter / 2, drum_width, 7500, true, true);
    mrigidBodyDrum->SetNameString("drum");
    mrigidBodyDrum->AddAsset(mvisual_bluegrey);
    mrigidBodyDrum->GetMaterialSurface()->SetFriction(0.5);
    mrigidBodyDrum->GetMaterialSurface()->SetRestitution(0);
    mrigidBodyDrum->GetMaterialSurface()->SetRollingFriction(0);
    mrigidBodyDrum->GetMaterialSurface()->SetSpinningFriction(0);
    mrigidBodyDrum->SetPos(ChVector<>(0, 0, 0));
    mrigidBodyDrum->SetRot(ChQuaternion<>(sin(CH_C_PI_4), cos(CH_C_PI_4), 0, 0));
    system.AddBody(mrigidBodyDrum);
    mrigidBodyDrum->GetCollisionModel()->SetFamily(3);
    mrigidBodyDrum->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
    mrigidBodyDrum->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);
    drum_csys.pos = mrigidBodyDrum->GetPos();

    // CONVEYOR: vibrating plate on which particles flow before falling on the drum
    // the conveyor is modeled as fixed box with no friction so that the particles will slip on it
    double conveyor_inclination = 10 * (CH_C_PI / 180);
    auto mrigidBodyConveyor = std::make_shared<ChBodyEasyBox>(conveyor_length, conveyor_thick, drum_width, true, true);
    mrigidBodyConveyor->SetNameString("conveyor");
    mrigidBodyConveyor->AddAsset(mvisual_grey);

    mrigidBodyConveyor->GetMaterialSurface()->SetFriction(0); // it was 0.2
    mrigidBodyConveyor->GetMaterialSurface()->SetRestitution(0);
    mrigidBodyConveyor->GetMaterialSurface()->SetRollingFriction(0);
    mrigidBodyConveyor->GetMaterialSurface()->SetSpinningFriction(0);
    mrigidBodyConveyor->SetPos(ChVector<>(-conveyor_length / 2 * cos(conveyor_inclination) + conveyor_thick / 2 * sin(conveyor_inclination), drum_diameter / 2 + conveyor_thick / 2 * cos(conveyor_inclination) + conveyor_length / 2 * sin(conveyor_inclination), 0));
    mrigidBodyConveyor->SetRot(ChQuaternion<>(sin(conveyor_inclination / 2), 0, 0, cos(conveyor_inclination / 2)));
    mrigidBodyConveyor->SetBodyFixed(true);
    system.AddBody(mrigidBodyConveyor);
    mrigidBodyConveyor->GetCollisionModel()->SetFamily(2);
    mrigidBodyConveyor->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
    mrigidBodyConveyor->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);

    // SPLITTERS: they intercept the particles that fall from the drum
    auto mrigidBodySplitter1 = std::make_shared<ChBodyEasyBox>(0.005, drum_diameter / 2, drum_diameter, 7500, splitters_collide, true);
    mrigidBodySplitter1->SetNameString("splitter01");
    mrigidBodySplitter1->AddAsset(mvisual_orange);
    mrigidBodySplitter1->SetBodyFixed(true);
    mrigidBodySplitter1->GetMaterialSurface()->SetFriction(0.1f);
    mrigidBodySplitter1->SetPos(ChVector<>(0.2, -(drum_diameter*0.5) - conveyor_thick / 2, 0));
    system.AddBody(mrigidBodySplitter1);
    mrigidBodySplitter1->GetCollisionModel()->SetFamily(3);
    mrigidBodySplitter1->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
    mrigidBodySplitter1->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);


    auto mrigidBodySplitter2 = std::make_shared<ChBodyEasyBox>(0.005, drum_diameter / 2, drum_diameter, 7500, splitters_collide, true);
    mrigidBodySplitter2->SetNameString("splitter02");
    mrigidBodySplitter2->SetBodyFixed(true);
    mrigidBodySplitter2->AddAsset(mvisual_orange);
    mrigidBodySplitter2->GetMaterialSurface()->SetFriction(0.1f);
    mrigidBodySplitter2->SetPos(ChVector<>(0.4, -(drum_diameter*0.5) - conveyor_thick / 2, 0));
    system.AddBody(mrigidBodySplitter2);
    mrigidBodySplitter2->GetCollisionModel()->SetFamily(3);
    mrigidBodySplitter2->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
    mrigidBodySplitter2->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);


    // Fixed truss on which the drum rotates
    auto drum_pivot = std::make_shared<ChBody>();
    drum_pivot->SetPos(mrigidBodyDrum->GetPos());
    drum_pivot->SetRot(mrigidBodyDrum->GetRot());
    drum_pivot->SetBodyFixed(true);
    drum_pivot->SetNameString("drum_pivot");
    system.AddBody(drum_pivot);

    // Impose rotation of the drum with ChLinkEngine
    auto drum_engine = std::make_shared<ChLinkEngine>();
    drum_engine->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
    drum_engine->Initialize(mrigidBodyDrum, drum_pivot, ChCoordsys<>(mrigidBodyDrum->GetPos()));
    drum_speed_function = std::dynamic_pointer_cast<ChFunction_Const>(drum_engine->Get_spe_funct());
    drum_speed_function->Set_yconst(drumspeed_rpm*CH_C_2PI / 60);
    system.AddLink(drum_engine);

    if (application)
    {
        /*application->SetPaused(true);*/
        //application->SetTryRealtime(true);
        application->AssetBindAll();
        application->AssetUpdateAll();
    }


    return 0;
}

int ElectrostaticCoronaSeparator::RunSimulation(irrlicht::ChIrrApp& application)
{

    // This is for GUI tweaking of system parameters...
    UserInterfaceEventReceiver receiver(&application, this);
    // note how to add the custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);


    //
    // Create an (optional) exporter to POVray 
    // 

    postprocess::ChPovRay pov_exporter = postprocess::ChPovRay(application.GetSystem());

    if (save_POV_screenshots)
    {
        // Sets some file names for in-out processes.
        pov_exporter.SetTemplateFile("../objects/_template_POV.pov");
        pov_exporter.SetOutputScriptFile("rendering_frames.pov");

        // save the .dat files and the .bmp files
        // in two subdirectories, to avoid cluttering the current directory...
        ChFileutils::MakeDirectory("outputPOV");
        ChFileutils::MakeDirectory("animPOV");

        pov_exporter.SetOutputDataFilebase("outputPOV/my_state");
        pov_exporter.SetPictureFilebase("animPOV/picture");

        // optional: modify the POV default light
        pov_exporter.SetLight(ChVector<>(0.5f, 0.75f, 0.0f), ChColor(0.1f, 0.1f, 0.1f), true);
        pov_exporter.SetCamera(ChVector<>(0.5f, 0.75f, 0.5f), ChVector<>(0.2f, 0.6f, 0.f), 30, false);

        // optional: use SetCustomPOVcommandsScript() to add further POV commands,
        // ex. create an additional light, and an additional grid, etc. 
        // Remember the "\" char per each newline.

        pov_exporter.SetCustomPOVcommandsScript(" \
				light_source {   \
				  <0.5, 0.8, 0.2>  \
				  color rgb<1.7,1.7,1.7> \
				  area_light <0.4, 0, 0>, <0, 0, 0.4>, 5, 5 \
				  adaptive 1 \
				  jitter\
				} \
				//object{ Grid(0.5,0.01, rgb<0.9,0.9,0.9>, rgbt<1,1,1,1>) rotate <90, 0, 0>  } \
				//object{ Grid(0.1,0.04, rgb<1.5,1.5,1.5>, rgbt<1,1,1,1>) rotate <90, 0, 0> translate 0.001*z} \
			");

        // IMPORTANT! Tell to the POVray exporter that 
        // he must take care of converting the shapes of
        // all items (that have been added so far)!
        pov_exporter.AddAll();

        // IMPORTANT! Create the two .pov and .ini files for POV-Ray (this must be done
        // only once at the beginning of the simulation).
        pov_exporter.ExportScript();
    }


    //
    // For enabling Irrlicht visualization of assets (that have been added so far)
    //

    application.AssetBindAll();
    application.AssetUpdateAll();
    application.AddShadowAll();


    // 
    // PROCESS THE FLOW with these tools:
    // 


    // Create also a ChParticleProcessor configured as a
    // counter of particles that flow into a rectangle with a statistical distribution to plot:
    //  -create the trigger:
    double flowmeter_length = this->flowmeter_xmax - this->flowmeter_xmin;
    auto distrrectangle = std::make_shared<ChParticleEventFlowInRectangle>(flowmeter_length, flowmeter_width);
    distrrectangle->rectangle_csys = ChCoordsys<>(
        drum_csys.pos + ChVector<>(this->flowmeter_xmin + 0.5 * flowmeter_length,
        this->flowmeter_y,
        0), // position of center rectangle
        Q_from_AngAxis(-CH_C_PI_2, VECT_X)); // rotate rectangle so that its Z is up
    distrrectangle->margin = 0.05;
    //  -create the counter, with 20x10 resolution of sampling, on x y
    //    This is defined in ProcessFlow.h and distinguishes plastic from metal
    auto countdistribution = std::make_shared<ProcessFlow>(this->flowmeter_bins, 1);
    //  -create the processor and plug in the trigger and the counter:
    ChParticleProcessor processor_distribution;
    processor_distribution.SetEventTrigger(distrrectangle);
    processor_distribution.SetParticleEventProcessor(countdistribution);


    // Create a remover, i.e. an object that takes care 
    // of removing particles that are inside or outside some volume.
    auto distrrectangle2 = std::make_shared<ChParticleEventFlowInRectangle>(0.20, 0.30);
    distrrectangle2->rectangle_csys = distrrectangle->rectangle_csys;
    distrrectangle2->margin = 0.05;

    std::shared_ptr<ChParticleProcessEventRemove> removal_event(new ChParticleProcessEventRemove);
    ChParticleProcessor processor_remover;
    processor_remover.SetEventTrigger(distrrectangle2);
    processor_remover.SetParticleEventProcessor(removal_event);


    // 
    // THE SOFT-REAL-TIME CYCLE
    //

    application.GetSystem()->Set_G_acc(ChVector<>(0, -9.81, 0));

    int savenum = 0;

    ChFileutils::MakeDirectory("screenshots");
    ChFileutils::MakeDirectory("output");

    application.GetSystem()->ShowHierarchy(GetLog());

    //create_debris_particlescan_original(timestep, 1000, *application.GetSystem(), &application);

    while (application.GetDevice()->run())
    {
        //if (application.GetSystem()->GetChTime() > this->Tmax)
        //    break;

        application.GetVideoDriver()->beginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        application.DrawAll();
        application.DoStep();

        if (!application.GetPaused())
        {
            totframes++;

            //GetLog() << "WOT: " << application.GetSystem()->GetChTime() << "\n";

            // Apply the forces caused by electrodes of the CES machine
            apply_forces(application.GetSystem());

            if (receiver.checkbox_plotECSforces->isChecked())
                drawForces(application, ECSforces_scalefactor);

            if (receiver.checkbox_plottrajectories->isChecked())
                drawTrajectories(application, true);


            //// Continuosly create debris that fall on the conveyor belt
            //this->emitter.EmitParticles(*application.GetSystem(), application.GetTimestep()); //***TEST***

            create_debris_particlescan(particle_flow, *application.GetSystem(), &application);

            if (serial_communicator && totframes % 100 == 0)
            {
                splitters[0].pos_x = static_cast<double>(rand() % 100)/100;
                serial_communicator->UpdateMotorPosition(splitters);
                //serial_communicator->SendCommand(EcsSerialCommunicator::message_output_t::MOTOR_HALT);
            }


            // Limit the max age (in seconds) of debris particles on the scene, 
            // deleting the oldest ones, for performance
            // purge_debris_byage(*application.GetSystem(), this->max_particle_age);

            // remove bodies that go out of scope
            ChVector<> min_vector(-5, -drum_diameter, -drum_width*1.5);
            ChVector<> max_vector(+5, +2, +drum_width*1.5);
            purge_debris_byposition(*application.GetSystem(), min_vector, max_vector);

            // Use the processor to count particle flow in the rectangle section:
            //processor_distribution.ProcessParticles(*application.GetSystem());

            // Continuosly check if some particle must be removed:
            //processor_remover.ProcessParticles(*application.GetSystem());

            // Update the drum speed in the link
            drum_speed_function->Set_yconst(-drumspeed_rpm*CH_C_2PI / 60);

            // update the assets containing the trajectories, if any
            if (receiver.checkbox_plottrajectories->isChecked())
                if (totframes % 20 == 0)
                    updateTrajectories(application, true);

            // Save data on file (each n integration steps, to avoid filling
            // the hard disk and to improve performance)
            if (totframes % saveEachNframes == 0)
            {
                savenum++;

                // Save log file as '.txt' files?

                if (save_dataset)
                {
                    char buffer[120];
                    sprintf(buffer, "output/esempio_output%05d.txt", savenum);
                    GetLog() << "\n saving dataset: " << buffer;
                    ChStreamOutAsciiFile file_for_output(buffer);
                    for (size_t i = 0; i < application.GetSystem()->Get_bodylist()->size(); i++)
                    {
                        auto abody = (*application.GetSystem()->Get_bodylist())[i];

                        // Fetch the ElectricParticleAsset asset from the list
                        for (size_t na = 0; na < abody->GetAssets().size(); na++)
                        {
                            std::shared_ptr<ChAsset> myasset = abody->GetAssetN(na);

                            if (auto electricproperties = std::dynamic_pointer_cast<ElectricParticleAsset>(myasset))
                            {
                                // ok, its a particle!

                                //double my_cond  = electricproperties->conductivity ;
                                //auto my_ElectricForce = electricproperties->ElectricForce;
                                //auto my_ElectricImageForce = electricproperties->ElectricImageForce;
                                //auto my_StokesForce = electricproperties->StokesForce;
                                //double rad = ((abody->GetMass()) * 3) / ((abody->GetDensity()) * 4 * CH_C_PI);
                                //ElectricParticleAsset::fraction_type fraction_identifier = electricproperties->e_fraction; // id will be 0=box, 1=cylinder, 2=sphere, 3=hull, 4=shavings, etc. (see enum)
                                //ElectricParticleAsset::material_type material_identifier = electricproperties->e_material; // id will be 0=plastic, 1=metal, 2=others (see enum)

                                //// Save on disk some infos...
                                //file_for_output << abody->GetIdentifier() << ", "
                                //    << fraction_identifier << ", "
                                //    << abody->GetPos().x << ", "
                                //    << abody->GetPos().y << ", "
                                //    << abody->GetPos().z << ", "
                                //    << abody->GetDensity() << ", "
                                //    //<< my_cond << ", "
                                //    << abody->GetMass() << ", "
                                //    << pow(rad, 1.0 / 3) << "\n";
                                ////<< abody->GetPos_dt().x << ", "
                                ////<< abody->GetPos_dt().y << ", "
                                ////<< abody->GetPos_dt().z << ", "
                                ////<< my_StokesForce << ", "
                                ////<< my_ElectricImageForce << ", "
                                ////<< my_ElectricForce << "\n";
                            }
                        }
                    }
                }

                // Save Irrlicht screenshots?

                if (save_irrlicht_screenshots)
                {
                    irr::video::IImage* image = application.GetVideoDriver()->createScreenShot();
                    char buffer[120];
                    sprintf(buffer, "screenshots/screenshot%05d.bmp", savenum);
                    GetLog() << "\n saving screenshot: " << buffer;
                    if (image)
                        application.GetVideoDriver()->writeImageToFile(image, buffer);
                    image->drop();
                }

                // Save POV screenshots?

                if (save_POV_screenshots)
                {
                    pov_exporter.ExportData();
                    GetLog() << "\n saving POV data n." << savenum;
                }
            } // end saving code
        }

        // Just for fun, plot the distribution matrices, 
        // i.e. countdistribution->mmass_plastic etc.
        // In this case, normalize to integral , and scale on Z
        double yscalefactor_plastic;
        double totmass_plastic = 0;
        for (int ir = 0; ir < countdistribution->mmass_plastic.GetRows(); ++ir)
            for (int ic = 0; ic < countdistribution->mmass_plastic.GetColumns(); ++ic)
                totmass_plastic += countdistribution->mmass_plastic(ir, ic);
        if (totmass_plastic == 0)
            yscalefactor_plastic = 0; // if not yet particle passed through sampling rectangle
        else
            yscalefactor_plastic = (0.002 * countdistribution->mmass_plastic.GetRows() * countdistribution->mmass_plastic.GetColumns()) / totmass_plastic;

        drawDistribution(application.GetVideoDriver(),
                         countdistribution->mmass_plastic * yscalefactor_plastic,
                         distrrectangle->rectangle_csys,
                         distrrectangle->Xsize,
                         distrrectangle->Ysize,
                         irr::video::SColor(255, 255, 0, 0));

        double yscalefactor_metal;
        double totmass_metal = 0;
        for (int ir = 0; ir < countdistribution->mmass_metal.GetRows(); ++ir)
            for (int ic = 0; ic < countdistribution->mmass_metal.GetColumns(); ++ic)
                totmass_metal += countdistribution->mmass_metal(ir, ic);
        if (totmass_plastic == 0)
            yscalefactor_metal = 0; // if not yet particle passed through sampling rectangle
        else
            yscalefactor_metal = (0.002 * countdistribution->mmass_metal.GetRows() * countdistribution->mmass_metal.GetColumns()) / totmass_metal;

        drawDistribution(application.GetVideoDriver(),
                         countdistribution->mmass_metal * yscalefactor_metal,
                         distrrectangle->rectangle_csys,
                         distrrectangle->Xsize,
                         distrrectangle->Ysize,
                         irr::video::SColor(255, 0, 255, 255));


        application.GetVideoDriver()->endScene();
    }

    //// At the end ot the T max simulation time, 
    //// save output distributions to disk (non normalized for unit area/volume), 
    //// they can be a nxm matrix of 2d bins or a n-vector of 1d bins
    //GetLog() << "\n saving output distributions... \n ";

    //ChStreamOutAsciiFile file_for_metal("out_distribution_metal.txt");
    //countdistribution->mmass_metal.StreamOUTdenseMatlabFormat(file_for_metal);
    //ChStreamOutAsciiFile file_for_plastic("out_distribution_plastic.txt");
    //countdistribution->mmass_plastic.StreamOUTdenseMatlabFormat(file_for_plastic);


    GetLog() << "\n Simulation Terminated. \n ";

    return 0;
}

ElectricParticleAsset::material_type ElectrostaticCoronaSeparator::getElectricParticleProperty_material_type(int matID)
{
    switch (matID)
    {
        case 3:
            return ElectricParticleAsset::material_type::e_mat_copper;
        case 6:
            return ElectricParticleAsset::material_type::e_mat_pcb6;
        case 7:
            return ElectricParticleAsset::material_type::e_mat_pcb7;
        case 8:
            return ElectricParticleAsset::material_type::e_mat_pcb8;
        case 9:
            return ElectricParticleAsset::material_type::e_mat_pcb9;
        case 10:
            return ElectricParticleAsset::material_type::e_mat_pcb10;
        default:
            return ElectricParticleAsset::material_type::e_mat_other;
    }
}
