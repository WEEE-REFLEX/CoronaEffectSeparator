#ifndef ELECTRICPARTICLEASSETS_H
#define ELECTRICPARTICLEASSETS_H

#include <core/ChVector.h>
#include <assets/ChAsset.h>
#include <assets/ChColorAsset.h>
#include <physics/ChBody.h>


using namespace chrono;


    class ElectricParticleAsset : public ChAsset
    {
    public:
        // Shape and material properties
        enum class fraction_type
        {
            e_fraction_box,
            e_fraction_cylinder,
            e_fraction_sphere,
            e_fraction_convexhull,
            e_fraction_shaving,
            e_fraction_others
        };

        enum class shape_type
        {
            e_box,
            e_cylinder,
            e_sphere,
            e_hull,
            e_shavings,
            e_other
        };

        enum class material_type
        {
            e_mat_plastic,
            e_mat_metal,
            e_mat_copper,
            e_mat_pcb6,
            e_mat_pcb7,
            e_mat_pcb8,
            e_mat_pcb9,
            e_mat_pcb10,
            e_mat_other
        };

    private:

        material_type e_material = material_type::e_mat_other;

        double custom_conductivity = -1;
        double custom_density = -1;

        static const std::unordered_map<material_type, std::tuple<double, double, std::shared_ptr<ChColorAsset>>> default_mat_properties;


    public:
        // Particle-specific properties
        ChVector<> Cdim{ 1,1,1 };
        double birthdate = 0;
        double chargeM = 0;		//***ida + ale (coulomb, for plastic)
        double chargeP = 0;
        ChVector<> ElectricForce;
        ChVector<> StokesForce;
        ChVector<> ElectricImageForce;
        size_t particleID = 0;

        shape_type e_shape = shape_type::e_other;
        fraction_type e_fraction = fraction_type::e_fraction_others;


        ElectricParticleAsset() {}
        explicit ElectricParticleAsset(material_type mat) : e_material(mat) {}
        virtual ~ElectricParticleAsset() {}

        ElectricParticleAsset(const ElectricParticleAsset& other)
        {
            particleID = other.particleID;
            e_material = other.e_material;
            custom_density = other.custom_density;
            custom_conductivity = other.custom_conductivity;
            e_shape = other.e_shape;
            e_fraction = other.e_fraction;
        }

        friend bool operator==(const ElectricParticleAsset& lhs, const ElectricParticleAsset& rhs);

        double GetDensity() const
        {
            if (custom_density > 0)
                return custom_density;

            auto found_item = default_mat_properties.find(e_material);
            return std::get<0>(found_item->second);
        }

        static double GetDefaultDensity(material_type mat)
        {
            auto found_item = default_mat_properties.find(mat);
            return std::get<0>(found_item->second);
        }

        void SetCustomDensity(double custom_dens) { custom_density = custom_dens; }


        double GetConductivity() const
        {
            if (custom_conductivity > 0)
                return custom_conductivity;

            auto found_item = default_mat_properties.find(e_material);
            return std::get<1>(found_item->second);
        }

        static double GetDefaultConductivity(material_type mat)
        {
            auto found_item = default_mat_properties.find(mat);
            return std::get<1>(found_item->second);
        }

        void SetCustomConductivity(double custom_cond) { custom_conductivity = custom_cond; }

        void RestoreDefaultMaterialProperties()
        {
            custom_density = -1;
            custom_conductivity = -1;
        }

        std::shared_ptr<ChColorAsset> GetDefaultColorAsset() const
        {
            auto found_item = default_mat_properties.find(e_material);
            return std::get<2>(found_item->second);
        }

        static std::shared_ptr<ChColorAsset> GetDefaultColorAsset(material_type mat)
        {
            auto found_item = default_mat_properties.find(mat);
            return std::get<2>(found_item->second);
        }

        void SetMaterial(material_type mat)
        {
            e_material = mat;
            RestoreDefaultMaterialProperties();
        }

        material_type GetMaterial() const { return e_material; }

        void UpdateCharacteristicSize(ChBody& body) //TODO: the Chbody argument should be const, but GetMass is not
        {
            ChVector<> Ine = body.GetInertiaXX();
            Cdim.x = sqrt((5 / (2 * body.GetMass())) * (Ine.y + Ine.z - Ine.x));
            Cdim.y = sqrt((5 / (2 * body.GetMass())) * (Ine.x + Ine.z - Ine.y));
            Cdim.z = sqrt((5 / (2 * body.GetMass())) * (Ine.x + Ine.y - Ine.z));
            Cdim.Scale(2);
        }

    };



    //
    // This can be added to store the trajectory on a per-particle basis.
    //
    class ParticleTrajectory : public ChAsset
    {
    public:
        std::list< ChVector<> > positions;
        std::list< ChVector<> > speeds;
        size_t max_points = 50;

        ParticleTrajectory() {}

        void SetMaxPoints(size_t maxp) { max_points = maxp; }

        void push_back(ChVector<>& new_position, ChVector<>& new_speed)
        {
            if (positions.size() == max_points)
            {
                positions.pop_front();
                speeds.pop_front();
            }

            positions.push_back(new_position);
            speeds.push_back(new_speed);
        }
    };


inline bool operator==(const ElectricParticleAsset& lhs, const ElectricParticleAsset& rhs)
    {
    
        if (lhs.particleID == rhs.particleID &&
            lhs.e_material == rhs.e_material &&
            lhs.custom_density == rhs.custom_density &&
            lhs.custom_conductivity == rhs.custom_conductivity &&
            lhs.e_shape == rhs.e_shape &&
            lhs.e_fraction == rhs.e_fraction)
            return false;

        return true;
    }





#endif

