#include "ElectricParticleAssets.h"

const std::unordered_map<ElectricParticleAsset::material_type, std::tuple<double, double, std::shared_ptr<ChColorAsset>>> ElectricParticleAsset::default_mat_properties = {
    { material_type::e_mat_plastic ,{ 1300, 100, std::make_shared<ChColorAsset>(1.00f, 1.00f, 1.00f) } },
    { material_type::e_mat_metal   ,{ 7500, 100, std::make_shared<ChColorAsset>(0.42f, 0.49f, 0.58f) } },
    { material_type::e_mat_copper  ,{ 8960, 100, std::make_shared<ChColorAsset>(0.67f, 0.33f, 0.00f) } },
    { material_type::e_mat_pcb6    ,{ 5000, 100, std::make_shared<ChColorAsset>(0.02f, 0.39f, 0.24f) } },
    { material_type::e_mat_pcb7    ,{ 5000, 100, std::make_shared<ChColorAsset>(0.58f, 0.02f, 0.02f) } },
    { material_type::e_mat_pcb8    ,{ 5000, 100, std::make_shared<ChColorAsset>(0.44f, 0.55f, 0.02f) } },
    { material_type::e_mat_pcb9    ,{ 5000, 100, std::make_shared<ChColorAsset>(0.00f, 0.93f, 0.86f) } },
    { material_type::e_mat_pcb10   ,{ 5000, 100, std::make_shared<ChColorAsset>(0.92f, 0.99f, 0.00f) } },
    { material_type::e_mat_other   ,{ 5000, 100, std::make_shared<ChColorAsset>(0.00f, 0.00f, 0.00f) } },
};



