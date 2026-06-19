#pragma once
#include "Types.h"
#include "interfaces/IConfigLoader.h"


class FileConfigLoader : public IConfigLoader {
    AmmoParams  currentAmo;
    DroneConfig currentDrone;
    int         ammoCount = 0;
    const char* fnameAmmo;
    const char* fnameDrone;
public:
    FileConfigLoader(const char* fnameAmmo, const char* fnameDrone);
    bool load() override; 
    AmmoParams  getAmoParams() override;
    DroneConfig getConfig() override;
};