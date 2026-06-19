#include "config/FileConfigLoader.h"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include "Logger.h"

#include <map>
#include <string>

using json = nlohmann::json;

FileConfigLoader::FileConfigLoader(const char* fnameAmmo, const char* fnameDrone)
    : fnameAmmo(fnameAmmo), fnameDrone(fnameDrone) {}

bool FileConfigLoader::load() {
    std::ifstream f1(fnameAmmo);
    std::ifstream f2(fnameDrone);
    if (!(f1 && f2)) {
        return false;
    }
    json j;
    f1 >> j;

    ammoCount = static_cast<int>(j.size());

    std::map<std::string, AmmoParams> ammoMap;
    for (const auto& ammo : j) {
        AmmoParams params;
        params.name = ammo["name"].get<std::string>();
        params.mass = ammo["mass"];
        params.drag = ammo["drag"];
        params.lift = ammo["lift"];
        ammoMap[params.name] = params;
    }
    json jDrone;
    f2 >> jDrone;

    currentDrone.startPos.x    = jDrone["drone"]["position"]["x"];
    currentDrone.startPos.y    = jDrone["drone"]["position"]["y"];
    currentDrone.altitude      = jDrone["drone"]["altitude"];
    currentDrone.initialDir    = jDrone["drone"]["initialDirection"];
    currentDrone.attackSpeed   = jDrone["drone"]["attackSpeed"];
    currentDrone.accelPath     = jDrone["drone"]["accelerationPath"];
    currentDrone.angularSpeed  = jDrone["drone"]["angularSpeed"];
    currentDrone.turnThreshold = jDrone["drone"]["turnThreshold"];
    currentDrone.simTimeStep   = jDrone["simulation"]["timeStep"];
    currentDrone.hitRadius     = jDrone["simulation"]["hitRadius"];
    currentDrone.arrayTimeStep = jDrone["targetArrayTimeStep"];

    currentDrone.ammoName = jDrone["ammo"].get<std::string>();
    auto it = ammoMap.find(currentDrone.ammoName);
    if (it != ammoMap.end()) {
        currentAmo = it->second;
        return true;
    }

    return false;
}

AmmoParams FileConfigLoader::getAmoParams() {
    return currentAmo;
}

DroneConfig FileConfigLoader::getConfig() {
    return currentDrone;
}