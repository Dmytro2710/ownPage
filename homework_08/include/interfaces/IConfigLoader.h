#pragma once
#include "Types.h"

class IConfigLoader {
public:
    virtual bool load() = 0;
    virtual AmmoParams getAmoParams() = 0;
    virtual DroneConfig getConfig() = 0;
    virtual ~IConfigLoader() {}
};