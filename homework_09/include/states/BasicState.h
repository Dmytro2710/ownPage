#pragma once
#include "Types.h"
#include <memory>


class IDroneState {
public:
    virtual ~IDroneState() = default;
    virtual std::unique_ptr<IDroneState> execute(DronePos& drone, const DropPoint& dp, DroneConfig config) = 0;
    virtual std::string name() const = 0;
};
