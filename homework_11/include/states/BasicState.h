#pragma once
#include "Types.h"
#include <memory>
#include "drone_link.h"


class IDroneState {
public:
    virtual ~IDroneState() = default;
    virtual std::unique_ptr<IDroneState> execute(DronePos& drone, const DropPoint& dp, const dlink::DroneCfg& config) = 0;
    virtual std::string name() const = 0;
};
