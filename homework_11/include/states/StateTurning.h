#pragma once
#include "BasicState.h"
#include "Types.h"
#include "drone_link.h"

class StateTurning : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DronePos& drone, const DropPoint& dp, const dlink::DroneCfg& config) override;
    std::string name() const override;  
};