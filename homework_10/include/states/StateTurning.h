#pragma once
#include "BasicState.h"
#include "Types.h"

class StateTurning : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DronePos& drone, const DropPoint& dp, DroneConfig config) override;
    std::string name() const override;  
};