#include "states/StateTurning.h"
#include "states/StateAccelerating.h"
#include "Types.h"
using namespace dlink;

std::unique_ptr<IDroneState> StateTurning::execute(DronePos& drone, const DropPoint& dp, const DroneCfg& config) {
    Coord toTarget = dp.firePos - drone.currentPos;
    float neededDir = std::atan2(toTarget.y, toTarget.x);
    float delta = norm_angle(neededDir - drone.currentDirection);
    
    if (std::abs(delta) <= config.turnThreshold) {
        drone.currentState = ACCELERATING;
        return std::make_unique<StateAccelerating>();
    }
    drone.currentState = TURNING;
    return std::make_unique<StateTurning>();
}

std::string StateTurning::name() const {
    return "Turning";
}