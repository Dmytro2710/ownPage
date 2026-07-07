#include "states/StateStoped.h"
#include "states/StateTurning.h"
#include "states/StateAccelerating.h"
#include "Types.h"
using namespace dlink;

std::unique_ptr<IDroneState> StateStoped::execute(DronePos& drone, const DropPoint& dp, const DroneCfg& config) {
    Coord toTarget = dp.firePos - drone.currentPos;
    float neededDir = std::atan2(toTarget.y, toTarget.x);
    float delta = norm_angle(neededDir - drone.currentDirection);
    if (std::abs(delta) > config.turnThreshold) {
        drone.currentState = TURNING;
        return std::make_unique<StateTurning>();
    } else {
        drone.currentState = ACCELERATING;
        return std::make_unique<StateAccelerating>();
    }
}

std::string StateStoped::name() const {
    return "Stoped";
}
