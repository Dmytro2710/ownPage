#include "states/StateMoving.h"
#include "states/StateDecelerating.h"
#include "Types.h"
using namespace dlink;

std::unique_ptr<IDroneState> StateMoving::execute(DronePos& drone, const DropPoint& dp, const DroneCfg& config) {
    Coord toTarget = dp.firePos - drone.currentPos;
    float neededDir = std::atan2(toTarget.y, toTarget.x);
    float delta = norm_angle(neededDir - drone.currentDirection);
    if (std::abs(delta) > config.turnThreshold) {
        drone.currentState = DECELERATING;
        return std::make_unique<StateDecelerating>();
    }
    drone.currentState = MOVING;
    return std::make_unique<StateMoving>();
}

std::string StateMoving::name() const {
    return "Moving";
}