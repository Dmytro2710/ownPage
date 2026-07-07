#include "states/StateDecelerating.h"
#include "states/StateTurning.h"
#include "states/StateAccelerating.h"
#include "Types.h"
using namespace dlink;

std::unique_ptr<IDroneState> StateDecelerating::execute(DronePos& drone, const DropPoint& dp, const DroneCfg& config) {
    Coord toTarget = dp.firePos - drone.currentPos;
    float neededDir = std::atan2(toTarget.y, toTarget.x);
    float delta = norm_angle(neededDir - drone.currentDirection);

    if (drone.currentSpeed <= 0.0f) {
        drone.currentState = (std::abs(delta) > config.turnThreshold) ? TURNING : ACCELERATING;
        if (drone.currentState == ACCELERATING) {
            return std::make_unique<StateAccelerating>();
        }
        return std::make_unique<StateTurning>();
    }
    return std::make_unique<StateDecelerating>();
}

std::string StateDecelerating::name() const {
    return "Decelerating";
}