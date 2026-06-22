#include "states/StateStoped.h"
#include "states/StateTurning.h"
#include "states/StateAccelerating.h"
#include "Types.h"

std::unique_ptr<IDroneState> StateStoped::execute(DronePos& drone, const DropPoint& dp, DroneConfig config) {
    Coord toTarget = dp.firePos - drone.currentPos;
    float neededDir = std::atan2(toTarget.y, toTarget.x);
    auto norm_angle_local = [](float a) {
        while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
        while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
        return a;
    };
    float delta = norm_angle_local(neededDir - drone.currentDirection);
    if (std::abs(delta) > config.turnThreshold) {
        drone.currentState = TURNING;
        return std::make_unique<StateTurning>();
    } else {
        drone.currentDirection = neededDir;
        drone.currentSpeed = 0.0f;
        drone.currentState = ACCELERATING;
        return std::make_unique<StateAccelerating>();
    }
}

std::string StateStoped::name() const {
    return "Stoped";
}
