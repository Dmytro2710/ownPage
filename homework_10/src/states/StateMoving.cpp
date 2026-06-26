#include "states/StateMoving.h"
#include "states/StateDecelerating.h"
#include "Types.h"

std::unique_ptr<IDroneState> StateMoving::execute(DronePos& drone, const DropPoint& dp, DroneConfig config) {
    Coord toTarget = dp.firePos - drone.currentPos;
    float neededDir = std::atan2(toTarget.y, toTarget.x);
    auto norm_angle_local = [](float a) {
        while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
        while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
        return a;
    };
    float delta = norm_angle_local(neededDir - drone.currentDirection);
    if (std::abs(delta) > config.turnThreshold) {
        drone.currentState = DECELERATING;
        return std::make_unique<StateDecelerating>();
    }
    return std::make_unique<StateMoving>();
}

std::string StateMoving::name() const {
    return "Moving";
}