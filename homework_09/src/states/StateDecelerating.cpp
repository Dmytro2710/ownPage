#include "states/StateDecelerating.h"
#include "states/StateTurning.h"
#include "states/StateAccelerating.h"
#include "Types.h"

std::unique_ptr<IDroneState> StateDecelerating::execute(DronePos& drone, const DropPoint& dp, DroneConfig config) {
    float dec = config.attackSpeed * config.attackSpeed / (2.0f * config.accelPath);
    Coord toTarget = dp.firePos - drone.currentPos;
    float neededDir = std::atan2(toTarget.y, toTarget.x);
    auto norm_angle_local = [](float a) {
        while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
        while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
        return a;
    };
    float delta = norm_angle_local(neededDir - drone.currentDirection);
    drone.currentSpeed -= dec * config.simTimeStep;
    if (drone.currentSpeed <= 0.0f) {
        drone.currentSpeed = 0.0f;
        drone.currentState = (std::abs(delta) > config.turnThreshold) ? TURNING : ACCELERATING;
        if (drone.currentState == ACCELERATING) {
            drone.currentDirection = neededDir;
            return std::make_unique<StateAccelerating>();
        } else {
            return std::make_unique<StateTurning>();
        }
    }
    return std::make_unique<StateDecelerating>();
}

std::string StateDecelerating::name() const {
    return "Decelerating";
}