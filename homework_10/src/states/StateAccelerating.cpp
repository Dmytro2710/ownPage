#include "states/StateAccelerating.h"
#include "states/StateDecelerating.h"
#include "states/StateMoving.h"
#include "Types.h"

std::unique_ptr<IDroneState> StateAccelerating::execute(DronePos& drone, const DropPoint& dp, DroneConfig config) {
    float acc = config.attackSpeed * config.attackSpeed / (2.0f * config.accelPath);
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
    drone.currentSpeed += acc * config.simTimeStep;
    if (drone.currentSpeed >= config.attackSpeed) {
        drone.currentSpeed = config.attackSpeed;
        drone.currentState = MOVING;
        return std::make_unique<StateMoving>(); 
    }
    return std::make_unique<StateAccelerating>();
}

std::string StateAccelerating::name() const {
    return "Accelerating";
}