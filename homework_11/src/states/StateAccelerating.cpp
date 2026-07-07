#include "states/StateAccelerating.h"
#include "states/StateDecelerating.h"
#include "states/StateMoving.h"
#include "Types.h"
using namespace dlink;

std::unique_ptr<IDroneState> StateAccelerating::execute(DronePos& drone, const DropPoint& dp, const DroneCfg& config) {
    Coord toTarget = dp.firePos - drone.currentPos;
    float neededDir = std::atan2(toTarget.y, toTarget.x);
    float delta = norm_angle(neededDir - drone.currentDirection);
    
    printf("[ACCEL] currentSpeed=%.3f attackSpeed=%.3f delta=%.3f threshold=%.3f\n",
        drone.currentSpeed, config.attackSpeed, delta, config.turnThreshold);
    fflush(stdout);
    
    if (std::abs(delta) > config.turnThreshold) {
        printf("[ACCEL] -> DECELERATING\n");
        drone.currentState = DECELERATING;
        return std::make_unique<StateDecelerating>();
    }
    if (drone.currentSpeed >= config.attackSpeed) {
        printf("[ACCEL] -> MOVING (speed reached)\n");
        drone.currentState = MOVING;
        return std::make_unique<StateMoving>();
    }
    drone.currentState = ACCELERATING;
    return std::make_unique<StateAccelerating>();
}

std::string StateAccelerating::name() const {
    return "Accelerating";
}