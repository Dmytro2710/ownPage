#define _USE_MATH_DEFINES
#include "solvers/TableSolver.h"
#include <cmath>
#include <iostream>

TableSolver::TableSolver(DroneConfig currentDrone, AmmoParams currentAmo, const char* tablePath)
    : currentDrone(currentDrone), currentAmo(currentAmo) {
        if (!table.load(tablePath)) {
            std::cout << "[ERROR] Failed to load ballistic table: " << tablePath << std::endl;
        } else {
            std::cout << "[LOG] Ballistic table loaded OK" << std::endl;
        }
    }


Coord TableSolver::interpolate_target(float totalTime, Target curTarget) {
    Coord predictedPos;
    predictedPos.x = curTarget.targetCoord.x + totalTime * curTarget.targetXSpeed;
    predictedPos.y = curTarget.targetCoord.y + totalTime * curTarget.targetYSpeed;
    return predictedPos;
}

float TableSolver::stop_penalty(DronePos drone) {
    float acc = currentDrone.attackSpeed * currentDrone.attackSpeed / (2.0f * currentDrone.accelPath);
    switch (drone.currentState) {
        case STOPPED:      return 0.0f;
        case TURNING:      return 0.0f;
        case MOVING:       return currentDrone.attackSpeed / acc;
        case ACCELERATING: return drone.currentSpeed / acc;
        case DECELERATING: return drone.currentSpeed / acc;
    }
    return 0.0f;
}

float TableSolver::norm_angle(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

DropPoint TableSolver::solve(DronePos drone, Target curTarget) {
    DropPoint result {};
    result.targetNumber = curTarget.targetNumber;
    auto balResult = table.lookup(
            currentDrone.altitude,
            currentDrone.attackSpeed,
            currentAmo.mass,
            currentAmo.drag,
            currentAmo.lift
        );
    float h = balResult.hDist;
    float timeToDrop = balResult.t;
    if (h < 0) {
        result.isValid       = false;
        result.estimatedTime = 1e9f;
        return result;
    }

    float acc   = currentDrone.attackSpeed * currentDrone.attackSpeed / (2.0f * currentDrone.accelPath);
    float t_acc = currentDrone.attackSpeed / acc;

    float totalTime = 0.0f;
    Coord pred  = curTarget.targetCoord;
    Coord delta = pred - drone.currentPos;
    float D     = length(delta);

    const float CONV = currentDrone.simTimeStep * 0.1f;
    for (int i = 0; i < 10; ++i) {
        float prev = totalTime;
        pred  = interpolate_target(totalTime, curTarget);
        delta = pred - drone.currentPos;
        D     = length(delta);
        float driveD = (D - h > 0) ? (D - h) : 0;

        float t_drive = (driveD <= currentDrone.accelPath)
            ? std::sqrt(2.0f * driveD / acc)
            : t_acc + (driveD - currentDrone.accelPath) / currentDrone.attackSpeed;

        float ang    = norm_angle(std::atan2(delta.y, delta.x) - drone.currentDirection);
        float t_turn = (std::abs(ang) > currentDrone.turnThreshold)
            ? std::abs(ang) / currentDrone.angularSpeed + 2.0f * t_acc
            : 0.0f;

        totalTime = stop_penalty(drone) + t_turn + t_drive;
        if (std::abs(totalTime - prev) < CONV) break;
    }

    result.estimatedTime   = totalTime;
    result.isValid         = true;
    result.predictedTarget = pred;
    result.firePos.x       = pred.x + (drone.currentPos.x - pred.x) * h / D;
    result.firePos.y       = pred.y + (drone.currentPos.y - pred.y) * h / D;
    result.flightDistance  = h;
    return result;
}