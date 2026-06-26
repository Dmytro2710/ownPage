#define _USE_MATH_DEFINES
#include "solvers/AnalyticalSolver.h"
#include <cmath>

AnalyticalSolver::AnalyticalSolver(DroneConfig currentDrone, AmmoParams currentAmo)
    : currentDrone(currentDrone), currentAmo(currentAmo) {}

float AnalyticalSolver::ballistics(float& out_t) {
    const float G = 9.81f;
    float           d = currentAmo.drag;
    float           m = currentAmo.mass;
    float           l = currentAmo.lift;
    float attackSpeed = currentDrone.attackSpeed;
    float           z = currentDrone.altitude;
    float a = d * G * m - 2.0f * d * d * l * attackSpeed;
    float b = 3.0f * d * l * m * attackSpeed - 3.0f * G * m * m;
    float c = 6.0f * m * m * z;
    float p = (-1.0f) * b * b / (3.0f * a * a);
    float q =  2.0f * b * b * b / (27.0f * a * a * a) + c / a;
    if (p >= 0) return -1.0f;
    float arg = (3.0f * q / (2.0f * p)) * std::sqrt(-3.0f / p);
    if (arg < -1.0f || arg > 1.0f) return -1.0f;
    float fi = std::acos(arg);
    float t  = 2.0f * std::sqrt(-p / 3.0f) * std::cos((fi + 4.0f * (float)M_PI) / 3.0f) - b / (3.0f * a);
    if (t <= 0) return -1.0f;

    float l2=l*l, l4=l2*l2, d2=d*d, d3=d2*d, d4=d3*d;
    float m2=m*m, m3=m2*m, m4=m3*m;
    float t2=t*t, t3=t2*t, t4=t3*t, t5=t4*t;
    float L2p1 = 1.0f + l2;
    float V = attackSpeed;

    float h = V*t
        - t2*d*V/(2.0f*m)
        + t3*(6.0f*d*G*l*m - 6.0f*d2*(l2-1.0f)*V)/(36.0f*m2)
        + t4*(-6.0f*d2*G*l*(1.0f+l2+l4)*m + 3.0f*d3*l2*L2p1*V + 6.0f*d3*l4*L2p1*V)
              /(36.0f*L2p1*L2p1*m3)
        + t5*(3.0f*d3*G*l2*l*m - 3.0f*d4*l2*L2p1*V)
              /(36.0f*L2p1*m4);

    if (h <= 0) return -1.0f;
    out_t = t;
    return h;
}

Coord AnalyticalSolver::interpolate_target(float totalTime, Target curTarget) {
    Coord predictedPos;
    predictedPos.x = curTarget.pos.x + totalTime * curTarget.velocity.x;
    predictedPos.y = curTarget.pos.y + totalTime * curTarget.velocity.y;
    return predictedPos;
}

float AnalyticalSolver::stop_penalty(DronePos drone) {
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

float AnalyticalSolver::norm_angle(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

DropPoint AnalyticalSolver::solve(DronePos drone, Target curTarget) {
    DropPoint result {};
    result.targetNumber = curTarget.number;
    float timeToDrop = 0.0f;
    float h = ballistics(timeToDrop);
    if (h < 0) {
        result.isValid       = false;
        result.estimatedTime = 1e9f;
        return result;
    }

    float acc   = currentDrone.attackSpeed * currentDrone.attackSpeed / (2.0f * currentDrone.accelPath);
    float t_acc = currentDrone.attackSpeed / acc;

    float totalTime = 0.0f;
    Coord pred  = curTarget.pos;
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