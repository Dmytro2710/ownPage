#pragma once
#include "Types.h"
#include "interfaces/IBallisticSolver.h"

class AnalyticalSolver : public IBallisticSolver {
    const DroneConfig currentDrone;
    const AmmoParams  currentAmo;

    float ballistics(float& out_t);
    Coord interpolate_target(float totalTime, Target curTarget);
    float stop_penalty(DronePos drone);
    float norm_angle(float a);

public:
    AnalyticalSolver(DroneConfig currentDrone, AmmoParams currentAmo);
    DropPoint solve(DronePos drone, Target curTarget) override;
    ~AnalyticalSolver() override {}
};