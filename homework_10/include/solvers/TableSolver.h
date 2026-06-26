#pragma once
#include "Types.h"
#include "interfaces/IBallisticSolver.h"
#include "solvers/BallisticTable.h"

class TableSolver : public IBallisticSolver {
    const DroneConfig currentDrone;
    const AmmoParams  currentAmo;
    BallisticTable table;

    Coord interpolate_target(float totalTime, Target curTarget);
    float stop_penalty(DronePos drone);
    float norm_angle(float a);

public:
    TableSolver(DroneConfig currentDrone, AmmoParams currentAmo, const char* tablePath);
    DropPoint solve(DronePos drone, Target curTarget) override;
    ~TableSolver() override {}
};