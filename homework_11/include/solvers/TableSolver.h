#pragma once
#include "Types.h"
#include "interfaces/IBallisticSolver.h"
#include "solvers/BallisticTable.h"

class TableSolver : public IBallisticSolver {
    const dlink::DroneCfg currentDrone;
    const dlink::AmmoCfg  currentAmmo;
    BallisticTable table;

    Coord interpolate_target(float totalTime, Target curTarget);
    float stop_penalty(DronePos drone);
    float norm_angle(float a);

public:
    TableSolver(const dlink::DroneCfg& currentDrone, const dlink::AmmoCfg& currentAmmo, const char* tablePath);
    DropPoint solve(DronePos drone, Target curTarget) override;
    ~TableSolver() override {}
};