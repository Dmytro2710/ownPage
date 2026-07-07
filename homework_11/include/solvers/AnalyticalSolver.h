#pragma once
#include "Types.h"
#include "interfaces/IBallisticSolver.h"
//#include "drone_link.h"

class AnalyticalSolver : public IBallisticSolver {
    const dlink::DroneCfg currentDrone;
    const dlink::AmmoCfg  currentAmmo;

    float ballistics(float& out_t, DronePos drone);
    Coord interpolate_target(float totalTime, Target curTarget);
    float stop_penalty(DronePos drone);
    float norm_angle(float a);

public:
    AnalyticalSolver(const dlink::DroneCfg& currentDrone, const dlink::AmmoCfg& currentAmmo);
    DropPoint solve(DronePos drone, Target curTarget) override;
    ~AnalyticalSolver() override {}
};