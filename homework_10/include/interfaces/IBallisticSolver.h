#pragma once
#include "Types.h"

class IBallisticSolver {
public:
    virtual DropPoint solve(DronePos drone, Target curTarget) = 0;
    virtual ~IBallisticSolver() {}
};