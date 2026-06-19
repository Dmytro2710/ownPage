#pragma once
#include "Types.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"

    class MissionProcessor {
        IBallisticSolver* solver;
        ITargetProvider*  provider;
        IConfigLoader*    loader;
        DroneConfig       config;
        AmmoParams        ammo;
        DronePos          drone;
        int               currentTargetIdx;
    public:
        MissionProcessor(IBallisticSolver* s, ITargetProvider* p, IConfigLoader* l);
        void init();
        bool hasNext();
        DropPoint step();
        void reset();
        void changeSolver(IBallisticSolver* s);
        void simulate();
    private:
        float norm_angle_local(float a);
        float stop_penalty_local(DronePos drone);
        void update_drone(DropPoint dp, Target curTarget);
    };