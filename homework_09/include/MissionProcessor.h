#pragma once
#include "Types.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"
#include <memory>

    class MissionProcessor {
        std::unique_ptr<IBallisticSolver> solver;
        std::unique_ptr<ITargetProvider>  provider;
        std::unique_ptr<IConfigLoader>    loader;
        DroneConfig       config;
        AmmoParams        ammo;
        DronePos          drone;
        int               currentTargetIdx;
    public:
        MissionProcessor(std::unique_ptr<IBallisticSolver> s, std::unique_ptr<ITargetProvider> p, std::unique_ptr<IConfigLoader> l);
        void init();
        bool hasNext();
        DropPoint step();
        void reset();
        void changeSolver(std::unique_ptr<IBallisticSolver> s);
        void simulate();
    private:
        float norm_angle_local(float a);
        float stop_penalty_local(DronePos drone);
        void update_drone(DropPoint dp);
    };