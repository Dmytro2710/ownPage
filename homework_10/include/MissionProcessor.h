#pragma once
#include "Types.h"
#include "DronePhysics.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"
#include <memory>
#include <atomic>

class MissionProcessor {
    std::unique_ptr<IBallisticSolver> solver;
    std::unique_ptr<ITargetProvider>  provider;
    std::unique_ptr<IConfigLoader>    loader;
    DroneConfig       config;
    AmmoParams        ammo;
    DronePhysics*     physics_;
    int               currentTargetIdx;
    std::atomic<bool> is_ready{false};
    std::atomic<bool> start_flag{false};
    std::atomic<bool> stop_flag{false};
    
public:
    MissionProcessor(std::unique_ptr<IBallisticSolver> s, std::unique_ptr<ITargetProvider> p, std::unique_ptr<IConfigLoader> l, DronePhysics* physics);
    void init();
    bool isThreadReady() const;
    void start();
    void stop();
    bool hasNext();
    //DropPoint step();
    void reset();
    void changeSolver(std::unique_ptr<IBallisticSolver> s);
    //void simulate();
    void run();
};