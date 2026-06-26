#pragma once
#include "Types.h"
#include "interfaces/ITargetProvider.h"
#include <vector>
#include <atomic>
#include <mutex>

class ThreadSafeTargetProvider : public ITargetProvider {
    std::vector<std::vector<Coord>> targets;
    int     targetCount;
    int     timeSteps;
    float   arrayTimeStep;
    float   simTimeStep;
    float   timeScale;
    std::atomic<bool> stop_flag{false};
    std::atomic<bool> start_flag{false};
    std::atomic<bool> is_ready{false};
    std::atomic<int>  currentStep{0};
    mutable std::mutex mtx_;
    std::vector<Target> currentTargets_;
public:
    ThreadSafeTargetProvider(const char* filename, float simTimeStep, float arrayTimeStep, float timeScale);
    int    getTargetCount() override;
    Target getTarget(int index) override;
    void   setCurrentStep(int step) override;
    bool   isThreadReady() const;
    void   start();
    void   stop();
    void   run();
    ~ThreadSafeTargetProvider();
};