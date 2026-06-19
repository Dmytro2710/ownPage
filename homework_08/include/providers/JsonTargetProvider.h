#pragma once
#include "Types.h"
#include "interfaces/ITargetProvider.h"
#include <vector>
class JsonTargetProvider : public ITargetProvider {
    std::vector<std::vector<Coord>> targets;
    int     targetCount;
    int     timeSteps;
    float   arrayTimeStep;
    float   simTimeStep;
    bool    loadingFail;
    int     currentStep = 0;
public:
    JsonTargetProvider(const char* filename, float simTimeStep, float arrayTimeStep);
    int    getTargetCount() override;
    Target getTarget(int index) override;
    void   setCurrentStep(int step) override;
    ~JsonTargetProvider();
};