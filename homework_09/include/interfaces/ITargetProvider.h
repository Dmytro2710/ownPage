#pragma once
#include "Types.h"

class ITargetProvider {
public:
    virtual int    getTargetCount() = 0;
    virtual Target getTarget(int index) = 0;
    virtual void   setCurrentStep(int step) = 0;
    virtual ~ITargetProvider() {}
};