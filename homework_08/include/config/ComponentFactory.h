#pragma once
#include "Types.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"

enum class SolverType   { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType   { FILE_LOADER };

class ComponentFactory {
public:
    IBallisticSolver* createSolver(SolverType type,
                                   DroneConfig cfg, AmmoParams ammo);
    ITargetProvider*  createProvider(ProviderType type, const char* filename,
                                     float simTS, float arrayTS);
    IConfigLoader*    createLoader(LoaderType type,
                                   const char* fnameAmmo, const char* fnameDrone);
};