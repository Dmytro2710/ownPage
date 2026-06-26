#pragma once
#include "Types.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"
#include <memory>

enum class SolverType   { ANALYTICAL, TABLE };
enum class ProviderType { JSON, THREAD_SAFE };
enum class LoaderType   { FILE_LOADER };

class ComponentFactory {
public:
    std::unique_ptr<IBallisticSolver> createSolver(SolverType type,
                                                   DroneConfig cfg, AmmoParams ammo);
    std::unique_ptr<ITargetProvider>  createProvider(ProviderType type, const char* filename,
                                                     float simTS, float arrayTS, float tScale = 1.0f);
    std::unique_ptr<IConfigLoader>    createLoader(LoaderType type,
                                                   const char* fnameAmmo, const char* fnameDrone);
};