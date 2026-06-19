#include "config/ComponentFactory.h"
#include "solvers/AnalyticalSolver.h"
#include "providers/JsonTargetProvider.h"
#include "config/FileConfigLoader.h"

IBallisticSolver* ComponentFactory::createSolver(SolverType type,
                                                  DroneConfig cfg, AmmoParams ammo) {
    switch (type) {
        case SolverType::ANALYTICAL:
            return new AnalyticalSolver(cfg, ammo);
        default:
            return nullptr;
    }
}

ITargetProvider* ComponentFactory::createProvider(ProviderType type, const char* filename,
                                                   float simTS, float arrayTS) {
    switch (type) {
        case ProviderType::JSON:
            return new JsonTargetProvider(filename, simTS, arrayTS);
        default:
            return nullptr;
    }
}

IConfigLoader* ComponentFactory::createLoader(LoaderType type,
                                              const char* fnameAmmo, const char* fnameDrone) {
    switch (type) {
        case LoaderType::FILE_LOADER:
            return new FileConfigLoader(fnameAmmo, fnameDrone);
        default:
            return nullptr;
    }
}