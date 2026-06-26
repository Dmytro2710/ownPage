#include "config/ComponentFactory.h"
#include "providers/ThreadSafeTargetProvider.h"
#include "solvers/AnalyticalSolver.h"
#include "providers/JsonTargetProvider.h"
#include "config/FileConfigLoader.h"
#include "solvers/TableSolver.h"
#include <memory>

std::unique_ptr<IBallisticSolver> ComponentFactory::createSolver(SolverType type,
                                                                  DroneConfig cfg, AmmoParams ammo) {
    switch (type) {
        case SolverType::ANALYTICAL:
            return std::make_unique<AnalyticalSolver>(cfg, ammo);
        case SolverType::TABLE:
            return std::make_unique<TableSolver>(cfg, ammo, "data/ballistic_table.txt");
        default:
            return nullptr;
    }
}

std::unique_ptr<ITargetProvider> ComponentFactory::createProvider(ProviderType type, const char* filename,
                                                   float simTS, float arrayTS, float tScale) {
    switch (type) {
        case ProviderType::JSON:
            return std::make_unique<JsonTargetProvider>(filename, simTS, arrayTS);
        case ProviderType::THREAD_SAFE:
            return std::make_unique<ThreadSafeTargetProvider>(filename, simTS, arrayTS, tScale);
        default:
            return nullptr;
    }
}

std::unique_ptr<IConfigLoader> ComponentFactory::createLoader(LoaderType type,
                                                              const char* fnameAmmo, const char* fnameDrone) {
    switch (type) {
        case LoaderType::FILE_LOADER:
            return std::make_unique<FileConfigLoader>(fnameAmmo, fnameDrone);
        default:
            return nullptr;
    }
}