#include "config/ComponentFactory.h"
#include "Types.h"
#include "Logger.h"
#include "MissionProcessor.h"

int main() {
    ComponentFactory newFactory;

    auto loader = newFactory.createLoader(LoaderType::FILE_LOADER,
                                          "data/ammo.json", "data/config.json");

    if (!loader->load()) {
        LOG("ERROR: load() failed");
        delete loader;
        return 1;
    }
    LOG("Config loaded OK");
    LOG("attackSpeed=" << loader->getConfig().attackSpeed);
    LOG("ammo=" << loader->getConfig().ammoName);

    auto provider = newFactory.createProvider(ProviderType::JSON, "data/targets.json",
        loader->getConfig().simTimeStep,
        loader->getConfig().arrayTimeStep);

    LOG("targetCount=" << provider->getTargetCount());

    auto solver = newFactory.createSolver(SolverType::ANALYTICAL,
        loader->getConfig(),
        loader->getAmoParams());

    MissionProcessor* droneMission = new MissionProcessor(solver, provider, loader);
    droneMission->init();
    droneMission->simulate();

    delete droneMission;
    delete solver;
    delete provider;
    delete loader;
    return 0;
}