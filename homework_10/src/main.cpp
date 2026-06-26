#include "config/ComponentFactory.h"
#include "Types.h"
#include "Logger.h"
#include "MissionProcessor.h"
#include <memory>
#include <thread>
#include "providers/ThreadSafeTargetProvider.h"

int main() {
    ComponentFactory newFactory;

    auto loader = newFactory.createLoader(LoaderType::FILE_LOADER,
                                          "data/ammo.json", "data/config.json");
    if (!loader->load()) { LOG("ERROR: load() failed"); return 1; }

    auto tsProvider = std::make_unique<ThreadSafeTargetProvider>(
        "data/targets.json",
        loader->getConfig().simTimeStep,
        loader->getConfig().arrayTimeStep,
        loader->getConfig().timeScale);
    ThreadSafeTargetProvider* providerPtr = tsProvider.get();

    auto solver = newFactory.createSolver(SolverType::TABLE,
        loader->getConfig(), loader->getAmoParams());

    auto dronePhysic = std::make_unique<DronePhysics>(loader->getConfig());

    auto droneMission = std::make_unique<MissionProcessor>(
        std::move(solver), std::move(tsProvider),
        std::move(loader), dronePhysic.get());

    std::thread providerThread(&ThreadSafeTargetProvider::run, providerPtr);
    std::thread physicsThread(&DronePhysics::run, dronePhysic.get());
    std::thread missionThread(&MissionProcessor::run, droneMission.get());

    while (!providerPtr->isThreadReady() ||
       !dronePhysic->isThreadReady()  ||
       !droneMission->isThreadReady())
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    LOG("All threads ready, starting...");
    providerPtr->start();
    dronePhysic->start();
    droneMission->start();
    LOG("All started");

    missionThread.join();

    dronePhysic->stop();
    providerPtr->stop();
    physicsThread.join();
    providerThread.join();

    return 0;
}