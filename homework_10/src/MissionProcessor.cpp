#include "Types.h"
#include "states/BasicState.h"
#define _USE_MATH_DEFINES
#include "MissionProcessor.h"
#include "Logger.h"
#include <fstream>
#include <cmath>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <memory.h>
#include "states/StateStoped.h"
#include "states/StateTurning.h"
#include "states/StateAccelerating.h"
#include "states/StateDecelerating.h"
#include "states/StateMoving.h"
#include <thread>


using json = nlohmann::json;

MissionProcessor::MissionProcessor(std::unique_ptr<IBallisticSolver> s, std::unique_ptr<ITargetProvider> p, std::unique_ptr<IConfigLoader> l, DronePhysics* physics)
    : solver(std::move(s)), provider(std::move(p)), loader(std::move(l)), physics_(physics) {}
void MissionProcessor::init() {
    config = loader->getConfig();
    LOG("simTimeStep=" << config.simTimeStep 
        << " timeScale=" << config.timeScale
        << " sleepMs =" << (config.simTimeStep / config.timeScale * 1000.0f));
    ammo   = loader->getAmoParams();
    /*drone  = {{config.startPos.x, config.startPos.y}, 
            0.0f, 
            config.initialDir, 
            config.altitude, 
            STOPPED};*/
    currentTargetIdx = 0;
}
bool MissionProcessor::hasNext() {return currentTargetIdx < provider->getTargetCount();};

void MissionProcessor::reset() {currentTargetIdx = 0;};
void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> s) {solver = std::move(s);};
bool MissionProcessor::isThreadReady() const {return is_ready.load();}
void MissionProcessor::start() {start_flag.store(true);}
void MissionProcessor::stop() {stop_flag.store(true);}


void MissionProcessor::run() {
    this->init();
    is_ready.store(true);
    LOG("Mission ready");

    while (!start_flag.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    auto nextStep = std::chrono::steady_clock::now();
    bool dropped_flag = false;
    DroneTelemetry currentTelemetry;
    DronePos drone;
    std::unique_ptr<IDroneState> currentState = std::make_unique<StateStoped>();
    std::vector<SimStep> simSteps;
    simSteps.reserve(MAX_STEPS);

    /*int sleepMs = static_cast<int>(
        (config.timeScale > 0)
        ? (config.simTimeStep / config.timeScale * 1000.0f)
        : config.simTimeStep * 1000.0f);*/
        
    auto stepDuration = std::chrono::duration<float>(
        config.simTimeStep / config.timeScale);
    int totalSteps = 0;

    while (!stop_flag.load() && !dropped_flag && totalSteps < MAX_STEPS) {
        LOG("Mission step start"); 
        auto wakeTime = std::chrono::steady_clock::now() + stepDuration;

        currentTelemetry = physics_->getTelemetry();
        LOG("First telemetry time=" << currentTelemetry.timeSecSinceStart);
        LOG("Telemetry got, pos=(" << currentTelemetry.pos.x << "," << currentTelemetry.pos.y << ")");
        drone = telemetryToPos(currentTelemetry, config.altitude);

        float penaltyChange = physics_->stop_penalty();
        int bestTarget = currentTargetIdx;
        float bestTime = solver->solve(drone, provider->getTarget(bestTarget)).estimatedTime;
        const float SWITCH_HYSTERESIS = 3.0f;
        for (int i = 0; i < provider->getTargetCount(); ++i) {
            if (i == bestTarget) continue;
            float t = solver->solve(drone, provider->getTarget(i)).estimatedTime + penaltyChange;
            if (t < bestTime - SWITCH_HYSTERESIS) { bestTime = t; bestTarget = i; }
        }
        currentTargetIdx = bestTarget;

        Target curTarget = provider->getTarget(currentTargetIdx);
        DropPoint dp = solver->solve(drone, curTarget);

        Coord dirVec   = {std::cos(drone.currentDirection), std::sin(drone.currentDirection)};
        Coord aimPoint = drone.currentPos + dirVec * dp.flightDistance;
        Coord toFire   = dp.firePos - drone.currentPos;
        float tarDirection = std::atan2(toFire.y, toFire.x);
        float ang_delta    = norm_angle(tarDirection - drone.currentDirection);

        simSteps.push_back({
            drone.currentPos,
            drone.currentDirection,
            static_cast<int>(drone.currentState),
            currentTargetIdx,
            dp.firePos,
            aimPoint,
            dp.predictedTarget,
            currentTelemetry.timeSecSinceStart
        });

        if (dp.isValid && drone.currentState == MOVING) {
            float distToFire = length(toFire);
            if (distToFire <= config.hitRadius && 
                std::abs(ang_delta) <= config.turnThreshold) {
                dropped_flag = true;
                LOG("DROP! target=" << currentTargetIdx
                    << " pos=(" << drone.currentPos.x << "," << drone.currentPos.y << ")"
                    << " time=" << currentTelemetry.timeSecSinceStart);
                break;
            }
        }

        auto nextStatePtr = currentState->execute(drone, dp, config);
        LOG("State: " << currentState->name() << " -> " << nextStatePtr->name()
        << " delta=" << norm_angle(std::atan2(dp.firePos.y - drone.currentPos.y,
                                          dp.firePos.x - drone.currentPos.x) 
                               - drone.currentDirection));
        currentState = std::move(nextStatePtr);
        physics_->sendCommand({drone.currentState, tarDirection});
        LOG("Command sent: state=" << drone.currentState << " dir=" << tarDirection);

        std::this_thread::sleep_until(wakeTime);
        totalSteps++;
        std::cout<<"[LOG] Step number: "<<totalSteps<<std::endl;
    }

    if (!dropped_flag)
        LOG("No drop occurred.");
    if (totalSteps >= MAX_STEPS)
        LOG("Max steps reached without drop");

    json out;
    out["totalSteps"] = simSteps.size();
    out["dropped"]    = dropped_flag;
    out["steps"]      = json::array();
    for (const auto& s : simSteps) {
        json step;
        step["position"]         = {{"x", s.pos.x}, {"y", s.pos.y}};
        step["direction"]        = s.direction;
        step["state"]            = s.state;
        step["targetIdx"]        = s.targetIdx;
        step["dropPoint"]        = {{"x", s.dropPoint.x},  {"y", s.dropPoint.y}};
        step["aimPoint"]         = {{"x", s.aimPoint.x},   {"y", s.aimPoint.y}};
        step["predictedTarget"]  = {{"x", s.predictedTarget.x}, {"y", s.predictedTarget.y}};
        step["timeSecSinceStart"] = s.timeSecSinceStart;
        out["steps"].push_back(step);
    }
    std::ofstream outFile("simulation_output.json");
    if (outFile) {
        outFile << out.dump(2);
        LOG("Simulation output saved to simulation_output.json");
    }
}

/*DropPoint MissionProcessor::step() {
    Target curentTarget = provider->getTarget(currentTargetIdx);
    DropPoint curTarDropPoint = solver->solve(drone, curentTarget);
    ++currentTargetIdx;
    return curTarDropPoint;
}*/


//====For better times====//
/*void MissionProcessor::simulate() {
    auto simSteps = std::make_unique<SimStep[]>(MAX_STEPS);
    bool dropped = false;
    int totalSteps = 0;
    float t_now = 0.0f;
    std::unique_ptr<IDroneState> currentState = std::make_unique<StateStoped>();
    while (totalSteps < MAX_STEPS) {
        t_now = totalSteps * config.simTimeStep;
        provider->setCurrentStep(totalSteps);
        float penaltyChange = stop_penalty_local(drone);
        int bestTarget = currentTargetIdx;
        float bestTime = solver->solve(drone, provider->getTarget(bestTarget)).estimatedTime;
        const float SWITCH_HYSTERESIS = 2.0f;
        for (int i = 0; i < provider->getTargetCount(); ++i) {
            if (i == bestTarget) continue;
            DropPoint dp = solver->solve(drone, provider->getTarget(i));
            float t = dp.estimatedTime + penaltyChange;

            if (t < bestTime - SWITCH_HYSTERESIS) {
                bestTime = t;
                bestTarget = i;
            }
        }
        currentTargetIdx = bestTarget;
        Target curTarget = provider->getTarget(currentTargetIdx);
        DropPoint dp = solver->solve(drone, curTarget);
        Coord dirVec  = {std::cos(drone.currentDirection), std::sin(drone.currentDirection)};
        Coord aimPoint = drone.currentPos + dirVec * dp.flightDistance;
        SimStep curentStep = {drone.currentPos, drone.currentDirection, static_cast<int>(drone.currentState), currentTargetIdx, dp.firePos, aimPoint, dp.predictedTarget};

        if (dp.isValid && currentState->name() == "Moving") {
            Coord toFire = dp.firePos - drone.currentPos;
            float distToFire = length(toFire);
            float ang_delta = norm_angle_local(std::atan2(toFire.y, toFire.x) - drone.currentDirection);
            if (distToFire <= config.hitRadius && std::abs(ang_delta) <= config.turnThreshold) {
                simSteps[totalSteps] = curentStep;
                dropped = true;
                LOG("DROP! step= " << totalSteps << ", target= " << currentTargetIdx<< ", pos=(" << drone.currentPos.x << ", " << drone.currentPos.y << ") time=" << t_now);  
                break;
            }
        }
        auto nextState = currentState->execute(drone, dp, config);
        if (nextState->name() != currentState->name()) {
            LOG("State change: " << currentState->name() << " -> " << nextState->name() << " at step " << totalSteps);
        }
        currentState = std::move(nextState);
        update_drone(dp);
        simSteps[totalSteps] = curentStep;
        ++totalSteps;
    }
        
    if (!dropped) {
        LOG("No drop occurred within " << MAX_STEPS << " steps.");
    }
    json out;
    out["totalSteps"] = totalSteps;
    out["dropped"] = dropped;
    out["steps"] = json::array();
    for (int i = 0; i < totalSteps; ++i) {
        json step;
        step["position"] = {{"x", simSteps[i].pos.x}, {"y", simSteps[i].pos.y}};
        step["direction"] = simSteps[i].direction;
        step["state"] = simSteps[i].state;
        step["targetIdx"] = simSteps[i].targetIdx;

        step["dropPoint"] = {{"x", simSteps[i].dropPoint.x}, {"y", simSteps[i].dropPoint.y}};
        step["aimPoint"] = {{"x", simSteps[i].aimPoint.x}, {"y", simSteps[i].aimPoint.y}};
        step["predictedTarget"] = {{"x", simSteps[i].predictedTarget.x}, {"y", simSteps[i].predictedTarget.y}};

        out["steps"].push_back(step);
    }
    std::ofstream outFile("simulation_output.json");
    if (outFile) {
        outFile << out.dump(2);
        LOG("Simulation output saved to simulation_output.json");
    } else {
        LOG("Failed to save simulation output.");
    }

    // delete[] simSteps;
    //simSteps = nullptr;
}*/
/*float MissionProcessor::norm_angle_local(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}
float MissionProcessor::stop_penalty_local(DronePos drone) {
    float acc = config.attackSpeed * config.attackSpeed / (2.0f * config.accelPath);
    switch (drone.currentState) {
        case STOPPED:      return 0.0f;
        case TURNING:      return 0.0f;
        case MOVING:       return config.attackSpeed / acc;
        case ACCELERATING: return drone.currentSpeed / acc;
        case DECELERATING: return drone.currentSpeed / acc;
    }
    return 0.0f;
}
void MissionProcessor::update_drone(DropPoint dp) {
    Coord dirVec = {std::cos(drone.currentDirection), std::sin(drone.currentDirection)};
    drone.currentPos = drone.currentPos + dirVec * (drone.currentSpeed * config.simTimeStep);
}*/
