#define _USE_MATH_DEFINES
#include "MissionProcessor.h"
#include "Logger.h"
#include <fstream>
#include <cmath>
#include <algorithm>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

MissionProcessor::MissionProcessor(IBallisticSolver* s, ITargetProvider* p, IConfigLoader* l)
    : solver(s), provider(p), loader(l) {}
void MissionProcessor::init() {
    config = loader->getConfig();
    ammo   = loader->getAmoParams();
    drone  = {{config.startPos.x, config.startPos.y}, 
            0.0f, 
            config.initialDir, 
            config.altitude, 
            STOPPED};
    currentTargetIdx = 0;
}
bool MissionProcessor::hasNext() {return currentTargetIdx < provider->getTargetCount();};
DropPoint MissionProcessor::step() {
    Target curentTarget = provider->getTarget(currentTargetIdx);
    DropPoint curTarDropPoint = solver->solve(drone, curentTarget);
    ++currentTargetIdx;
    return curTarDropPoint;
}
void MissionProcessor::reset() {currentTargetIdx = 0;};
void MissionProcessor::changeSolver(IBallisticSolver* s) {solver = s;};
void MissionProcessor::simulate() {
    SimStep* simSteps = new SimStep[MAX_STEPS];
    bool dropped = false;
    int totalSteps = 0;
    float t_now = 0.0f;
    while (totalSteps < MAX_STEPS) {
        t_now = totalSteps * config.simTimeStep;
        provider->setCurrentStep(totalSteps);
        float penaltyChange = stop_penalty_local(drone);
        int bestTarget = currentTargetIdx;
        float bestTime = solver->solve(drone, provider->getTarget(bestTarget)).estimatedTime;
        for (int i = 0; i < provider->getTargetCount(); ++i) {
            if (i == bestTarget) continue;
            DropPoint dp = solver->solve(drone, provider->getTarget(i));
            float t = dp.estimatedTime + penaltyChange;

            if (t < bestTime) {
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

        if (dp.isValid && drone.currentState == MOVING) {
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
        update_drone(dp, curTarget);
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

    delete[] simSteps;
    simSteps = nullptr;
    }
float MissionProcessor::norm_angle_local(float a) {
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
void MissionProcessor::update_drone(DropPoint dp, Target curTarget) {
    float acc = config.attackSpeed * config.attackSpeed / (2.0f * config.accelPath);
    Coord toTarget = dp.firePos - drone.currentPos;
    float neededDir = std::atan2(toTarget.y, toTarget.x);
    float delta = norm_angle_local(neededDir - drone.currentDirection);

    switch (drone.currentState) {
        case STOPPED:
            drone.currentState = (std::abs(delta) > config.turnThreshold) ? TURNING : ACCELERATING;
            if (drone.currentState == ACCELERATING) {
                drone.currentDirection = neededDir;
                drone.currentSpeed = 0.0f;
            }
            break;
        case TURNING:
            if (std::abs(delta) <= config.turnThreshold) {
                drone.currentState = ACCELERATING;
                drone.currentDirection = neededDir;
                drone.currentSpeed = 0.0f;
            } else {
                    float maxTurn = config.angularSpeed * config.simTimeStep;
                    drone.currentDirection = norm_angle_local(drone.currentDirection + (delta > 0 ? 1.0f : -1.0f) * std::min(maxTurn, std::abs(delta)));
                }
                break;
            case ACCELERATING:
                if (std::abs(delta) > config.turnThreshold) {
                    drone.currentState = DECELERATING;
                    break;
                }
                drone.currentSpeed += acc * config.simTimeStep;
                if (drone.currentSpeed >= config.attackSpeed) {
                    drone.currentState = MOVING;
                    drone.currentSpeed = config.attackSpeed;
                }
                break;
            case DECELERATING:
                drone.currentSpeed -= acc * config.simTimeStep;
                if (drone.currentSpeed <= 0.0f) {
                    drone.currentSpeed = 0.0f;
                    drone.currentState = (std::abs(delta) > config.turnThreshold) ? TURNING : ACCELERATING;
                }
                break;
            case MOVING:
                if (std::abs(delta) > config.turnThreshold) {
                    drone.currentState = DECELERATING;
                }
                break;
    }
    Coord dirVec = {std::cos(drone.currentDirection), std::sin(drone.currentDirection)};
    drone.currentPos = drone.currentPos + dirVec * (drone.currentSpeed * config.simTimeStep);
}
