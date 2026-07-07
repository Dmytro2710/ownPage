#include "MissionProcessor.h"
#include <sys/types.h>
#include "Logger.h"
#include "UartLink.h"
#include "Types.h"
#include "drone_link.h"
#include "states/BasicState.h"
#include "states/StateStoped.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include "solvers/TableSolver.h"
#include "GpioController.h"
#include <thread>
#include <chrono>

using namespace dlink;
using json = nlohmann::json;

MissionProcessor::MissionProcessor(UartLink& uart, GpioController& gpio, std::atomic<bool>& stopFlag)
    : uart_(uart), gpio_(gpio), currentState_(std::make_unique<StateStoped>()), stopFlag_(stopFlag) {
    
        if (!uart_.isOpen())    LOG("UART link is not open");
        if (!gpio_.isReady())   LOG("GPIO controller is not ready");
        LOG("MissionProcessor initialized");
    }

void MissionProcessor::handleAmmo(const AmmoCfg& ammo) {
    ammo_ = ammo;
    ammoReady_ = true;
}

void MissionProcessor::handleConfig(const DroneCfg& config) {
    config_ = config;
    configReady_ = true;
}

void MissionProcessor::handleTarget(const dlink::TargetPos& t) {
    targets_[t.id] = {{t.x, t.y}, {0.0f, 0.0f}, t.id};
}

void MissionProcessor::handleTelemetry(const Telemetry& telemetry) {
    if (!configReady_ || !ammoReady_) {
        LOG("Config or Ammo not ready, ignoring telemetry");
        return;
    }
    if (!solverReady_) {
        tryCreateSolver();
        LOG("Solver not ready, ignoring telemetry");
        return;
    }
    drone_ = telemetryToPos(telemetry);
    drone_.currentState = currentDroneState_;
    float penaltyChange = stop_penalty(drone_, config_);
    uint8_t  bestTarget = currentTargetIdx_;
    float bestTime = solver_->solve(drone_, targets_[bestTarget]).estimatedTime + penaltyChange;
    const float SWITCH_HYSTERESIS = 2.0f;
    for (const auto& [id, target] : targets_) {
        if (id == bestTarget) continue;
        DropPoint dp = solver_->solve(drone_, target);
        float t = dp.estimatedTime + penaltyChange;
        if (t < bestTime - SWITCH_HYSTERESIS) {
            bestTime = t;
            bestTarget = id;
        }
    }
    currentTargetIdx_ = bestTarget;
    DropPoint dp = solver_->solve(drone_, targets_[currentTargetIdx_]);
    auto nextState = currentState_->execute(drone_, dp, config_);
    if (nextState->name() != currentState_->name()) {
        LOG("State change: " << currentState_->name() << " -> " << nextState->name());
    }
    currentState_ = std::move(nextState);
    currentDroneState_ = drone_.currentState;
    Coord toFireDir = dp.firePos - drone_.currentPos;
    float tarDirection = std::atan2(toFireDir.y, toFireDir.x);
    float delta = norm_angle(tarDirection - drone_.currentDirection);

    float accel, turnRate;
    switch (drone_.currentState) {
        case TURNING:
        case STOPPED:
            accel    = -1.0f;
            turnRate = (delta > 0) ? 1.0f : -1.0f;
            break;
        case ACCELERATING:
            accel    = 1.0f;
            turnRate = 0.0f;
            break;
        case MOVING:
            accel    = 0.0f;
            turnRate = 0.0f;
            break;
        case DECELERATING:
            accel    = -1.0f;
            turnRate = 0.0f;
            break;
    }
    //LOG("Sending accel=" << accel << " turnRate=" << turnRate);
    uart_.sendControl(accel, turnRate);
    if (dp.isValid && drone_.currentState == MOVING) {
        Coord toFire = dp.firePos - drone_.currentPos;
        float distToFire = length(toFire);
        float ang_delta = norm_angle(std::atan2(toFire.y, toFire.x) - drone_.currentDirection);
        if (distToFire <= ammo_.hitRadius && std::abs(ang_delta) <= config_.turnThreshold && !dropped_) {
            dropped_ = true;
        LOG("DROP! target=" << currentTargetIdx_
                << " pos=(" << drone_.currentPos.x << "," << drone_.currentPos.y << ")"
                << " time=" << telemetry.t_ms);
            gpio_.pulseDrop();
        }
    }
    Coord dirVec = {std::cos(drone_.currentDirection), std::sin(drone_.currentDirection)};
    Coord aimPoint = drone_.currentPos + dirVec * dp.flightDistance;

    simSteps_.push_back({
        drone_.currentPos,
        drone_.currentDirection,
        static_cast<int>(drone_.currentState),
        static_cast<int>(currentTargetIdx_),
        dp.firePos,
        aimPoint,
        dp.predictedTarget,
        telemetry.t_ms
    });
}

void MissionProcessor::run() {
    
    //LOG("START signal sent, waiting for data...");
    uint8_t type, len;
    uint8_t payload[260];
    gpio_.setStart(1);
    while (!dropped_ && !stopFlag_.load()) {
        //LOG("Waiting for packet...");
        if (!uart_.readPacket(type, payload, len)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        //LOG("Packet received: type=0x" << std::hex << (int)type << " len=" << std::dec << (int)len);
        switch (type) {
            case PKT_AMMO: {
                AmmoCfg ammo;
                memcpy(&ammo, payload, sizeof(ammo));
                handleAmmo(ammo);
                break;
            }
            case PKT_CONFIG: {
                DroneCfg cfg;
                memcpy(&cfg, payload, sizeof(cfg));
                handleConfig(cfg);
                break;
            }
            case PKT_TARGET: {
                TargetPos t;
                memcpy(&t, payload, sizeof(t));
                handleTarget(t);
                break;
            }
            case PKT_TELEMETRY: {
                //LOG("Before handleTelemetry");
                Telemetry t;
                memcpy(&t, payload, sizeof(t));
                handleTelemetry(t);
                //LOG("After handleTelemetry");
                break;
            }
            default:
                //LOG("Unknown packet type=0x" << std::hex << (int)type);
                break;
        }
    }
    saveOutput();
    gpio_.setStart(0);
}

void MissionProcessor::saveOutput() const {
    json out;
    out["totalSteps"] = simSteps_.size();
    out["dropped"]    = dropped_;
    out["steps"]      = json::array();
    for (const auto& s : simSteps_) {
        json step;
        step["position"]         = {{"x", s.pos.x}, {"y", s.pos.y}};
        step["direction"]        = s.direction;
        step["state"]            = s.state;
        step["targetIdx"]        = s.targetIdx;
        step["dropPoint"]        = {{"x", s.dropPoint.x},  {"y", s.dropPoint.y}};
        step["aimPoint"]         = {{"x", s.aimPoint.x},   {"y", s.aimPoint.y}};
        step["predictedTarget"]  = {{"x", s.predictedTarget.x}, {"y", s.predictedTarget.y}};
        step["timeSecSinceStart"] = s.t_ms;
        out["steps"].push_back(step);
    }
    std::ofstream outFile("simulation_output.json");
    if (outFile) {
        outFile << out.dump(2);
        LOG("Simulation output saved to simulation_output.json");
    } else {
        LOG("Failed to save simulation output.");
    }
}

void MissionProcessor::tryCreateSolver() {
    if (configReady_ && ammoReady_) {
        solver_ = std::make_unique<TableSolver>(
            config_, ammo_, "data/ballistic_table.txt");
        LOG("Solver created");
        solverReady_ = true;
    }
}


        


