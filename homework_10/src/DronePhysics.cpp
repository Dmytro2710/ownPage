#define _USE_MATH_DEFINES
#include "DronePhysics.h"
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>
#include "Logger.h"

DronePhysics::DronePhysics(DroneConfig cfg)
    : config(cfg),
      pendingCmd{STOPPED, 0.0f},
      telemetry{cfg.startPos, {0.0f, 0.0f}, 0.0f, STOPPED, 0.0f},
      currentAltitude(cfg.altitude),
      currentSpeed(0.0f),
      currentDirection(cfg.initialDir),
      currentState(STOPPED)
{}

DroneTelemetry DronePhysics::getTelemetry() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return telemetry;
}

void DronePhysics::sendCommand(DroneCommand cmd) {
    std::lock_guard<std::mutex> lk(mtx_);
    pendingCmd = cmd;
}

float DronePhysics::stop_penalty() const {
    float attackSpeed = config.attackSpeed;
    float acc = attackSpeed * attackSpeed / (2.0f * config.accelPath);
    
    float speed;
    DroneState state;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        speed = currentSpeed;
        state = currentState;
    }
    switch (state) {
        case STOPPED:      return 0.0f;
        case TURNING:      return 0.0f;
        case MOVING:       return attackSpeed / acc;
        case ACCELERATING: return speed / acc;
        case DECELERATING: return speed / acc;
    }
    return 0.0f;
}

void DronePhysics::step(float dt) {

    DroneCommand cmd;
    float currentSpeedCopy;
    float currentDirectionCopy;
    DroneState currentStateCopy;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        cmd                  = pendingCmd;
        currentSpeedCopy     = currentSpeed;
        currentDirectionCopy = currentDirection;
        currentStateCopy     = currentState;
    }

    float acc = config.attackSpeed * config.attackSpeed / (2.0f * config.accelPath);
    float delta = norm_angle(cmd.targetDirection - currentDirectionCopy);

    switch (cmd.desiredState) {
        case STOPPED:
            currentSpeedCopy = 0.0f;
            currentStateCopy = STOPPED;
            break;
        case TURNING:
            currentStateCopy = TURNING;
            if (std::abs(delta) <= config.turnThreshold) {
                currentDirectionCopy = cmd.targetDirection;
            } else {
                float maxTurn = config.angularSpeed * dt;
                currentDirectionCopy = norm_angle(currentDirectionCopy +
                    (delta > 0 ? 1.0f : -1.0f) *
                    std::min(maxTurn, std::abs(delta)));
            }
            break;
        case ACCELERATING:
            currentStateCopy = ACCELERATING;
            currentSpeedCopy += acc * dt;
            if (currentSpeedCopy >= config.attackSpeed)
                currentSpeedCopy = config.attackSpeed;
            break;
        case MOVING:
            currentStateCopy = MOVING;
            currentSpeedCopy = config.attackSpeed;
            break;
        case DECELERATING:
            currentStateCopy = DECELERATING;
            currentSpeedCopy -= acc * dt;
            if (currentSpeedCopy < 0.0f)
                currentSpeedCopy = 0.0f;
            break;
    }

    Coord dirVec = {std::cos(currentDirectionCopy), std::sin(currentDirectionCopy)};
    {
        std::lock_guard<std::mutex> lk(mtx_);
        currentSpeed     = currentSpeedCopy;
        currentDirection = currentDirectionCopy;
        currentState     = currentStateCopy;
        telemetry.currentDirection   = currentDirectionCopy;
        telemetry.pos          = telemetry.pos + dirVec * (currentSpeedCopy * dt);
        telemetry.speed        = dirVec * currentSpeedCopy;
        telemetry.currentState = currentStateCopy;
        telemetry.timeSecSinceStart += dt;
    }
}

void DronePhysics::run() {
    is_ready.store(true);
    LOG("DronePhysics ready");
    while (!start_flag.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    auto stepDuration = std::chrono::duration<float>(config.physicsTimeStep / config.timeScale);
    //auto stepDuration = std::chrono::duration<float>(config.physicsTimeStep);
    LOG("physicsTimeStep=" << config.physicsTimeStep 
        << " timeScale=" << config.timeScale
        << " calc=" << (config.physicsTimeStep / config.timeScale * 1000.0f)
        << " sleepMs=" << stepDuration);
    while (!stop_flag.load()) {
        DEBUG("Physics starting, current time=" << telemetry.timeSecSinceStart);
        auto wakeTime = std::chrono::steady_clock::now() + stepDuration;
        DEBUG("Physics step, speed=" << currentSpeed << " state=" << currentState);
        step(config.physicsTimeStep);
        std::this_thread::sleep_until(wakeTime);
    }

}

bool DronePhysics::isThreadReady() const { return is_ready.load(); }
void DronePhysics::start() { start_flag.store(true); }
void DronePhysics::stop()  { stop_flag.store(true);  }

void DronePhysics::update_drone(float dt) {
    Coord dirVec = {std::cos(currentDirection), std::sin(currentDirection)};
    std::lock_guard<std::mutex> lk(mtx_);
    telemetry.pos          = telemetry.pos + dirVec * (currentSpeed * dt);
    telemetry.speed        = dirVec * currentSpeed;
    telemetry.currentState = currentState;
    telemetry.timeSecSinceStart += dt;
}