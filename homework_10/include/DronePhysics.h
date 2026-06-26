#pragma once
#include "Types.h"
#include <atomic>
#include <mutex>

class DronePhysics {
public:
    DronePhysics(DroneConfig cfg);

    void           step(float dt);
    void           sendCommand(DroneCommand cmd);
    DroneTelemetry getTelemetry() const;
    float          stop_penalty() const;
    void           run();
    void           start();
    void           stop();
    bool           isThreadReady() const;
    

private:
    DroneConfig    config;
    DroneCommand   pendingCmd;
    DroneTelemetry telemetry;
    float          currentAltitude;
    float          currentSpeed     = 0.0f;
    float          currentDirection = 0.0f;
    DroneState     currentState     = STOPPED;
    std::atomic<bool> stop_flag{false};
    std::atomic<bool> start_flag{false};
    std::atomic<bool> is_ready{false};
    mutable std::mutex mtx_;

    void  update_drone(float dt);
};