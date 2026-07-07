#pragma once
#include "Types.h"
#include "interfaces/IBallisticSolver.h"
#include <memory>
#include <map>
#include <vector>
#include "GpioController.h"
#include "UartLink.h"
#include "states/BasicState.h"
#include "drone_link.h"
#include <atomic>

class MissionProcessor {
        UartLink&                           uart_;
        GpioController&                     gpio_;
        std::unique_ptr<IBallisticSolver>   solver_;
        
        dlink::DroneCfg                 config_;
        dlink::AmmoCfg                  ammo_;
        DronePos                        drone_; 
        std::map<uint8_t, Target>       targets_;
        uint8_t                         currentTargetIdx_ = 0;
        bool                            dropped_ = false;       
        std::unique_ptr<IDroneState>    currentState_;
        std::vector<SimStep>            simSteps_;
        bool                            configReady_ = false;
        bool                            ammoReady_   = false;
        bool                            solverReady_ = false;
        std::atomic<bool>&              stopFlag_;    
        DroneState                      currentDroneState_ = STOPPED;
    public:
        MissionProcessor(UartLink& uart, GpioController& gpio, std::atomic<bool>& stopFlag);
        void run();
        void handleTelemetry(const dlink::Telemetry& telemetry);
        void handleTarget(const dlink::TargetPos& target);
        void handleAmmo(const dlink::AmmoCfg& ammo);
        void handleConfig(const dlink::DroneCfg& config);
        void saveOutput() const;
        void tryCreateSolver();
    };