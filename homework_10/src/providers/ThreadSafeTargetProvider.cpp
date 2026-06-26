#include "providers/ThreadSafeTargetProvider.h"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include "Logger.h"
#include "Types.h"
#include <chrono>
#include <thread>

using json = nlohmann::json;

ThreadSafeTargetProvider::ThreadSafeTargetProvider(const char* filename, float simTimeStep, float arrayTimeStep, float timeScale)
    : arrayTimeStep(arrayTimeStep), simTimeStep(simTimeStep), timeScale(timeScale) {
    std::ifstream f(filename);
    DEBUG("Opening: " << filename << " -> " << (f ? "OK" : "FAILED"));
    if (!f) {
        targets   = std::vector<std::vector<Coord>>();
        targetCount = 0;
        timeSteps = 0;
        return;
    }
    json j;
    f >> j;
    targetCount = j["targetCount"];
    timeSteps   = j["timeSteps"];
    targets.clear();
    targets.reserve(targetCount);
    for (const auto& target : j["targets"]) {
        std::vector<Coord> row;
        row.reserve(timeSteps);
        for (const auto& pos : target["positions"]) {
            row.push_back(Coord{pos["x"], pos["y"]});
        }
        targets.push_back(row);
    }
}

void ThreadSafeTargetProvider::run() {
    int tagetCounter = 0;
    mtx_.lock();
    for (auto& target: targets) {
        currentTargets_.push_back({target[0], {0.0f, 0.0f}, tagetCounter++});
    }
    mtx_.unlock();
    is_ready.store(true);
    LOG("Provider ready");
       
    while (!start_flag.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    auto nextStep = std::chrono::steady_clock::now();
    auto startSim = nextStep;

    //int sleepMs = static_cast<int>((timeScale != 0) ? ((arrayTimeStep / timeScale) * 1000.0f): arrayTimeStep * 1000.0f);
    auto stepDuration = std::chrono::duration<float>(arrayTimeStep / timeScale);
    std::chrono::duration<float> aStep(arrayTimeStep);

    while (!stop_flag.load()) {
        auto wakeTime = std::chrono::steady_clock::now() + stepDuration;
        auto timePass = std::chrono::duration<float>(std::chrono::steady_clock::now() - startSim).count();
        int totalStep = static_cast<int>(timePass / arrayTimeStep);
        int currentIdx = totalStep % timeSteps;
        int prevIdx    = (currentIdx == 0) ? (timeSteps - 1) : (currentIdx - 1);
        float timeShift = timePass - arrayTimeStep * totalStep;

        {
            std::lock_guard<std::mutex> lk(mtx_);
            for (int i = 0; i < targetCount; ++i) {
                if (totalStep) {
                    Coord vel = {
                        (targets[i][currentIdx].x - targets[i][prevIdx].x) / arrayTimeStep,
                        (targets[i][currentIdx].y - targets[i][prevIdx].y) / arrayTimeStep
                    };
                    {currentTargets_[i].velocity = vel;}
                    currentTargets_[i].pos = {
                        targets[i][currentIdx].x + timeShift * vel.x,
                        targets[i][currentIdx].y + timeShift * vel.y
                    };
                }
            }
        }
        std::this_thread::sleep_until(wakeTime);
    }
}

void ThreadSafeTargetProvider::setCurrentStep(int) {}

int ThreadSafeTargetProvider::getTargetCount() {
    return targetCount;
}

Target ThreadSafeTargetProvider::getTarget(int index) {
    std::lock_guard<std::mutex> lk(mtx_);
    return currentTargets_[index];
}

bool ThreadSafeTargetProvider::isThreadReady() const {
    return is_ready.load();
}

void ThreadSafeTargetProvider::start() {
    start_flag.store(true);
}

void ThreadSafeTargetProvider::stop() {
    stop_flag.store(true);
}

ThreadSafeTargetProvider::~ThreadSafeTargetProvider() {}