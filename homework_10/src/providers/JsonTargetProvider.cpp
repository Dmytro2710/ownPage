
#include "providers/JsonTargetProvider.h"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include "Logger.h"

using json = nlohmann::json;

JsonTargetProvider::JsonTargetProvider(const char* filename, float simTimeStep, float arrayTimeStep) 
    : arrayTimeStep(arrayTimeStep), simTimeStep(simTimeStep) {
        std::ifstream f(filename);
    DEBUG("Opening: " << filename << " -> " << (f ? "OK" : "FAILED"));
    if (!f) {
        targets   = std::vector<std::vector<Coord>>();
        targetCount = 0;
        timeSteps = 0;
        loadingFail = true;
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
    loadingFail = false;
};

int JsonTargetProvider::getTargetCount() {return targetCount;}

Target JsonTargetProvider::getTarget(int index) {
    float currentTime = currentStep * simTimeStep;
    int lastStep = static_cast<int>((currentTime) / arrayTimeStep);
    int lastIndex = lastStep % 60;
    int prevIndex = lastIndex == 0 ? (timeSteps - 1) : (lastIndex - 1);
    float xSpeed, ySpeed;
    if (currentTime < arrayTimeStep) {xSpeed = 0.0f; ySpeed = 0.0f;}
    else {
        xSpeed = (targets[index][lastIndex].x - targets[index][prevIndex].x) / arrayTimeStep;
        ySpeed = (targets[index][lastIndex].y - targets[index][prevIndex].y) / arrayTimeStep;
    }
    float xInterpol = targets[index][lastIndex].x + xSpeed * (currentTime - lastStep * arrayTimeStep);
    float yInterpol = targets[index][lastIndex].y + ySpeed * (currentTime - lastStep * arrayTimeStep);
    return {{xInterpol, yInterpol}, {xSpeed, ySpeed}, index};
};

void JsonTargetProvider::setCurrentStep(int step) {currentStep = step;};

JsonTargetProvider::~JsonTargetProvider() {
    // targets.clear();
};