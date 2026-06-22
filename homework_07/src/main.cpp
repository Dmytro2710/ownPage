#define ENABLE_LOG   1
#define ENABLE_DEBUG 0

#if ENABLE_LOG
#define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define DEBUG(msg)
#endif

#define _USE_MATH_DEFINES

#include <nlohmann/json.hpp>
#include <iostream>

#include <fstream>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <vector>
#include <string>
#include <array>
#include <map>

using json = nlohmann::json;

#define MAX_STEPS 10000

struct Coord {
    float x;
    float y;

    Coord operator+(const Coord& o)  const {return {x + o.x, y + o.y};  }
    Coord operator-(const Coord& o)  const {return {x - o.x, y - o.y};  }
    Coord operator*(float s)         const {return {x * s,   y * s};    }
    Coord operator/(float s)         const {return {x / s,   y / s};    }
    bool  operator==(const Coord& o) const {return x == o.x && y == o.y;}
};

struct Target {
    Coord targetCoord;
    float targetXSpeed;
    float targetYSpeed;
    int   targetNumber;
};

struct DropPoint {
    int    targetNumber;
    Coord  firePos;
    Coord  predictedTarget;
    float  flightDistance;
    float  estimatedTime;
    bool   isValid;
};

float length(Coord c)    {return std::hypot(c.x, c.y);}

Coord normalize(Coord c) {
    float len = length(c);
    if (len == 0.0f) return {0.0f, 0.0f};
    return c / len;
}

struct AmmoParams {
    std::string name;
    float mass;
    float drag;
    float lift;
};

struct DroneConfig {
    Coord startPos;
    float altitude;
    float initialDir;
    float attackSpeed;
    float accelPath;
    std::string ammoName;
    float arrayTimeStep;
    float simTimeStep;
    float hitRadius;
    float angularSpeed;
    float turnThreshold;
};

enum DroneState { 
    STOPPED      = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING      = 3,
    MOVING       = 4 
};

struct DronePos {
    Coord       currentPos;
    float       currentSpeed;
    float       currentDirection;
    float       currentAltitude;
    DroneState  currentState;
};

struct SimStep {
    Coord pos;
    float direction;
    int   state;
    int   targetIdx;
    Coord dropPoint;
    Coord aimPoint;
    Coord predictedTarget;
};


class ITargetProvider {
public:
    virtual int    getTargetCount() = 0;
    virtual Target getTarget(int index) = 0;
    virtual void   setCurrentStep(int step) = 0;
    virtual ~ITargetProvider() {}
};

class IBallisticSolver {
public:
    virtual DropPoint solve(DronePos drone, Target curTarget) = 0;
    virtual ~IBallisticSolver() {}
};

class IConfigLoader {
    public:
    virtual bool load() = 0;
    virtual AmmoParams getAmoParams() = 0;
    virtual DroneConfig getConfig() = 0;
    virtual ~IConfigLoader() {}
};

class JsonTargetProvider : public ITargetProvider {
    std::vector<std::vector<Coord>> targets;
    int     targetCount;
    int     timeSteps;
    float   arrayTimeStep;
    float   simTimeStep;
    bool    loadingFail;
    int     currentStep = 0;
public:
    JsonTargetProvider(const char* filename, float simTimeStep, float arrayTimeStep) 
        : arrayTimeStep(arrayTimeStep), simTimeStep(simTimeStep) {
        std::ifstream f(filename);
        std::cout << "Opening: " << filename << " -> " 
              << (f ? "OK" : "FAILED") << std::endl;
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
    int    getTargetCount() override {return targetCount;};
    Target getTarget(int index) override {
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
        return {{xInterpol, yInterpol}, xSpeed, ySpeed, index};
    };
    void   setCurrentStep(int step) override {currentStep = step;};
    //bool   getLoadingStatus();
    ~JsonTargetProvider() {
        targets.clear();
    };
};

class AnalyticalSolver : public IBallisticSolver {
    //DropPoint         solvedPont;
    const DroneConfig currentDrone;
    const AmmoParams  currentAmo;
    //float             timeToDrop;
    //float             currentTime;
    //char      errorReport[128];
    //bool      errorStatus;
public:
    AnalyticalSolver(DroneConfig currentDrone, AmmoParams  currentAmo/*, float currentTime*/)
        : currentDrone(currentDrone), currentAmo(currentAmo)/*,  currentTime(currentTime)*/ {}

    float ballistics(float& out_t) {
        const float G = 9.81f;
        float           d = currentAmo.drag;
        float           m = currentAmo.mass;
        float           l = currentAmo.lift;
        float attackSpeed = currentDrone.attackSpeed;
        float           z = currentDrone.altitude;
        float a = d * G * m - 2.0f * d * d * l * attackSpeed;
        float b = 3.0f * d * l * m * attackSpeed - 3.0f * G * m * m;
        float c = 6.0f * m * m * z;
        float p = (-1.0f) * b * b / (3.0f * a * a);
        float q =  2.0f * b * b *b / (27.0f * a * a * a) + c / a;
        if (p >= 0) return -1.0f;
        float arg = (3.0f * q / (2.0f * p)) * std::sqrt(-3.0f / p);
        if (arg < -1.0f || arg > 1.0f) return -1.0f;
        float fi = std::acos(arg);
        float t  = 2.0f * std::sqrt(-p / 3.0f) * std::cos((fi + 4.0f * (float)M_PI) / 3.0f) - b / (3.0f * a);
        if (t <= 0) return -1.0f;

        float l2=l * l, l4=l2 * l2, d2=d * d, d3=d2 * d, d4=d3 * d;
        float m2=m * m, m3=m2 * m, m4=m3 * m;
        float t2=t * t, t3=t2 * t, t4=t3 * t, t5=t4 * t;
        float L2p1 = 1.0f + l2;
        float V = attackSpeed;

        float h = V * t
            - t2 * d * V / (2.0f * m)
            + t3 * (6.0f * d * G * l * m - 6.0f * d2 * (l2 - 1.0f) * V) / (36.0f * m2)
            + t4 * (-6.0f * d2 * G * l * (1.0f + l2 + l4) * m + 3.0f * d3 * l2 * L2p1 * V + 6.0f * d3 * l4 * L2p1 * V)
                / (36.0f * L2p1 * L2p1 * m3)
            + t5 * (3.0f * d3 * G * l2 * l * m - 3.0f * d4 * l2 * L2p1 * V)
                /(36.0f*L2p1*m4);

        if (h <= 0) return -1.0f;
        //timeToDrop = t;
        out_t = t;
        return h;
    }
    Coord interpolate_target(float totalTime, Target curTarget) {
        Coord predictedPos;
        predictedPos.x = curTarget.targetCoord.x + totalTime * curTarget.targetXSpeed;
        predictedPos.y = curTarget.targetCoord.y + totalTime * curTarget.targetYSpeed;
        return predictedPos;
    }
    float stop_penalty(DronePos drone) {
        float acc = currentDrone.attackSpeed * currentDrone.attackSpeed / (2.0f * currentDrone.accelPath);
        switch (drone.currentState) {
            case STOPPED:      return 0.0f;
            case TURNING:      return 0.0f;
            case MOVING:       return currentDrone.attackSpeed / acc;
            case ACCELERATING: return drone.currentSpeed / acc;
            case DECELERATING: return drone.currentSpeed / acc;
        }
        return 0.0f;
    }
    float norm_angle(float a) {
        while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
        while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
        return a;
    }
    DropPoint solve(DronePos drone, Target curTarget) override {
        DropPoint result {};
        result.targetNumber = curTarget.targetNumber;
        float timeToDrop = 0.0f; 
        float h = ballistics(timeToDrop);
        if (h < 0) {
            result.isValid       = false;
            result.estimatedTime = 1e9f;
            return result;
        }

        float acc   = currentDrone.attackSpeed * currentDrone.attackSpeed / (2.0f * currentDrone.accelPath);
        float t_acc = currentDrone.attackSpeed / acc;

        float totalTime = 0.0f;
        Coord pred  = curTarget.targetCoord;
        Coord delta = pred - drone.currentPos;
        float D     = length(delta);

        const float CONV = currentDrone.simTimeStep * 0.1f;
        for (int i = 0; i < 10; ++i) {
            float prev = totalTime;
            pred  = interpolate_target(totalTime, curTarget);
            delta = pred - drone.currentPos;
            D     = length(delta);
            float driveD = (D - h > 0) ? (D - h) : 0;

            float t_drive = (driveD <= currentDrone.accelPath)
                ? std::sqrt(2.0f * driveD / acc)
                : t_acc + (driveD - currentDrone.accelPath) / currentDrone.attackSpeed;

            float ang    = norm_angle(std::atan2(delta.y, delta.x) - drone.currentDirection);
            float t_turn = (std::abs(ang) > currentDrone.turnThreshold)
                ? std::abs(ang) / currentDrone.angularSpeed + 2.0f * t_acc
                : 0.0f;

            totalTime = stop_penalty(drone) + t_turn + t_drive;
            if (std::abs(totalTime - prev) < CONV) break;
        }

        result.estimatedTime = totalTime;
        result.isValid       = true;
        result.predictedTarget = pred;
        result.firePos.x     = pred.x + (drone.currentPos.x - pred.x) * h / D;
        result.firePos.y     = pred.y + (drone.currentPos.y - pred.y) * h / D;
        result.flightDistance = h;
        return result;
    }
    //char*     getErrorReport();
    //bool      getErrorStatus();
    ~AnalyticalSolver() {};
};

class FileConfigLoader : public IConfigLoader {
    AmmoParams  currentAmo;
    DroneConfig currentDrone;
    int         ammoCount = 0;
    const char* fnameAmmo;
    const char* fnameDrone;
public:
    FileConfigLoader(const char* fnameAmmo, const char* fnameDrone)
        : fnameAmmo(fnameAmmo), fnameDrone(fnameDrone) {}
    bool load() override {
        std::ifstream f1(fnameAmmo);
        std::ifstream f2(fnameDrone);
        if (!(f1 && f2)) {
            //AmoCount = 0;
            return false;
        }
        json j;
        f1 >> j;

        ammoCount = static_cast<int>(j.size());
        /*std::vector<AmmoParams> ammoList;
        ammoList.reserve(ammoCount);
        for (const auto& ammo : j) {
            ammoList.emplace_back();
            ammoList.back().name = ammo["name"].get<std::string>();
            //std::strncpy(ammoList[i].name, name.c_str(), 31);
            ammoList.back().mass = ammo["mass"];
            ammoList.back().drag = ammo["drag"];
            ammoList.back().lift = ammo["lift"];
        }*/
        std::map<std::string, AmmoParams> ammoMap;
        for (const auto& ammo : j) {
            AmmoParams params;
            params.name = ammo["name"].get<std::string>();
            params.mass = ammo["mass"];
            params.drag = ammo["drag"];
            params.lift = ammo["lift"];
            ammoMap[params.name] = params;
        }
        json jDrone;
        f2 >> jDrone;

        currentDrone.startPos.x    = jDrone["drone"]["position"]["x"];
        currentDrone.startPos.y    = jDrone["drone"]["position"]["y"];
        currentDrone.altitude      = jDrone["drone"]["altitude"];
        currentDrone.initialDir    = jDrone["drone"]["initialDirection"];
        currentDrone.attackSpeed   = jDrone["drone"]["attackSpeed"];
        currentDrone.accelPath     = jDrone["drone"]["accelerationPath"];
        currentDrone.angularSpeed  = jDrone["drone"]["angularSpeed"];
        currentDrone.turnThreshold = jDrone["drone"]["turnThreshold"];
        currentDrone.simTimeStep   = jDrone["simulation"]["timeStep"];
        currentDrone.hitRadius     = jDrone["simulation"]["hitRadius"];
        currentDrone.arrayTimeStep = jDrone["targetArrayTimeStep"];

        currentDrone.ammoName = jDrone["ammo"].get<std::string>();
        //std::strncpy(currentDrone.ammoName, ammoStr.c_str(), 31);
        //currentDrone.ammoName[31] = '\0';
        auto it = ammoMap.find(currentDrone.ammoName);
        if (it != ammoMap.end()) {
            currentAmo = it->second;
            return true;
        }

        /*for (const auto& ammo : ammoList)
            if (currentDrone.ammoName == ammo.name) {
                currentAmo = ammo;
                return true;
            }*/
        return false;
    };
    AmmoParams  getAmoParams() override {
        return currentAmo;
    };
    DroneConfig getConfig() override {
        return currentDrone;
    };
};

enum class SolverType   { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType   { FILE_LOADER };

class Factory {
public:
    IBallisticSolver* createSolver(SolverType type, 
                                DroneConfig cfg, AmmoParams ammo) {
        switch (type) {
            case SolverType::ANALYTICAL: 
                return new AnalyticalSolver(cfg, ammo);
            default: return nullptr;
        }
        return nullptr;
    }
    ITargetProvider* createProvider(ProviderType type, const char* filename,
                                 float simTS, float arrayTS) {
        switch (type) {
            case ProviderType::JSON: 
                return new JsonTargetProvider(filename, simTS, arrayTS);
            default: return nullptr;
        }
        return nullptr;
    }
    IConfigLoader* createLoader(LoaderType type) {
        switch (type) {
            case LoaderType::FILE_LOADER : 
                return new FileConfigLoader("ammo.json", "config.json");
            default: return nullptr;
        }
        return nullptr;
    }
};

class MissionProcessor {
    IBallisticSolver* solver;
    ITargetProvider*  provider;
    IConfigLoader*    loader;
    DroneConfig       config;
    AmmoParams        ammo;
    DronePos          drone;
    int               currentTargetIdx;
public:
    MissionProcessor(IBallisticSolver* s, ITargetProvider* p, IConfigLoader* l)
    : solver(s), provider(p), loader(l) {}
    void init() {
        //loader->load();
        config = loader->getConfig();
        ammo   = loader->getAmoParams();
        drone  = {{config.startPos.x, config.startPos.y}, 
                0.0f, 
                config.initialDir, 
                config.altitude, 
                STOPPED};
        currentTargetIdx = 0;
    }
    bool hasNext() {return currentTargetIdx < provider->getTargetCount();};
    DropPoint step() {
        Target curentTarget = provider->getTarget(currentTargetIdx);
        DropPoint curTarDropPoint = solver->solve(drone, curentTarget);
        ++currentTargetIdx;
        return curTarDropPoint;
    };
    void reset() {currentTargetIdx = 0;};
    void changeSolver(IBallisticSolver* s) {solver = s;};
    void simulate() {
        SimStep* simSteps = new SimStep[MAX_STEPS];
        //float h_dummy = 0.0f;
        bool dropped = false;
        int totalSteps = 0;
        float t_now = 0.0f;
        //float acc = config.attackSpeed * config.attackSpeed / (2.0f * config.accelPath);
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
private:
    float norm_angle_local(float a) {
        while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
        while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
        return a;
    }
    float stop_penalty_local(DronePos drone) {
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
    void update_drone(DropPoint dp, Target curTarget) {
        float acc = config.attackSpeed * config.attackSpeed / (2.0f * config.accelPath);
        Coord toTarget = dp.firePos - drone.currentPos;
        float neededDir = std::atan2(toTarget.y, toTarget.x);
        float delta = norm_angle_local(neededDir - drone.currentDirection);
        //float distToFire = dp.isValid ? length(dp.firePos - drone.currentPos) : length(toTarget);

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
};


int main() {
    Factory newFactory;
    
    auto loader = newFactory.createLoader(LoaderType::FILE_LOADER);

    if (!loader->load()) {
        LOG("ERROR: load() failed");
        delete loader;
        return 1;
    }
    LOG("Config loaded OK");
    LOG("attackSpeed=" << loader->getConfig().attackSpeed);
    LOG("ammo=" << loader->getConfig().ammoName);

    auto provider = newFactory.createProvider(ProviderType::JSON, "targets.json",
        loader->getConfig().simTimeStep,
        loader->getConfig().arrayTimeStep);

    LOG("targetCount=" << provider->getTargetCount());

    LOG("targetCount=" << provider->getTargetCount());

    auto solver = newFactory.createSolver(SolverType::ANALYTICAL,
        loader->getConfig(),
        loader->getAmoParams());

    MissionProcessor* droneMission = new MissionProcessor(solver, provider, loader);
    droneMission->init();
    droneMission->simulate();

    /*LOG("hasNext=" << droneMission->hasNext());

    while (droneMission->hasNext()) {
        DropPoint result = droneMission->step();
        if (result.isValid)
            LOG("Target " << result.targetNumber
                << " firePos=(" << result.firePos.x << ", " << result.firePos.y << ")"
                << " time=" << result.estimatedTime);
        else
            LOG("Target " << result.targetNumber << " ballistics failed");
    }*/
    
    delete droneMission;
    delete solver;
    delete provider;
    delete loader;
    return 0;
}