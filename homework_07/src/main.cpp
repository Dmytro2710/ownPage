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
    char  name[32];
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
    char  ammoName[32];
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
    Coord** targets = nullptr;
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
            targets   = nullptr;
            targetCount = 0;
            timeSteps = 0;
            loadingFail = true;
            return;
        }
        json j;
        f >> j;
        targetCount = j["targetCount"];
        timeSteps   = j["timeSteps"];
        targets = new Coord*[targetCount];
        for (int i = 0; i < targetCount; ++i) {
            targets[i] = new Coord[timeSteps];

            for (int k = 0; k < timeSteps; ++k) {
                targets[i][k].x = j["targets"][i]["positions"][k]["x"];
                targets[i][k].y = j["targets"][i]["positions"][k]["y"];
            }
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
        if (targets == nullptr) return;
        for (int i = 0; i < targetCount; ++i) {
            delete[] targets[i];
            targets[i] = nullptr;
        }
        delete[] targets;
        targets = nullptr;
    };
};

class AnalyticalSolver : public IBallisticSolver {
    //DropPoint         solvedPont;
    const DroneConfig currentDrone;
    const AmmoParams  currentAmo;
    float             timeToDrop;
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
        result.firePos.x     = pred.x + (drone.currentPos.x - pred.x) * h / D;
        result.firePos.y     = pred.y + (drone.currentPos.y - pred.y) * h / D;
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
        AmmoParams* ammoList = new AmmoParams[ammoCount];
        for (int i = 0; i < ammoCount; ++i) {
            std::string name = j[i]["name"].get<std::string>();
            std::strncpy(ammoList[i].name, name.c_str(), 31);
            ammoList[i].name[31] = '\0';
            ammoList[i].mass = j[i]["mass"];
            ammoList[i].drag = j[i]["drag"];
            ammoList[i].lift = j[i]["lift"];
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

        std::string ammoStr = jDrone["ammo"].get<std::string>();
        std::strncpy(currentDrone.ammoName, ammoStr.c_str(), 31);
        currentDrone.ammoName[31] = '\0';
         for (int i = 0; i < ammoCount; ++i)
            if (strcmp(currentDrone.ammoName, ammoList[i].name) == 0) {
                currentAmo = ammoList[i];
                delete[] ammoList;
                return true;
            }
        delete[] ammoList;
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

    LOG("hasNext=" << droneMission->hasNext());
    
    while (droneMission->hasNext()) {
        DropPoint result = droneMission->step();
        if (result.isValid)
            LOG("Target " << result.targetNumber
                << " firePos=(" << result.firePos.x << ", " << result.firePos.y << ")"
                << " time=" << result.estimatedTime);
        else
            LOG("Target " << result.targetNumber << " ballistics failed");
    }
    
    delete droneMission;
    delete solver;
    delete provider;
    delete loader;
    return 0;
}