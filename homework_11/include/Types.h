#pragma once

#include <cmath>

#include <string>
#include <cstdint>
#include "drone_link.h"

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
    Coord pos;
    Coord velocity;
    int   number;
};

struct DropPoint {
    int    targetNumber;
    Coord  firePos;
    Coord  predictedTarget;
    float  flightDistance;
    float  estimatedTime;
    bool   isValid;
};


float length(Coord c);

Coord normalize(Coord c);

/*struct AmmoParams {
    std::string name;
    float mass;
    float drag;
    float lift;
    float hitRadius;
};

struct DroneConfig {
    float altitude;
    float attackSpeed;
    float accelPath;
    float simTimeStep;
    float angularSpeed;
    float turnThreshold;
};*/

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
    uint32_t t_ms;
};

inline DronePos telemetryToPos(const dlink::Telemetry& t) {
    return {
        {t.x, t.y},
        t.speed,
        t.dir,
        t.z,
        STOPPED 
    };
}

inline float norm_angle(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

inline float stop_penalty(const DronePos& drone, const dlink::DroneCfg& cfg) {
    float acc = cfg.attackSpeed * cfg.attackSpeed / (2.0f * cfg.accelerationPath);
    switch (drone.currentState) {
        case STOPPED:      return 0.0f;
        case TURNING:      return 0.0f;
        case MOVING:       return cfg.attackSpeed / acc;
        case ACCELERATING: return drone.currentSpeed / acc;
        case DECELERATING: return drone.currentSpeed / acc;
    }
    return 0.0f;
}
