#pragma once

#include <cmath>

#include <string>

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

float length(Coord c);

Coord normalize(Coord c);

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
