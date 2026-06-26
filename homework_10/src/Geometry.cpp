#include "Types.h"
#include <cmath>

float length(Coord c) {
    return std::hypot(c.x, c.y);
}

Coord normalize(Coord c) {
    float len = length(c);
    if (len == 0.0f) return {0.0f, 0.0f};
    return c / len;
}