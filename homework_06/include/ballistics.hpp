#pragma once
#include <cmath>
#include <string>
#include <fstream>
#include <iostream>
#include <numbers>

struct BallisticsInput {
  float xd;
  float yd;
  float zd;
  float targetX;
  float targetY;
  float atackSpeed;
  float accelerationPath;
  std::string amo_name;
};

struct AmoParam {
  std::string name;
  float M;
  float D;
  float L;
};

struct point {
  float x;
  float y;
};

struct DropSolution {
  point drop;
  point middle;
  bool need_middle;
  bool success;
};

bool read_input(const char* path, BallisticsInput& input);
bool get_amo_param(std::string name, AmoParam& param);
float calculate_time_of_flight(float zd, float atackSpeed, const AmoParam& param);
float calculate_horizontal_distance(float timeOfFlight, float atackSpeed, const AmoParam& param);
inline float calculate_distance(float x1, float y1, float x2, float y2)
{
  return static_cast<float>(sqrt(pow(x2 - x1, 2.0f) + pow(y2 - y1, 2.0f)));
}
point calculate_midle_point(
  float xd, float yd, float targetX, float targetY, float horizontalDistance, float accelerationPath, float& distance, bool& need_midle);
point calculate_drop_point(float xd, float yd, float targetX, float targetY, float horizontalDistance, float distance);
DropSolution compute_drop_solution(const BallisticsInput& input);