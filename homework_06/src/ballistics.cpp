#include "ballistics.hpp"

const AmoParam AmoParams[] = {{"VOG-17", 0.35f, 0.07f, 0.0f},
                              {"M67", 0.6f, 0.10f, 0.0f},
                              {"RKG-3", 1.2f, 0.10f, 0.0f},
                              {"GLIDING-VOG", 0.45f, 0.10f, 0.1f},
                              {"GLIDING-RKG", 1.4f, 0.10f, 0.1f}};

bool get_amo_param(std::string name, AmoParam& param)
{
  for (const auto& amo : AmoParams) {
    if (amo.name == name) {
      param = amo;
      return true;
    }
  }
  return false;
}

float calculate_time_of_flight(float zd, float atackSpeed, const AmoParam& param)
{
  const float g = 9.8f;
  float a = param.D * g * param.M - 2.0f * powf(param.D, 2.0f) * param.L * atackSpeed;
  float b = 3.0f * param.D * param.L * param.M * atackSpeed - 3.0f * g * powf(param.M, 2.0f);
  float c = 6.0f * powf(param.M, 2.0f) * zd;
  float p = (-1) * powf(b, 2.0f) / (3 * powf(a, 2.0f));
  float q = 2 * powf(b, 3.0f) / (27.0f * powf(a, 3.0f)) + c / a;
  float arg = (3.0f * q / (2.0f * p)) * sqrtf(-3.0f / p);
  if (arg < -1 || arg > 1) {
    std::cerr << "Arccos arg is out of range: " << arg << ". Error! Finishing program\n";
    return -1;
  }
  float fi = acosf(arg);
  float t = 2.0F * sqrtf(p / -3.0F) * cosf((fi + 4.0F * std::numbers::pi_v<float>) / 3.0F) - b / (3.0F * a);
  if (t < 0) {
    std::cerr << "Negative time of flight. Error! Finishing program\n";
    return -1;
  }
  return t;
}

float calculate_horizontal_distance(float timeOfFlight, float atackSpeed, const AmoParam& param)
{
  float t = timeOfFlight;
  float h = atackSpeed * t - powf(t, 2.0f) * param.D * atackSpeed / (2.0f * param.M) +
            powf(t, 3.0f) *
              (6.0f * param.D * 9.8f * param.L * param.M - 6 * powf(param.D, 2.0f) * (powf(param.L, 2.0f) - 1.0f) * atackSpeed) /
              (36.0f * powf(param.M, 2.0f)) +
            powf(t, 4.0f) *
              (-6.0f * powf(param.D, 2.0f) * 9.8f * param.L * (1.0f + powf(param.L, 2.0f) + powf(param.L, 4.0f)) * param.M +
               3.0f * powf(param.D, 3.0f) * powf(param.L, 2.0f) * (1 + powf(param.L, 2.0f)) * atackSpeed +
               6.0f * powf(param.D, 3.0f) * powf(param.L, 4.0f) * (1 + powf(param.L, 2.0f)) * atackSpeed) /
              (36.0f * powf(param.M, 3.0f) * powf((1 + powf(param.L, 2.0f)), 2.0f)) +
            powf(t, 5.0f) *
              (3.0f * powf(param.D, 3.0f) * 9.8f * powf(param.L, 3.0f) * param.M -
               3.0f * powf(param.D, 4.0f) * powf(param.L, 2.0f) * (1 + powf(param.L, 2.0f)) * atackSpeed) /
              (36.0f * powf(param.M, 4.0f) * (1 + powf(param.L, 2.0f)));
  if (h < 0) {
    std::cerr << "Negative horizontal distance. Error! Finishing program\n";
    return -1;
  }
  return h;
}

point calculate_midle_point(
  float xd, float yd, float targetX, float targetY, float horizontalDistance, float accelerationPath, float& distance, bool& need_midle)
{
  point mp = {0.0f, 0.0f};
  if (horizontalDistance + accelerationPath > distance) {
    need_midle = true;
    mp.x = targetX - (targetX - xd) * (horizontalDistance + accelerationPath) / distance;
    mp.y = targetY - (targetY - yd) * (horizontalDistance + accelerationPath) / distance;
    distance = calculate_distance(mp.x, mp.y, targetX, targetY);
    return mp;
  }
  need_midle = false;
  return mp;
}

point calculate_drop_point(float xd, float yd, float targetX, float targetY, float horizontalDistance, float distance)
{
  point dp = {0.0f, 0.0f};
  float ratio = (distance - horizontalDistance) / distance;
  dp.x = xd + (targetX - xd) * ratio;
  dp.y = yd + (targetY - yd) * ratio;
  return dp;
}

bool read_input(const char* path, BallisticsInput& input)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    std::cerr << "Error opening file: " << path << "\n";
    return false;
  }
  file >> input.xd >> input.yd >> input.zd;
  if (input.zd <= 0) {
    std::cerr << "Invalid zd value: " << input.zd << "\n";
    return false;
  }
  file >> input.targetX >> input.targetY;
  file >> input.atackSpeed;
  if (input.atackSpeed <= 0) {
    std::cerr << "Invalid atackSpeed value: " << input.atackSpeed << "\n";
    return false;
  }
  file >> input.accelerationPath;
  if (input.accelerationPath <= 0) {
    std::cerr << "Invalid accelerationPath value: " << input.accelerationPath << "\n";
    return false;
  }
  file >> input.amo_name;
  file.close();
  std::cout << "Input data: xd=" << input.xd << ", yd=" << input.yd << ", zd=" << input.zd << ", targetX=" << input.targetX
            << ", targetY=" << input.targetY << ", atackSpeed=" << input.atackSpeed << ", accelerationPath=" << input.accelerationPath
            << ", amoName=" << input.amo_name << "\n";
  return true;
}

DropSolution compute_drop_solution(const BallisticsInput& input)
{
  DropSolution solution{};
  solution.success = false;

  AmoParam param{};
  if (!get_amo_param(input.amo_name, param)) {
    std::cerr << "Ammunition type not found: " << input.amo_name << '\n';
    return solution;
  }

  float timeOfFlight = calculate_time_of_flight(input.zd, input.atackSpeed, param);
  if (timeOfFlight < 0)
    return solution;

  float horizontalDistance = calculate_horizontal_distance(timeOfFlight, input.atackSpeed, param);
  if (horizontalDistance < 0)
    return solution;

  float distance = calculate_distance(input.xd, input.yd, input.targetX, input.targetY);
  if (distance <= 0) {
    std::cerr << "Zero distance to target\n";
    return solution;
  }

  solution.middle = calculate_midle_point(
    input.xd, input.yd, input.targetX, input.targetY, horizontalDistance, input.accelerationPath, distance, solution.need_middle);
  solution.drop = calculate_drop_point(input.xd, input.yd, input.targetX, input.targetY, horizontalDistance, distance);
  solution.success = true;
  return solution;
}
