#include "ballistics.hpp"
#include <cstring>

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cerr << "usage: ballistics_cli <input_path>\n";
    return 1;
  }
  BallisticsInput input;
  if (!read_input(argv[1], input))
    return 1;

  DropSolution solution = compute_drop_solution(input);
  if (!solution.success)
    return 1;

  if (solution.need_middle) {
    std::cout << "Middle point: (" << solution.middle.x << ", " << solution.middle.y << ")\n";
  }
  std::cout << "Drop point: (" << solution.drop.x << ", " << solution.drop.y << ")\n";
  return 0;
}