#include "ballistics.hpp"
#include <gtest/gtest.h>

TEST(Ballistics, ComputesKnownDropPoint)
{
  BallisticsInput input{};
  input.xd = 100.0f;
  input.yd = 100.0f;
  input.zd = 100.0f;
  input.targetX = 200.0f;
  input.targetY = 200.0f;
  input.atackSpeed = 10.0f;
  input.accelerationPath = 10.0f;
  input.amo_name = "VOG-17";
  DropSolution solution = compute_drop_solution(input);
  EXPECT_TRUE(solution.success);
  EXPECT_NEAR(solution.drop.x, 173.759f, 0.1f);
  EXPECT_NEAR(solution.drop.y, 173.759f, 0.1f);
}

TEST(Ballistics, UnknownAmmoReturnsFalse)
{
  AmoParam param;
  const bool result = get_amo_param("UNKNOWN-AMMO", param);
  EXPECT_FALSE(result);
}