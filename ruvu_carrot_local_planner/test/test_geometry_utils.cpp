// Copyright 2020 RUVU BV.

#include <gtest/gtest.h>

#include "../src/geometry_utils.hpp"

TEST(TestGeometryUtils, distance2)
{
    geometry_msgs::PoseStamped p1, p2;
    p2.pose.position.x = 3;
    p2.pose.position.y = 4;
    ASSERT_DOUBLE_EQ(ruvu_carrot_local_planner::distance2(p1, p2), 25.0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
