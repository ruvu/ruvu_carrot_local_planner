#include <gtest/gtest.h>

#include "../src/utils.h"

TEST(TestSgn, minus1)
{
    ASSERT_DOUBLE_EQ(ruvu_carrot_local_planner::sgn(-1.0), -1);
}

TEST(TestSgn, 0)
{
    ASSERT_DOUBLE_EQ(ruvu_carrot_local_planner::sgn(0.0), 1);
}

TEST(TestSgn, 1)
{
    ASSERT_DOUBLE_EQ(ruvu_carrot_local_planner::sgn(1.0), 1);
}

TEST(TestSignum, minus1)
{
    ASSERT_DOUBLE_EQ(ruvu_carrot_local_planner::signum(-1.0), -1);
}

TEST(TestSignum, 0)
{
    ASSERT_DOUBLE_EQ(ruvu_carrot_local_planner::signum(0.0), 0);
}

TEST(TestSignum, 1)
{
    ASSERT_DOUBLE_EQ(ruvu_carrot_local_planner::signum(1.0), 1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
