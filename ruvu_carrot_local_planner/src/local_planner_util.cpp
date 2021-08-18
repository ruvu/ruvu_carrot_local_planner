// Copyright 2019 RUVU BV.

#include "./local_planner_util.hpp"

#include <base_local_planner/goal_functions.h>

#include "./utils.hpp"

namespace ruvu_carrot_local_planner
{
void LocalPlannerUtil::reconfigureCB(LocalPlannerLimits& config, bool restore_defaults)
{
  auto limits = static_cast<base_local_planner::LocalPlannerLimits>(config);
  base_local_planner::LocalPlannerUtil::reconfigureCB(limits, restore_defaults);
}
void LocalPlannerUtil::initialize(TF* tf, costmap_2d::Costmap2D* costmap, std::string global_frame)
{
  base_local_planner::LocalPlannerUtil::initialize(tf, costmap, global_frame);
}

bool LocalPlannerUtil::getLocalPlan(const geometry_msgs::PoseStamped& global_pose,
                                    std::vector<geometry_msgs::PoseStamped>& transformed_plan)
{
#ifdef USE_OLD_TF
  tf::Stamped<tf::Pose> gp;
  tf::poseStampedMsgToTF(global_pose, gp);
#else
  geometry_msgs::PoseStamped gp = global_pose;
#endif

  if (!base_local_planner::LocalPlannerUtil::getLocalPlan(gp, transformed_plan))
    return false;

  // now we'll prune the plan based on the position of the robot
  if (getCurrentLimits().prune_plan)
  {
    // Custom path pruning

    // Look for the closest point on the path
    auto closest =
        min_by(transformed_plan.begin(), transformed_plan.end(), [&gp](const geometry_msgs::PoseStamped& ps) {
          return base_local_planner::getGoalPositionDistance(gp, ps.pose.position.x, ps.pose.position.y);
        });

    auto remove_count = closest - transformed_plan.begin();
    transformed_plan.erase(transformed_plan.begin(), transformed_plan.begin() + remove_count);
  }

  return true;
}

LocalPlannerLimits LocalPlannerUtil::getCurrentLimits()
{
  return static_cast<LocalPlannerLimits>(base_local_planner::LocalPlannerUtil::getCurrentLimits());
}

}  // namespace ruvu_carrot_local_planner
