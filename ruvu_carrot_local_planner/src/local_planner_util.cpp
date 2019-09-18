// Copyright 2019 RUVU BV.

#include "./local_planner_util.h"

#include <base_local_planner/goal_functions.h>

#include "./utils.h"

namespace ruvu_carrot_local_planner
{
void LocalPlannerUtil::initialize(tf::TransformListener* tf, costmap_2d::Costmap2D* costmap, std::string global_frame)
{
  base_local_planner::LocalPlannerUtil::initialize(tf, costmap, global_frame);
}

bool LocalPlannerUtil::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!base_local_planner::LocalPlannerUtil::setPlan(orig_global_plan))
    return false;

  pruned_plan_ = orig_global_plan;
  return true;
}

bool LocalPlannerUtil::getLocalPlan(const geometry_msgs::PoseStamped& global_pose,
                                    std::vector<geometry_msgs::PoseStamped>& transformed_plan)
{
  tf::Stamped<tf::Pose> gp;
  tf::poseStampedMsgToTF(global_pose, gp);

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
    pruned_plan_.erase(pruned_plan_.begin(), pruned_plan_.begin() + remove_count);
    transformed_plan.erase(transformed_plan.begin(), transformed_plan.begin() + remove_count);

    base_local_planner::LocalPlannerUtil::setPlan(pruned_plan_);
  }

  return true;
}

}  // namespace ruvu_carrot_local_planner
