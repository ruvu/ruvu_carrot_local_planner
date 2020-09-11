// Copyright 2019 RUVU BV.

#pragma once

#include <base_local_planner/local_planner_util.h>
#include <mbf_utility/types.h>
#include "./local_planner_limits.hpp"

namespace ruvu_carrot_local_planner
{
class LocalPlannerUtil : public base_local_planner::LocalPlannerUtil
{
public:
  void reconfigureCB(LocalPlannerLimits& config, bool restore_defaults);
  void initialize(TF* tf, costmap_2d::Costmap2D* costmap, std::string global_frame);
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
  bool getLocalPlan(const geometry_msgs::PoseStamped& global_pose,
                    std::vector<geometry_msgs::PoseStamped>& transformed_plan);
  LocalPlannerLimits getCurrentLimits();

private:
  std::vector<geometry_msgs::PoseStamped> pruned_plan_;
};
}  // namespace ruvu_carrot_local_planner
