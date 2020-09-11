// Copyright 2019 RUVU BV.

#include "./local_planner_limits.hpp"

#include <ros/common.h>

namespace ruvu_carrot_local_planner
{
LocalPlannerLimits::LocalPlannerLimits(base_local_planner::LocalPlannerLimits limits)
{
  max_vel_x = limits.max_vel_x;
  min_vel_x = limits.min_vel_x;
  max_vel_y = limits.max_vel_y;
  min_vel_y = limits.min_vel_y;

  acc_lim_x = limits.acc_lim_x;
  acc_lim_y = limits.acc_lim_y;
  acc_lim_theta = limits.acc_lim_theta;

  xy_goal_tolerance = limits.xy_goal_tolerance;
  yaw_goal_tolerance = limits.yaw_goal_tolerance;
  prune_plan = limits.prune_plan;
  trans_stopped_vel = limits.trans_stopped_vel;

#if ROS_VERSION_MINIMUM(1, 14, 0)  // melodic
  max_vel_trans = limits.max_vel_trans;
  min_vel_trans = limits.min_vel_trans;
  max_vel_theta = limits.max_vel_theta;
  min_vel_theta = limits.min_vel_theta;
  acc_lim_trans = limits.acc_lim_trans;
  theta_stopped_vel = limits.theta_stopped_vel;
#else
  max_vel_trans = limits.max_trans_vel;
  min_vel_trans = limits.min_trans_vel;
  max_vel_theta = limits.max_rot_vel;
  min_vel_theta = limits.min_rot_vel;
  acc_lim_trans = limits.acc_limit_trans;
  theta_stopped_vel = limits.rot_stopped_vel;
#endif
}

LocalPlannerLimits::operator base_local_planner::LocalPlannerLimits() const
{
  base_local_planner::LocalPlannerLimits limits;

  limits.max_vel_x = max_vel_x;
  limits.min_vel_x = min_vel_x;
  limits.max_vel_y = max_vel_y;
  limits.min_vel_y = min_vel_y;

  limits.acc_lim_x = acc_lim_x;
  limits.acc_lim_y = acc_lim_y;
  limits.acc_lim_theta = acc_lim_theta;

  limits.xy_goal_tolerance = xy_goal_tolerance;
  limits.yaw_goal_tolerance = yaw_goal_tolerance;
  limits.prune_plan = prune_plan;
  limits.trans_stopped_vel = trans_stopped_vel;

#if ROS_VERSION_MINIMUM(1, 14, 0)  // melodic
  limits.max_vel_trans = max_vel_trans;
  limits.min_vel_trans = min_vel_trans;
  limits.max_vel_theta = max_vel_theta;
  limits.min_vel_theta = min_vel_theta;
  limits.acc_lim_trans = acc_lim_trans;
  limits.theta_stopped_vel = theta_stopped_vel;
#else
  limits.max_trans_vel = max_vel_trans;
  limits.min_trans_vel = min_vel_trans;
  limits.max_rot_vel = max_vel_theta;
  limits.min_rot_vel = min_vel_theta;
  limits.acc_limit_trans = acc_lim_trans;
  limits.rot_stopped_vel = theta_stopped_vel;
#endif

  return limits;
}

}  // namespace ruvu_carrot_local_planner
