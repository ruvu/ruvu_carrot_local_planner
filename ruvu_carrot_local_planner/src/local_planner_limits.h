// Copyright 2019 RUVU BV.

#pragma once

#include <base_local_planner/local_planner_limits.h>

namespace ruvu_carrot_local_planner
{
/**
 * This LocalPlannerLimits implements the melodic style parameters, even on kinetic. This allow the code to be backwards
 * compatible.
 */
class LocalPlannerLimits
{
public:
  double max_vel_trans;
  double min_vel_trans;
  double max_vel_x;
  double min_vel_x;
  double max_vel_y;
  double min_vel_y;
  double max_vel_theta;
  double min_vel_theta;
  double acc_lim_x;
  double acc_lim_y;
  double acc_lim_theta;
  double acc_lim_trans;
  bool prune_plan;
  double xy_goal_tolerance;
  double yaw_goal_tolerance;
  double trans_stopped_vel;
  double theta_stopped_vel;
  bool restore_defaults;

  Eigen::Vector3f getAccLimits()
  {
    Eigen::Vector3f acc_limits;
    acc_limits[0] = acc_lim_x;
    acc_limits[1] = acc_lim_y;
    acc_limits[2] = acc_lim_theta;
    return acc_limits;
  }

  LocalPlannerLimits() = default;

  LocalPlannerLimits(base_local_planner::LocalPlannerLimits limits);

  explicit operator base_local_planner::LocalPlannerLimits() const;
};
}  // namespace ruvu_carrot_local_planner
