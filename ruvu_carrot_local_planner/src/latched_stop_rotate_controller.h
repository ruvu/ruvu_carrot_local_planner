// Copyright 2019 RUVU BV.

#pragma once

#include <base_local_planner/latched_stop_rotate_controller.h>
#include <mbf_utility/types.h>

namespace ruvu_carrot_local_planner
{
class LatchedStopRotateController : public base_local_planner::LatchedStopRotateController
{
public:
#ifdef USE_OLD_TF
  bool isPositionReached(base_local_planner::LocalPlannerUtil* planner_util,
                         const geometry_msgs::PoseStamped& global_pose);
  bool computeVelocityCommandsStopRotate(
      geometry_msgs::Twist& cmd_vel, Eigen::Vector3f acc_lim, double sim_period,
      base_local_planner::LocalPlannerUtil* planner_util, base_local_planner::OdometryHelperRos& odom_helper,
      const geometry_msgs::PoseStamped& global_pose,
      boost::function<bool(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples)> obstacle_check);
#endif
};

}  // namespace ruvu_carrot_local_planner