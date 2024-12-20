// Copyright 2019 RUVU BV.

#pragma once

#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/local_planner_util.h>

namespace ruvu_carrot_local_planner
{
/**
 * This class implements wrapper functions with geometry_msgs::PoseStamped for kinetic to use it more easily with
 * move_base_flex.
 */
class LatchedStopRotateController
{
public:
  explicit LatchedStopRotateController(const std::string& name);
  ~LatchedStopRotateController();  // defined in the implementation file, where impl is a complete type

  bool isPositionReached(base_local_planner::LocalPlannerUtil* planner_util,
                         const geometry_msgs::PoseStamped& global_pose);
  bool isGoalReached(base_local_planner::LocalPlannerUtil* planner_util,
                     base_local_planner::OdometryHelperRos& odom_helper, const geometry_msgs::PoseStamped& global_pose);
  bool computeVelocityCommandsStopRotate(
      geometry_msgs::Twist& cmd_vel, Eigen::Vector3f acc_lim, double sim_period,
      base_local_planner::LocalPlannerUtil* planner_util, base_local_planner::OdometryHelperRos& odom_helper,
      const geometry_msgs::PoseStamped& global_pose,
      boost::function<bool(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples)> obstacle_check);
  void resetLatching();

private:
  class LatchedStopRotateControllerImpl;
  std::unique_ptr<LatchedStopRotateControllerImpl> impl_;
};

}  // namespace ruvu_carrot_local_planner
