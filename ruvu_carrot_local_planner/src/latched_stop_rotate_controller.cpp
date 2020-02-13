// Copyright 2019 RUVU BV.

#include "./latched_stop_rotate_controller.h"

namespace ruvu_carrot_local_planner
{
#ifdef USE_OLD_TF
bool LatchedStopRotateController::isPositionReached(base_local_planner::LocalPlannerUtil* planner_util,
                                                    const geometry_msgs::PoseStamped& global_pose)
{
  tf::Stamped<tf::Pose> global_pose_tf;
  tf::poseStampedMsgToTF(global_pose, global_pose_tf);
  return base_local_planner::LatchedStopRotateController::isPositionReached(planner_util, global_pose_tf);
}

bool LatchedStopRotateController::computeVelocityCommandsStopRotate(
    geometry_msgs::Twist& cmd_vel, Eigen::Vector3f acc_lim, double sim_period,
    base_local_planner::LocalPlannerUtil* planner_util, base_local_planner::OdometryHelperRos& odom_helper_,
    const geometry_msgs::PoseStamped& global_pose,
    boost::function<bool(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples)> obstacle_check)
{
  tf::Stamped<tf::Pose> global_pose_tf;
  tf::poseStampedMsgToTF(global_pose, global_pose_tf);
  return base_local_planner::LatchedStopRotateController::computeVelocityCommandsStopRotate(
      cmd_vel, acc_lim, sim_period, planner_util, odom_helper_, global_pose_tf, obstacle_check);
}
#endif
}  // namespace ruvu_carrot_local_planner