// Copyright 2019 RUVU BV.

#include "./latched_stop_rotate_controller.hpp"

#include <mbf_utility/types.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

namespace ruvu_carrot_local_planner
{
class LatchedStopRotateController::LatchedStopRotateControllerImpl
{
public:
#if ROS_VERSION_MINIMUM(1, 13, 0)
  explicit LatchedStopRotateControllerImpl(const std::string& name) : inner_controller_(name)
  {
  }
#else
  // on kinetic the name argument does not yet exist, fall back to default constructor
  explicit LatchedStopRotateControllerImpl(const std::string& name) : inner_controller_()
  {
  }
#endif
  base_local_planner::LatchedStopRotateController inner_controller_;
};

LatchedStopRotateController::LatchedStopRotateController(const std::string& name)
  : impl_(new LatchedStopRotateControllerImpl(name))
{
}
LatchedStopRotateController::~LatchedStopRotateController() = default;

bool LatchedStopRotateController::isPositionReached(base_local_planner::LocalPlannerUtil* planner_util,
                                                    const geometry_msgs::PoseStamped& global_pose)
{
#ifdef USE_OLD_TF
  tf::Stamped<tf::Pose> global_pose_tf;
  tf::poseStampedMsgToTF(global_pose, global_pose_tf);
  return impl_->inner_controller_.isPositionReached(planner_util, global_pose_tf);
#else
  return impl_->inner_controller_.isPositionReached(planner_util, global_pose);
#endif
}

bool LatchedStopRotateController::isGoalReached(base_local_planner::LocalPlannerUtil* planner_util,
                                                base_local_planner::OdometryHelperRos& odom_helper,
                                                const geometry_msgs::PoseStamped& global_pose)
{
#ifdef USE_OLD_TF
  tf::Stamped<tf::Pose> global_pose_tf;
  tf::poseStampedMsgToTF(global_pose, global_pose_tf);
  return impl_->inner_controller_.isGoalReached(planner_util, odom_helper, global_pose_tf);
#else
  return impl_->inner_controller_.isGoalReached(planner_util, odom_helper, global_pose);
#endif
}

bool LatchedStopRotateController::computeVelocityCommandsStopRotate(
    geometry_msgs::Twist& cmd_vel, Eigen::Vector3f acc_lim, double sim_period,
    base_local_planner::LocalPlannerUtil* planner_util, base_local_planner::OdometryHelperRos& odom_helper_,
    const geometry_msgs::PoseStamped& global_pose,
    boost::function<bool(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples)> obstacle_check)
{
#ifdef USE_OLD_TF
  tf::Stamped<tf::Pose> global_pose_tf;
  tf::poseStampedMsgToTF(global_pose, global_pose_tf);
  return impl_->inner_controller_.computeVelocityCommandsStopRotate(cmd_vel, acc_lim, sim_period, planner_util,
                                                                    odom_helper_, global_pose_tf, obstacle_check);
#else
  return impl_->inner_controller_.computeVelocityCommandsStopRotate(cmd_vel, acc_lim, sim_period, planner_util,
                                                                    odom_helper_, global_pose, obstacle_check);
#endif
}

void LatchedStopRotateController::resetLatching()
{
  impl_->inner_controller_.resetLatching();
}
}  // namespace ruvu_carrot_local_planner
