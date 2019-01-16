#include "./carrot_planner_ros.h"

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

#include "./utils.h"

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ruvu_carrot_local_planner::CarrotPlannerROS, nav_core::BaseLocalPlanner)

namespace ruvu_carrot_local_planner
{
void CarrotPlannerROS::reconfigureCB(CarrotPlannerConfig& config, uint32_t level)
{
  if (setup_ && config.restore_defaults)
  {
    config = default_config_;
    config.restore_defaults = false;
  }
  if (!setup_)
  {
    default_config_ = config;
    setup_ = true;
  }

  // update generic local planner params
  base_local_planner::LocalPlannerLimits limits;
  limits.max_trans_vel = config.max_trans_vel;
  limits.min_trans_vel = config.min_trans_vel;
  limits.max_vel_x = config.max_vel_x;
  limits.min_vel_x = config.min_vel_x;
  limits.max_vel_y = config.max_vel_y;
  limits.min_vel_y = config.min_vel_y;
  limits.max_rot_vel = config.max_rot_vel;
  limits.min_rot_vel = config.min_rot_vel;
  limits.acc_lim_x = config.acc_lim_x;
  limits.acc_lim_y = config.acc_lim_y;
  limits.acc_lim_theta = config.acc_lim_theta;
  limits.acc_limit_trans = config.acc_limit_trans;
  limits.xy_goal_tolerance = config.xy_goal_tolerance;
  limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
  limits.prune_plan = config.prune_plan;
  limits.trans_stopped_vel = config.trans_stopped_vel;
  limits.rot_stopped_vel = config.rot_stopped_vel;
  planner_util_.reconfigureCB(limits, config.restore_defaults);

  // initialize parameters specific to this module
  parameters.carrot_distance = config.carrot_distance;
  parameters.p_angle = config.p_angle;
}

CarrotPlannerROS::CarrotPlannerROS() : odom_helper_("odom")
{
}

void CarrotPlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!isInitialized())
  {
    ros::NodeHandle private_nh("~/" + name);
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    // make sure to update the costmap we'll use for this cycle
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

    planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

    if (private_nh.getParam("odom_topic", odom_topic_))
    {
      odom_helper_.setOdomTopic(odom_topic_);
    }

    initialized_ = true;

    dsrv_ = new dynamic_reconfigure::Server<CarrotPlannerConfig>(private_nh);
    dynamic_reconfigure::Server<CarrotPlannerConfig>::CallbackType cb =
        boost::bind(&CarrotPlannerROS::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }
  else
  {
    ROS_WARN("This planner has already been initialized, doing nothing.");
  }
}

bool CarrotPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!isInitialized())
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  // when we get a new plan, we also want to clear any latch we may have on goal tolerances
  latchedStopRotateController_.resetLatching();

  ROS_INFO("Got new plan");
  goal_reached_ = false;
  return planner_util_.setPlan(orig_global_plan);
}

bool CarrotPlannerROS::isGoalReached()
{
  if (!isInitialized())
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  if (goal_reached_)
  {
    ROS_INFO("CarrotPlanner: Goal reached.");
  }
  return goal_reached_;
}

void CarrotPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path)
{
  base_local_planner::publishPlan(path, l_plan_pub_);
}

void CarrotPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path)
{
  base_local_planner::publishPlan(path, g_plan_pub_);
}

CarrotPlannerROS::~CarrotPlannerROS()
{
  // make sure to clean things up
  delete dsrv_;
}

bool CarrotPlannerROS::carrotComputeVelocityCommands(const std::vector<geometry_msgs::PoseStamped>& path,
                                                     const tf::Stamped<tf::Pose>& global_pose,
                                                     geometry_msgs::Twist& cmd_vel)
{
  // look for the closest point on the path
  auto closest = min_by(path.begin(), path.end(), [&](const geometry_msgs::PoseStamped& ps) {
    return base_local_planner::getGoalPositionDistance(global_pose, ps.pose.position.x, ps.pose.position.y);
  });
  ROS_INFO_STREAM_NAMED("ruvu_carrot_local_planner", "closest element at: " << std::distance(path.begin(), closest));

  // convert to tf
  tf::Stamped<tf::Pose> closest_pose;
  tf::poseStampedMsgToTF(*closest, closest_pose);

  // walk carrot_distance forward
  auto carrot = std::find_if(closest, path.end(), [&](const geometry_msgs::PoseStamped& ps) {
    return parameters.carrot_distance <
           base_local_planner::getGoalPositionDistance(closest_pose, ps.pose.position.x, ps.pose.position.y);
  });
  ROS_INFO_STREAM_NAMED("ruvu_carrot_local_planner", "carrot element at: " << std::distance(path.begin(), carrot));

  if (carrot == path.end())
  {
    ROS_WARN_STREAM_NAMED("ruvu_carrot_local_planner", "carrot is at the end of the path");
    return false;
  }

  // convert to tf
  tf::Stamped<tf::Pose> carrot_pose;
  tf::poseStampedMsgToTF(*carrot, carrot_pose);

  double x = global_pose.getOrigin().getX();
  double y = global_pose.getOrigin().getY();
  double angle_to_goal = atan2(carrot->pose.position.y - y, carrot->pose.position.x - x);
  double angle_error = base_local_planner::getGoalOrientationAngleDifference(global_pose, angle_to_goal);

  cmd_vel.linear.x = 1.0;
  cmd_vel.angular.z = parameters.p_angle * angle_error;

  return true;
}

bool CarrotPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close
  // enough to goal
  tf::Stamped<tf::Pose> current_pose_;
  if (!costmap_ros_->getRobotPose(current_pose_))
  {
    ROS_ERROR("Could not get robot pose");
    return false;
  }
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  if (!planner_util_.getLocalPlan(current_pose_, transformed_plan))
  {
    ROS_ERROR("Could not get local plan");
    return false;
  }

  // if the global plan passed in is empty... we won't do anything
  if (transformed_plan.empty())
  {
    ROS_WARN_NAMED("ruvu_carrot_local_planner", "Received an empty transformed plan.");
    return false;
  }
  ROS_DEBUG_NAMED("ruvu_carrot_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

  if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_))
  {
    goal_reached_ = true;

    // publish an empty plan because we've reached our goal position
    std::vector<geometry_msgs::PoseStamped> local_plan;
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    publishGlobalPlan(transformed_plan);
    publishLocalPlan(local_plan);
  }
  else
  {
    bool isOk = carrotComputeVelocityCommands(transformed_plan, current_pose_, cmd_vel);
    if (isOk)
    {
      publishGlobalPlan(transformed_plan);
    }
    else
    {
      ROS_WARN_NAMED("ruvu_carrot_local_planner", "DWA planner failed to produce path.");
      std::vector<geometry_msgs::PoseStamped> empty_plan;
      publishGlobalPlan(empty_plan);
    }
    return isOk;
  }
}
};
