#include "./carrot_planner_ros.h"

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

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
    debug_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("visualization", 1);
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    // make sure to update the costmap we'll use for this cycle
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

    planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

    if (private_nh.getParam("odom_topic", odom_topic_))
    {
      odom_helper_.setOdomTopic(odom_topic_);
    }

    sim_period_ = getSimPeriodParam(private_nh);

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

double CarrotPlannerROS::getSimPeriodParam(ros::NodeHandle private_nh)
{
  // https://github.com/ros-planning/navigation/blob/melodic-devel/dwa_local_planner/src/dwa_planner_ros.cpp

  // Assuming this planner is being run within the navigation stack, we can
  // just do an upward search for the frequency at which its being run. This
  // also allows the frequency to be overwritten locally.
  double sim_period;
  std::string controller_frequency_param_name;
  if (!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
  {
    sim_period = 0.05;
  }
  else
  {
    double controller_frequency = 0;
    private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
    if (controller_frequency > 0)
    {
      sim_period = 1.0 / controller_frequency;
    }
    else
    {
      ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
      sim_period = 0.05;
    }
  }
  ROS_INFO("Sim period is set to %.2f", sim_period);

  return sim_period;
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
  tf::Stamped<tf::Pose> carrot;
  double goal_distance;
  computeCarrot(path, global_pose, carrot, goal_distance);
  ROS_INFO_NAMED("ruvu_carrot_local_planner", "goal distance: %f", goal_distance);

  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = global_pose.frame_id_;
    marker.header.stamp = global_pose.stamp_;
    marker.ns = "carrot";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    tf::poseTFToMsg(carrot, marker.pose);
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    visualization_msgs::MarkerArray markers;
    markers.markers = { marker };
    debug_pub_.publish(markers);
  }

  nav_msgs::Odometry odom;
  odom_helper_.getOdom(odom);

  auto limits = planner_util_.getCurrentLimits();
  // Don't go faster than the braking distance sqrt(2as)
  double v_max = sqrt(2 * limits.acc_lim_x * goal_distance);
  cmd_vel.linear.x = v_max > limits.max_vel_x ? limits.max_vel_x : v_max;

  auto error = carrot.getOrigin() - global_pose.getOrigin();
  double angle_to_goal = atan2(error.getY(), error.getX());
  double angle_error = base_local_planner::getGoalOrientationAngleDifference(global_pose, angle_to_goal);
  cmd_vel.angular.z = parameters.p_angle * angle_error;

  // If we rotate faster than possible, scale back the both velocities
  if (fabs(cmd_vel.angular.z) > limits.max_rot_vel)
  {
    cmd_vel.linear.x = cmd_vel.linear.x * limits.max_rot_vel / fabs(cmd_vel.angular.z);
    cmd_vel.angular.z = cmd_vel.angular.z * limits.max_rot_vel / fabs(cmd_vel.angular.z);
  }

  // Smooth the required velocity with the maximum acceleration
  double max_x_step = limits.acc_lim_x * sim_period_;
  double max_theta_step = limits.acc_lim_theta * sim_period_;

  if (fabs(odom.twist.twist.linear.x - cmd_vel.linear.x) > max_x_step)
  {
    cmd_vel.linear.x = odom.twist.twist.linear.x + sgn(odom.twist.twist.linear.x - cmd_vel.linear.x) * max_x_step;
  }

  return true;
}

void CarrotPlannerROS::computeCarrot(const std::vector<geometry_msgs::PoseStamped>& path,
                                     const tf::Stamped<tf::Pose>& global_pose, tf::Stamped<tf::Pose>& carrot,
                                     double& goal_distance)
{
  // Look for the closest point on the path
  auto closest = min_by(path.begin(), path.end(), [&](const geometry_msgs::PoseStamped& ps) {
    return base_local_planner::getGoalPositionDistance(global_pose, ps.pose.position.x, ps.pose.position.y);
  });
  ROS_INFO_STREAM_NAMED("ruvu_carrot_local_planner", "closest element at: " << std::distance(path.begin(), closest));

  // Walk along the path forward and count the distance. When carrot_distance has been walked, the carrot is found.
  double distance = parameters.carrot_distance;

  auto it = closest;
  tf::Stamped<tf::Pose> previous;
  tf::Stamped<tf::Pose> current;
  tf::poseStampedMsgToTF(*it, current);
  for (; ++it < path.end();)
  {
    // Update previous & current
    previous = current;
    tf::poseStampedMsgToTF(*it, current);

    distance -= (current.getOrigin() - previous.getOrigin()).length();
    if (distance <= 0)
    {
      break;
    }
  }

  goal_distance = parameters.carrot_distance - distance;

  if (it == path.end())
  {
    // The carrot point is further than path.end(), so let's extrapolate the path
    // We can't use the orientation of the last point because it could be in a different direction. The point before
    // that has a almost random orientation in the current global_planner implementation. So let's take the point before
    // that because it appears to be more stable.
    if (it - 3 >= path.begin())
    {
      tf::poseStampedMsgToTF(*(it - 3), previous);
    }

    // Travel `distance` meters in that direction
    auto direction = tf::quatRotate(previous.getRotation(), tf::Vector3(distance, 0, 0));
    current.setOrigin(current.getOrigin() + direction);
  }
  carrot = current;
}

bool CarrotPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  // Dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close
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

  // If the global plan passed in is empty... we won't do anything
  if (transformed_plan.empty())
  {
    ROS_WARN_NAMED("ruvu_carrot_local_planner", "Received an empty transformed plan.");
    return false;
  }
  ROS_DEBUG_NAMED("ruvu_carrot_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

  if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_))
  {
    goal_reached_ = true;

    // Publish an empty plan because we've reached our goal position
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
