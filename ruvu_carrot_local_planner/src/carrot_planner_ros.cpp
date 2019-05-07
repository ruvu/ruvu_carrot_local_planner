// Copyright 2019 RUVU BV.

#include "./carrot_planner_ros.h"

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include "./utils.h"

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ruvu_carrot_local_planner::CarrotPlannerROS, mbf_costmap_core::CostmapController)

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
    ROS_WARN_NAMED("ruvu_carrot_local_planner", "This planner has already been initialized, doing nothing.");
  }
}

bool CarrotPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!isInitialized())
  {
    ROS_ERROR_NAMED("ruvu_carrot_local_planner", "This planner has not been initialized, please call initialize() "
                                                 "before using this planner");
    return false;
  }
  // when we get a new plan, we also want to clear any latch we may have on goal tolerances
  latchedStopRotateController_.resetLatching();

  state_ = State::DRIVING;

  ROS_INFO_NAMED("ruvu_carrot_local_planner", "Got new plan");
  return planner_util_.setPlan(orig_global_plan);
}

bool CarrotPlannerROS::isGoalReached(double xy_tolerance, double yaw_tolerance)
{
  if (!isInitialized())
  {
    ROS_ERROR_NAMED("ruvu_carrot_local_planner", "This planner has not been initialized, please call initialize() "
                                                 "before using this planner");
    return false;
  }
  tf::Stamped<tf::Pose> current_pose_;
  if (!costmap_ros_->getRobotPose(current_pose_))
  {
    ROS_ERROR_NAMED("ruvu_carrot_local_planner", "Could not get robot pose");
    return false;
  }

  if (latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_))
  {
    ROS_INFO_NAMED("ruvu_carrot_local_planner", "Goal reached");
    return true;
  }
  else
  {
    return false;
  }
}

bool CarrotPlannerROS::cancel()
{
  return false;
}

void CarrotPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path)
{
  base_local_planner::publishPlan(path, l_plan_pub_);
}

void CarrotPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path)
{
  base_local_planner::publishPlan(path, g_plan_pub_);
}

void CarrotPlannerROS::publishDebugCarrot(const tf::Stamped<tf::Pose>& carrot)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = carrot.frame_id_;
  marker.header.stamp = carrot.stamp_;
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

CarrotPlannerROS::~CarrotPlannerROS()
{
  // make sure to clean things up
  delete dsrv_;
}

bool CarrotPlannerROS::carrotComputeVelocityCommands(const std::vector<geometry_msgs::PoseStamped>& path,
                                                     const tf::Stamped<tf::Pose>& global_pose,
                                                     geometry_msgs::Twist& cmd_vel)
{
  // Look for the closest point on the path
  auto closest = min_by(path.begin(), path.end(), [&global_pose](const geometry_msgs::PoseStamped& ps) {
    return base_local_planner::getGoalPositionDistance(global_pose, ps.pose.position.x, ps.pose.position.y);
  });

  tf::Point goal;
  tf::pointMsgToTF(path.back().pose.position, goal);
  auto goal_error = goal - global_pose.getOrigin();
  double goal_distance = goal_error.length();

  if (goal_distance < parameters.carrot_distance && state_ != State::ARRIVING)
  {
    ROS_INFO_NAMED("ruvu_carrot_local_planner", "I'm close to the goal, let's go to the arriving state");
    state_ = State::ARRIVING;
    arriving_angle_ = atan2(goal_error.getY(), goal_error.getX());
  }

  tf::Stamped<tf::Pose> carrot(tf::Pose::getIdentity(), global_pose.stamp_, global_pose.frame_id_);
  switch (state_)
  {
    case State::DRIVING:
      computeCarrot(path, closest, carrot);
      break;
    case State::ARRIVING:
    {
      // The carrot walks along the arriving angle from the goal
      auto direction = tf::quatRotate(tf::createQuaternionFromYaw(arriving_angle_),
                                      tf::Vector3(parameters.carrot_distance - goal_distance, 0, 0));
      if (direction.dot(goal_error) < 0)
      {
        ROS_WARN_THROTTLE_NAMED(5, "ruvu_carrot_local_planner", "Goal overshoot detected");
        return false;
      }
      carrot.setOrigin(goal + direction);
      break;
    }
    default:
      assert(false);
  }

  publishDebugCarrot(carrot);

  nav_msgs::Odometry odom;
  odom_helper_.getOdom(odom);
  auto limits = planner_util_.getCurrentLimits();

  {
    // Don't go faster than the braking distance sqrt(2as)
    double stopping_distance = limits.xy_goal_tolerance * 2;
    goal_distance = goal_distance > stopping_distance ? goal_distance - stopping_distance : 0;
    double v_max = sqrt(2 * limits.acc_lim_x * goal_distance);
    ROS_DEBUG_STREAM_NAMED("ruvu_carrot_local_planner", "v_max: " << v_max);
    cmd_vel.linear.x = v_max > limits.max_vel_x ? limits.max_vel_x : v_max;
    cmd_vel.linear.x = cmd_vel.linear.x < limits.min_trans_vel ? limits.min_trans_vel : cmd_vel.linear.x;
  }

  auto error = carrot.getOrigin() - global_pose.getOrigin();
  double angle_to_carrot = atan2(error.getY(), error.getX());
  double carrot_error = base_local_planner::getGoalOrientationAngleDifference(global_pose, angle_to_carrot);

  // determine the position of the carrot relative to the closest point
  tf::Point closest_position;
  tf::pointMsgToTF(closest->pose.position, closest_position);
  auto rel_carrot = carrot.getOrigin() - closest_position;

  // Determine the angle of the poses on the path relative to the direction of the path. The direciton of the path can
  // be estimated by looking at the location of the carrot relative the the closest point. That is "forward" along the
  // path.
  double path_direction = angles::shortest_angular_distance(tf::getYaw(closest->pose.orientation),
                                                            atan2(rel_carrot.getY(), rel_carrot.getX()));
  bool switch_direction = !(-M_PI_2 < path_direction && path_direction < M_PI_2);

  if (switch_direction)
  {
    carrot_error = angles::normalize_angle(carrot_error + M_PI);
  }

  cmd_vel.angular.z = parameters.p_angle * carrot_error;

  if (switch_direction)
  {
    cmd_vel.linear.x = -cmd_vel.linear.x;
  }

  // If we rotate faster than possible, scale back the both velocities
  if (fabs(cmd_vel.angular.z) > limits.max_rot_vel)
  {
    cmd_vel.linear.x = cmd_vel.linear.x * limits.max_rot_vel / fabs(cmd_vel.angular.z);
    cmd_vel.angular.z = cmd_vel.angular.z * limits.max_rot_vel / fabs(cmd_vel.angular.z);
  }

  // Smooth the required velocity with the maximum acceleration
  double max_x_step = limits.acc_lim_x * sim_period_;
  double max_theta_step = limits.acc_lim_theta * sim_period_;
  if (fabs(cmd_vel.linear.x - odom.twist.twist.linear.x) > max_x_step)
  {
    cmd_vel.linear.x = odom.twist.twist.linear.x + sgn(cmd_vel.linear.x - odom.twist.twist.linear.x) * max_x_step;
  }
  if (fabs(cmd_vel.angular.z - odom.twist.twist.angular.z) > max_theta_step)
  {
    cmd_vel.angular.z =
        odom.twist.twist.angular.z + sgn(cmd_vel.angular.z - odom.twist.twist.angular.z) * max_theta_step;
  }

  return true;
}

void CarrotPlannerROS::computeCarrot(const std::vector<geometry_msgs::PoseStamped>& path,
                                     std::vector<geometry_msgs::PoseStamped>::const_iterator it,
                                     tf::Stamped<tf::Pose>& carrot)
{
  // Walk along the path forward and count the distance. When carrot_distance has been walked, the carrot is found.
  double distance = parameters.carrot_distance;

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

  carrot = current;
}

uint32_t CarrotPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& robot_pose,
                                                   const geometry_msgs::TwistStamped& robot_velocity,
                                                   geometry_msgs::TwistStamped& cmd_vel, std::string& message)
{
  // Dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close
  // enough to goal
  tf::Stamped<tf::Pose> current_pose_;
  if (!costmap_ros_->getRobotPose(current_pose_))
  {
    ROS_ERROR_NAMED("ruvu_carrot_local_planner", "Could not get robot pose");
    return 111; // TF_ERROR
  }
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  if (!planner_util_.getLocalPlan(current_pose_, transformed_plan))
  {
    ROS_ERROR_NAMED("ruvu_carrot_local_planner", "Could not get local plan");
    return 108; // MISSED_PATH
  }

  // If the global plan passed in is empty... we won't do anything
  if (transformed_plan.empty())
  {
    ROS_WARN_NAMED("ruvu_carrot_local_planner", "Received an empty transformed plan.");
    return 108; // MISSED_PATH
  }

  if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_))
  {
    // Publish an empty plan because we've reached our goal position
    std::vector<geometry_msgs::PoseStamped> local_plan;
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    publishGlobalPlan(transformed_plan);
    publishLocalPlan(local_plan);
    auto limits = planner_util_.getCurrentLimits();
    if (latchedStopRotateController_.computeVelocityCommandsStopRotate(
        cmd_vel.twist, limits.getAccLimits(), sim_period_, &planner_util_, odom_helper_, current_pose_,
        boost::bind(&CarrotPlannerROS::checkTrajectory, this, _1, _2, _3))) {
        return 0;
    } else {
        return 100;
    }
  }
  else
  {
    bool isOk = carrotComputeVelocityCommands(transformed_plan, current_pose_, cmd_vel.twist);
    if (isOk)
    {
      ROS_DEBUG_NAMED("ruvu_carrot_local_planner", "Computed the following cmd_vel: %.3lf, %.3lf, %.3lf",
                      cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
      publishGlobalPlan(transformed_plan);
    }
    else
    {
      ROS_WARN_THROTTLE_NAMED(5, "ruvu_carrot_local_planner", "Carrot planner failed to produce path.");
      std::vector<geometry_msgs::PoseStamped> empty_plan;
      publishGlobalPlan(empty_plan);
    }
    return isOk ? 0 : 100;
  }
}

bool CarrotPlannerROS::checkTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples)
{
  // TODO(ramon): check if the footprint collides with an obstacle
  return true;
}
}  // namespace ruvu_carrot_local_planner
