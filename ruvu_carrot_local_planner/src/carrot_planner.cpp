// Copyright 2019 RUVU BV.

#include "./carrot_planner.hpp"

#include <angles/angles.h>
#include <tf2/utils.h>
#include <visualization_msgs/MarkerArray.h>

#include "./local_planner_util.hpp"
#include "./simulator.hpp"
#include "./utils.hpp"
#include "./geometry_utils.hpp"

namespace ruvu_carrot_local_planner
{
CarrotPlanner::CarrotPlanner(ros::NodeHandle private_nh, Simulator* simulator, LocalPlannerUtil* planner_util)
  : simulator_(simulator)
  , planner_util_(planner_util)
  , sim_period_(getSimPeriodParam(private_nh))
  , debug_pub_(private_nh.advertise<visualization_msgs::MarkerArray>("visualization", 1))
{
}

void CarrotPlanner::reconfigure(const Parameters& parameters)
{
  boost::mutex::scoped_lock l(configuration_mutex_);

  parameters_ = parameters;
}

void CarrotPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  state_ = State::DRIVING;
}

void CarrotPlanner::updatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan)
{
  global_plan_.resize(new_plan.size());
  for (unsigned int i = 0; i < new_plan.size(); ++i)
  {
    global_plan_[i] = new_plan[i];
  }
}

CarrotPlanner::Outcome CarrotPlanner::computeVelocityCommands(const tf2::Stamped<tf2::Transform>& global_pose,
                                                              const geometry_msgs::Twist& global_vel,
                                                              geometry_msgs::Twist& cmd_vel, std::string& message,
                                                              base_local_planner::Trajectory& trajectory)
{
  geometry_msgs::PoseStamped gp;
  tf2::toMsg(global_pose, gp);

  // Look for the closest point on the path
  auto closest = min_by(global_plan_.begin(), global_plan_.end(),
                        [&gp](const geometry_msgs::PoseStamped& ps) { return distance2(gp, ps); });

  tf2::Vector3 goal;
  tf2::convert(global_plan_.back().pose.position, goal);
  auto goal_error = goal - global_pose.getOrigin();
  double goal_distance = goal_error.length();
  double carrot_distance =
      parameters_.carrot_distance + fabs(global_vel.linear.x) * parameters_.carrot_distance_scaling;

  if (goal_distance < carrot_distance && state_ != State::ARRIVING)
  {
    ROS_INFO_NAMED("ruvu_carrot_local_planner", "I'm close to the goal, let's go to the arriving state");
    state_ = State::ARRIVING;
    arriving_angle_ = atan2(goal_error.getY(), goal_error.getX());
  }

  auto limits = planner_util_->getCurrentLimits();

  tf2::Stamped<tf2::Transform> carrot(tf2::Transform::getIdentity(), global_pose.stamp_, global_pose.frame_id_);
  switch (state_)
  {
    case State::DRIVING:
      message = "Driving";
      carrot = computeCarrot(global_plan_, closest, carrot_distance);
      break;
    case State::ARRIVING: {
      message = "Arriving";
      // The carrot walks along the arriving angle from the goal
      tf2::Quaternion q;
      q.setRPY(0, 0, arriving_angle_);
      auto direction = tf2::quatRotate(q, tf2::Vector3(1, 0, 0));
      if (direction.dot(goal_error) < 0)
      {
        message = "Goal overshoot detected";
        ROS_WARN_STREAM_THROTTLE_NAMED(5, "ruvu_carrot_local_planner", message);
        return Outcome::MISSED_GOAL;
      }
      carrot.setOrigin(goal + (carrot_distance - goal_distance) * direction);
      break;
    }
    default:
      assert(false);
  }

  publishDebugCarrot(carrot);

  {
    double a = limits.acc_lim_x;
    double m = parameters_.slow_down_margin;

    double v1 = -a * m - sqrt(a * (a * m * m + 2 * goal_distance));
    double v2 = -a * m + sqrt(a * (a * m * m + 2 * goal_distance));
    ROS_DEBUG_NAMED("ruvu_carrot_local_planner", "v1=%f, v2=%f", v1, v2);

    double v_max = v2;
    ROS_DEBUG_NAMED("ruvu_carrot_local_planner", "v_max=%f v1=%f)", v_max, v1);
    cmd_vel.linear.x = v_max;
  }

  auto error = carrot.getOrigin() - global_pose.getOrigin();
  double angle_to_carrot = atan2(error.getY(), error.getX());

  // note, inlined getGoalOrientationAngleDifference because of broken compatability
  double carrot_error = angles::shortest_angular_distance(tf2::getYaw(gp.pose.orientation), angle_to_carrot);
  ROS_DEBUG_NAMED("ruvu_carrot_local_planner", "carrot_error=%f", carrot_error);

  // determine the position of the carrot relative to the closest point
  tf2::Vector3 closest_position;
  tf2::convert(closest->pose.position, closest_position);
  auto rel_carrot = carrot.getOrigin() - closest_position;

  // Determine the angle of the poses on the path relative to the direction of the path. The direciton of the path can
  // be estimated by looking at the location of the carrot relative the the closest point. That is "forward" along the
  // path.
  double path_direction = angles::shortest_angular_distance(tf2::getYaw(closest->pose.orientation),
                                                            atan2(rel_carrot.getY(), rel_carrot.getX()));
  bool switch_direction = !(-M_PI_2 < path_direction && path_direction < M_PI_2);

  if (switch_direction)
  {
    carrot_error = angles::normalize_angle(carrot_error + M_PI);
  }

  cmd_vel.angular.z = parameters_.p_angle * carrot_error;

  if (switch_direction)
  {
    cmd_vel.linear.x = -cmd_vel.linear.x;
  }

  // Smooth the required velocity with the maximum acceleration
  double max_x_step = limits.acc_lim_x * sim_period_;
  double max_theta_step = limits.acc_lim_theta * sim_period_;
  if (fabs(cmd_vel.linear.x - global_vel.linear.x) > max_x_step)
  {
    cmd_vel.linear.x = global_vel.linear.x + sgn(cmd_vel.linear.x - global_vel.linear.x) * max_x_step;
  }
  if (fabs(cmd_vel.angular.z - global_vel.angular.z) > max_theta_step)
  {
    cmd_vel.angular.z = global_vel.angular.z + sgn(cmd_vel.angular.z - global_vel.angular.z) * max_theta_step;
  }

  // Apply motion limits, be careful of the order, max vels should be after min_vel_trans
  cmd_vel.linear.x = std::abs(cmd_vel.linear.x) < limits.min_vel_trans ? sgn(cmd_vel.linear.x) * limits.min_vel_trans :
                                                                         cmd_vel.linear.x;
  cmd_vel.linear.x = std::max(cmd_vel.linear.x, limits.min_vel_x);
  cmd_vel.linear.x = std::min(cmd_vel.linear.x, limits.max_vel_x);
  cmd_vel.linear.x = std::min(cmd_vel.linear.x, limits.max_vel_trans);
  cmd_vel.linear.x = std::max(cmd_vel.linear.x, -limits.max_vel_trans);

  // verify
  if (std::abs(cmd_vel.linear.x) < limits.min_vel_trans)
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(5, "ruvu_carrot_local_planner", "min_vel_trans prevents the robot from moving");
    return Outcome::NO_VALID_CMD;
  }

  // At this point we have a valid linear velocity

  // Scale back the forward velocity when turning faster
  double brake_factor = 1 - pow(cmd_vel.angular.z, 2) / pow(limits.max_vel_theta, 2);
  brake_factor = brake_factor < 0 ? 0 : brake_factor;
  cmd_vel.linear.x *= brake_factor;

  // Re-apply potential violated linear velocity constraints
  cmd_vel.linear.x = std::abs(cmd_vel.linear.x) < limits.min_vel_trans ? sgn(cmd_vel.linear.x) * limits.min_vel_trans :
                                                                         cmd_vel.linear.x;
  cmd_vel.linear.x = std::max(cmd_vel.linear.x, limits.min_vel_x);
  cmd_vel.linear.x = std::min(cmd_vel.linear.x, limits.max_vel_x);

  // Apply all angular motion constraints
  cmd_vel.angular.z = std::abs(cmd_vel.angular.z) < limits.min_vel_theta ?
                          sgn(cmd_vel.angular.z) * limits.min_vel_theta :
                          cmd_vel.angular.z;
  if (fabs(cmd_vel.angular.z) > limits.max_vel_theta)
  {
    cmd_vel.angular.z = cmd_vel.angular.z * limits.max_vel_theta / fabs(cmd_vel.angular.z);
  }

  // check if that cmd_vel collides with an obstacle in the future
  trajectory = simulator_->simulateVelocity(global_pose, global_vel, cmd_vel);

  if (trajectory.cost_ < 0)
    return Outcome::BLOCKED_PATH;
  else
    return Outcome::OK;
}

tf2::Stamped<tf2::Transform> CarrotPlanner::computeCarrot(const std::vector<geometry_msgs::PoseStamped>& path,
                                                          std::vector<geometry_msgs::PoseStamped>::const_iterator it,
                                                          double carrot_distance)
{
  // Walk along the path forward and count the distance. When carrot_distance has been walked, the carrot is found.
  double distance = carrot_distance;

  tf2::Stamped<tf2::Transform> previous;
  tf2::Stamped<tf2::Transform> current;
  tf2::convert(*it, current);
  for (; ++it < path.end();)
  {
    // Update previous & current
    previous = current;
    tf2::convert(*it, current);

    distance -= (current.getOrigin() - previous.getOrigin()).length();
    if (distance <= 0)
    {
      break;
    }
  }

  return current;
}

void CarrotPlanner::publishDebugCarrot(const tf2::Stamped<tf2::Transform>& carrot)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = carrot.frame_id_;
  marker.header.stamp = carrot.stamp_;
  marker.ns = "carrot";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  tf2::toMsg(carrot, marker.pose);
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
}  // namespace ruvu_carrot_local_planner
