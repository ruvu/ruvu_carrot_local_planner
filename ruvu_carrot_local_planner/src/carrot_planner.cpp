// Copyright 2019 RUVU BV.

#include "./carrot_planner.h"

#include <base_local_planner/goal_functions.h>
#include <visualization_msgs/MarkerArray.h>
#include <mbf_utility/navigation_utility.h>
#include <tf2/utils.h>

#include "./utils.h"

namespace ruvu_carrot_local_planner
{
CarrotPlanner::CarrotPlanner(ros::NodeHandle private_nh, LocalPlannerUtil* planner_util)
  : planner_util_(planner_util)
  , sim_period_(getSimPeriodParam(private_nh))
  , obstacle_costs_(planner_util->getCostmap())
  , debug_pub_(private_nh.advertise<visualization_msgs::MarkerArray>("visualization", 1))
{
  // set up all the cost functions that will be applied in order
  std::vector<base_local_planner::TrajectoryCostFunction*> critics;
  critics.push_back(&obstacle_costs_);  // discards trajectories that move into obstacles

  // trajectory generators
  std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
  generator_list.push_back(&generator_);

  scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);
}

void CarrotPlanner::reconfigure(const Parameters& parameters)
{
  boost::mutex::scoped_lock l(configuration_mutex_);

  parameters_ = parameters;
  generator_.setParameters(parameters.sim_time, parameters.sim_granularity, parameters.angular_sim_granularity,
                           parameters.use_dwa, sim_period_);

  // obstacle costs can vary due to scaling footprint feature
  obstacle_costs_.setScale(parameters.occdist_scale);
  obstacle_costs_.setParams(planner_util_->getCurrentLimits().max_vel_trans, parameters.max_scaling_factor,
                            parameters.scaling_speed);
}

void CarrotPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  state_ = State::DRIVING;
}

void CarrotPlanner::updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
                                            const std::vector<geometry_msgs::PoseStamped>& new_plan,
                                            const std::vector<geometry_msgs::Point>& footprint_spec)
{
  global_plan_.resize(new_plan.size());
  for (unsigned int i = 0; i < new_plan.size(); ++i)
  {
    global_plan_[i] = new_plan[i];
  }

  obstacle_costs_.setFootprint(footprint_spec);
}

CarrotPlanner::Outcome CarrotPlanner::computeVelocityCommands(const tf::Stamped<tf::Pose>& global_pose,
                                                              const geometry_msgs::Twist& global_vel,
                                                              geometry_msgs::Twist& cmd_vel, std::string& message,
                                                              base_local_planner::Trajectory& trajectory)
{
  geometry_msgs::PoseStamped gp;
  tf::poseStampedTFToMsg(global_pose, gp);

  // Look for the closest point on the path
  auto closest = min_by(global_plan_.begin(), global_plan_.end(),
                        [&gp](const geometry_msgs::PoseStamped& ps) { return mbf_utility::distance(gp, ps); });

  tf::Point goal;
  tf::pointMsgToTF(global_plan_.back().pose.position, goal);
  auto goal_error = goal - global_pose.getOrigin();
  double goal_distance = goal_error.length();

  if (goal_distance < parameters_.carrot_distance && state_ != State::ARRIVING)
  {
    ROS_INFO_NAMED("ruvu_carrot_local_planner", "I'm close to the goal, let's go to the arriving state");
    state_ = State::ARRIVING;
    arriving_angle_ = atan2(goal_error.getY(), goal_error.getX());
  }

  tf::Stamped<tf::Pose> carrot(tf::Pose::getIdentity(), global_pose.stamp_, global_pose.frame_id_);
  switch (state_)
  {
    case State::DRIVING:
      message = "Driving";
      computeCarrot(global_plan_, closest, carrot);
      break;
    case State::ARRIVING:
    {
      message = "Arriving";
      // The carrot walks along the arriving angle from the goal
      auto direction = tf::quatRotate(tf::createQuaternionFromYaw(arriving_angle_),
                                      tf::Vector3(parameters_.carrot_distance - goal_distance, 0, 0));
      if (direction.dot(goal_error) < 0)
      {
        message = "Goal overshoot detected";
        ROS_WARN_STREAM_THROTTLE_NAMED(5, "ruvu_carrot_local_planner", message);
        return Outcome::MISSED_GOAL;
      }
      carrot.setOrigin(goal + direction);
      break;
    }
    default:
      assert(false);
  }

  publishDebugCarrot(carrot);

  auto limits = planner_util_->getCurrentLimits();

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

  cmd_vel.angular.z = parameters_.p_angle * carrot_error;

  if (switch_direction)
  {
    cmd_vel.linear.x = -cmd_vel.linear.x;
  }

  // Scale back the forward velocity when turning faster
  {
    double brake_factor = 1 - fabs(cmd_vel.angular.z / limits.max_vel_theta);
    brake_factor = brake_factor < 0 ? 0 : brake_factor;
    cmd_vel.linear.x *= brake_factor;
  }

  if (fabs(cmd_vel.angular.z) > limits.max_vel_theta)
  {
    cmd_vel.angular.z = cmd_vel.angular.z * limits.max_vel_theta / fabs(cmd_vel.angular.z);
  }

  // Apply some motion limits
  cmd_vel.linear.x = std::min(cmd_vel.linear.x, limits.max_vel_x);
  cmd_vel.linear.x = std::max(cmd_vel.linear.x, limits.min_vel_x);
  cmd_vel.linear.x = std::min(cmd_vel.linear.x, limits.max_vel_trans);
  cmd_vel.linear.x = std::max(cmd_vel.linear.x, -limits.max_vel_trans);

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

  // Apply min_vel_trans limit
  cmd_vel.linear.x = std::abs(cmd_vel.linear.x) < limits.min_vel_trans ? sgn(cmd_vel.linear.x) * limits.min_vel_trans :
                                                                         cmd_vel.linear.x;

  // check if that cmd_vel collides with an obstacle in the future
  trajectory = simulateVelocity(global_pose, global_vel, cmd_vel);

  if (trajectory.cost_ < 0)
    return Outcome::BLOCKED_PATH;
  else
    return Outcome::OK;
}

void CarrotPlanner::computeCarrot(const std::vector<geometry_msgs::PoseStamped>& path,
                                  std::vector<geometry_msgs::PoseStamped>::const_iterator it,
                                  tf::Stamped<tf::Pose>& carrot)
{
  // Walk along the path forward and count the distance. When carrot_distance has been walked, the carrot is found.
  double distance = parameters_.carrot_distance;

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

bool CarrotPlanner::checkTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples)
{
  base_local_planner::Trajectory traj = simulateVelocity(pos, vel, vel_samples);

  // if the trajectory is a legal one... the check passes
  if (traj.cost_ >= 0)
  {
    return true;
  }
  ROS_WARN_NAMED("ruvu_carrot_local_planner", "Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1],
                 vel_samples[2], traj.cost_);

  // otherwise the check fails
  return false;
}

base_local_planner::Trajectory CarrotPlanner::simulateVelocity(const tf::Stamped<tf::Pose>& global_pose,
                                                               const geometry_msgs::Twist& global_vel,
                                                               geometry_msgs::Twist& cmd_vel)
{
  double yaw = tf::getYaw(global_pose.getRotation());
  Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw);
  Eigen::Vector3f vel(global_vel.linear.x, global_vel.linear.y, global_vel.angular.z);
  Eigen::Vector3f vel_samples(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

  return simulateVelocity(pos, vel, vel_samples);
}

base_local_planner::Trajectory CarrotPlanner::simulateVelocity(Eigen::Vector3f pos, Eigen::Vector3f vel,
                                                               Eigen::Vector3f vel_samples)
{
  geometry_msgs::PoseStamped goal_pose = global_plan_.back();
  Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
  auto limits = static_cast<base_local_planner::LocalPlannerLimits>(planner_util_->getCurrentLimits());

  Eigen::Vector3f vsamples(0, 0, 0);
  generator_.initialise(pos, vel, goal, &limits, vsamples);

  base_local_planner::Trajectory traj;

  // sim_time == 0 is a feature to disable collision checking
  if (parameters_.sim_time == 0)
  {
    traj.cost_ = 0;
    traj.resetPoints();
    return traj;
  }

  if (generator_.generateTrajectory(pos, vel, vel_samples, traj))
    traj.cost_ = scored_sampling_planner_.scoreTrajectory(traj, -1);
  else
    ROS_WARN_STREAM_NAMED("ruvu_carrot_local_planner", "Carot controller generated an invalid trajectory");

  return traj;
}

void CarrotPlanner::publishDebugCarrot(const tf::Stamped<tf::Pose>& carrot)
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
}  // namespace ruvu_carrot_local_planner
