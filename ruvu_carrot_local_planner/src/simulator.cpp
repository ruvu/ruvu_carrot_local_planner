#include "simulator.h"

#include "./local_planner_util.h"
#include "./utils.h"

namespace ruvu_carrot_local_planner
{
Simulator::Simulator(ros::NodeHandle private_nh, LocalPlannerUtil* planner_util)
  : planner_util_(planner_util), sim_period_(getSimPeriodParam(private_nh)), obstacle_costs_(planner_util->getCostmap())
{
  // set up all the cost functions that will be applied in order
  std::vector<base_local_planner::TrajectoryCostFunction*> critics;
  critics.push_back(&obstacle_costs_);  // discards trajectories that move into obstacles

  // trajectory generators
  std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
  generator_list.push_back(&generator_);

  scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);
}

void Simulator::reconfigure(const Parameters& parameters)
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

void Simulator::updatePlanAndFootprint(const std::vector<geometry_msgs::PoseStamped>& new_plan,
                                       const std::vector<geometry_msgs::Point>& footprint_spec)
{
  global_plan_.resize(new_plan.size());
  for (unsigned int i = 0; i < new_plan.size(); ++i)
  {
    global_plan_[i] = new_plan[i];
  }

  obstacle_costs_.setFootprint(footprint_spec);
}

bool Simulator::checkTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples)
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

base_local_planner::Trajectory Simulator::simulateVelocity(const tf::Stamped<tf::Pose>& global_pose,
                                                           const geometry_msgs::Twist& global_vel,
                                                           geometry_msgs::Twist& cmd_vel)
{
  double yaw = tf::getYaw(global_pose.getRotation());
  Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw);
  Eigen::Vector3f vel(global_vel.linear.x, global_vel.linear.y, global_vel.angular.z);
  Eigen::Vector3f vel_samples(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

  return simulateVelocity(pos, vel, vel_samples);
}

base_local_planner::Trajectory Simulator::simulateVelocity(Eigen::Vector3f pos, Eigen::Vector3f vel,
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
}  // namespace ruvu_carrot_local_planner
