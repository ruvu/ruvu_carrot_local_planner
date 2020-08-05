// Copyright 2019 RUVU BV.

#pragma once

#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>
#include <base_local_planner/simple_trajectory_generator.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>

namespace ruvu_carrot_local_planner
{
// forward declare
class LocalPlannerUtil;

class Simulator
{
public:
  struct Parameters
  {
    double sim_time;
    double sim_granularity;
    double angular_sim_granularity;
    double occdist_scale;
    double scaling_speed;
    double max_scaling_factor;
    bool use_dwa;
  };

  Simulator(ros::NodeHandle private_nh, LocalPlannerUtil* planner_util);

  /**
   * @brief Reconfigures the carrot planner
   */
  void reconfigure(const Parameters& parameters);

  /**
   * @brief  Update the cost functions before planning
   * @param  new_plan The new global plan
   * @param  footprint_spec The robot's footprint
   *
   * The obstacle cost function gets the footprint.
   * The global_plan is used during forward simulation
   */
  void updatePlanAndFootprint(const std::vector<geometry_msgs::PoseStamped>& new_plan,
                              const std::vector<geometry_msgs::Point>& footprint_spec);

  /**
   * @brief Get the period at which the local planner is expected to run
   * @return The simulation period
   */
  double getSimPeriod()
  {
    return sim_period_;
  }

  /**
   * @brief  Check if a trajectory is legal for a position/velocity pair
   * @param pos The robot's position
   * @param vel The robot's velocity
   * @param vel_samples The desired velocity
   * @return True if the trajectory is valid, false otherwise
   */
  bool checkTrajectory(const Eigen::Vector3f pos, const Eigen::Vector3f vel, const Eigen::Vector3f vel_samples);

  base_local_planner::Trajectory simulateVelocity(const tf::Stamped<tf::Pose>& global_pose,
                                                  const geometry_msgs::Twist& global_vel,
                                                  geometry_msgs::Twist& cmd_vel);

private:
  base_local_planner::Trajectory simulateVelocity(Eigen::Vector3f pos, Eigen::Vector3f vel,
                                                  Eigen::Vector3f vel_samples);

  // input to the algorithm
  LocalPlannerUtil* planner_util_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  Parameters parameters_;
  double sim_period_;

  base_local_planner::SimpleTrajectoryGenerator generator_;
  base_local_planner::ObstacleCostFunction obstacle_costs_;
  base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;

  // utilities
  boost::mutex configuration_mutex_;
};
}  // namespace ruvu_carrot_local_planner
