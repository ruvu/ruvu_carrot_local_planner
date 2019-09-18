// Copyright 2019 RUVU BV.

#pragma once

#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

namespace ruvu_carrot_local_planner
{
class CarrotPlanner
{
public:
  struct Parameters
  {
    double carrot_distance;
    double p_angle;
    double slow_down_margin;
    double sim_time;
    double sim_granularity;
    double angular_sim_granularity;
    double occdist_scale;
    double scaling_speed;
    double max_scaling_factor;
    bool use_dwa;
  };

  enum class Outcome : uint32_t
  {
    OK = 0,
    MISSED_GOAL = 107,
    BLOCKED_PATH = 109,
  };

  /**
   * @brief  Constructor for the planner
   * @param name The name of the planner
   * @param costmap_ros A pointer to the costmap instance the planner should use
   * @param global_frame the frame id of the tf frame to use
   */
  CarrotPlanner(ros::NodeHandle private_nh, base_local_planner::LocalPlannerUtil* planner_util);

  /**
   * @brief Reconfigures the carrot planner
   */
  void reconfigure(const Parameters& parameters);

  /**
   * sets new plan and resets state
   */
  void setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief  Update the cost functions before planning
   * @param  global_pose The robot's current pose
   * @param  new_plan The new global plan
   * @param  footprint_spec The robot's footprint
   *
   * The obstacle cost function gets the footprint.
   * The path and goal cost functions get the global_plan
   * The alignment cost functions get a version of the global plan
   *   that is modified based on the global_pose
   */
  void updatePlanAndLocalCosts(tf::Stamped<tf::Pose> global_pose,
                               const std::vector<geometry_msgs::PoseStamped>& new_plan,
                               const std::vector<geometry_msgs::Point>& footprint_spec);

  Outcome computeVelocityCommands(const tf::Stamped<tf::Pose>& global_pose, const geometry_msgs::Twist& global_vel,
                                  geometry_msgs::Twist& cmd_vel, std::string& message,
                                  base_local_planner::Trajectory& trajectory);

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

private:
  enum class State
  {
    DRIVING = 1,
    ARRIVING = 2,
  } state_ = State::DRIVING;

  void computeCarrot(const std::vector<geometry_msgs::PoseStamped>& path,
                     std::vector<geometry_msgs::PoseStamped>::const_iterator it, tf::Stamped<tf::Pose>& carrot);

  base_local_planner::Trajectory simulateVelocity(const tf::Stamped<tf::Pose>& global_pose,
                                                  const geometry_msgs::Twist& global_vel,
                                                  geometry_msgs::Twist& cmd_vel);
  base_local_planner::Trajectory simulateVelocity(Eigen::Vector3f pos, Eigen::Vector3f vel,
                                                  Eigen::Vector3f vel_samples);

  void publishDebugCarrot(const tf::Stamped<tf::Pose>& carrot);

  // input to the algorithm
  base_local_planner::LocalPlannerUtil* planner_util_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  Parameters parameters_;
  double sim_period_;

  // local state
  double arriving_angle_;

  // utilities
  boost::mutex configuration_mutex_;

  base_local_planner::SimpleTrajectoryGenerator generator_;
  base_local_planner::ObstacleCostFunction obstacle_costs_;
  base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;

  ros::Publisher debug_pub_;
};
}  // namespace ruvu_carrot_local_planner
