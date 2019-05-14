// Copyright 2019 RUVU BV.

#pragma once

#include <base_local_planner/local_planner_util.h>

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
  };

  enum class Outcome
  {
    OK = 0,
    MISSED_GOAL = 107,
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

  Outcome computeVelocityCommands(const std::vector<geometry_msgs::PoseStamped>& path,
                                  const tf::Stamped<tf::Pose>& global_pose, const geometry_msgs::Twist& global_vel,
                                  geometry_msgs::Twist& cmd_vel, std::string& message);

  /**
   * @brief Get the period at which the local planner is expected to run
   * @return The simulation period
   */
  double getSimPeriod()
  {
    return sim_period_;
  }

private:
  enum class State
  {
    DRIVING = 1,
    ARRIVING = 2,
  } state_ = State::DRIVING;

  void computeCarrot(const std::vector<geometry_msgs::PoseStamped>& path,
                     std::vector<geometry_msgs::PoseStamped>::const_iterator it, tf::Stamped<tf::Pose>& carrot);

  void publishDebugCarrot(const tf::Stamped<tf::Pose>& carrot);

  base_local_planner::LocalPlannerUtil* planner_util_;
  Parameters parameters_;
  double sim_period_;

  double arriving_angle_;
  ros::Publisher debug_pub_;
};
}  // namespace ruvu_carrot_local_planner
