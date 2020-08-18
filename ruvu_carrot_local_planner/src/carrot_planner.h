// Copyright 2019 RUVU BV.

#pragma once

#include <base_local_planner/trajectory.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>
#include <tf/transform_datatypes.h>

namespace ruvu_carrot_local_planner
{
// forward declare
class Simulator;
class LocalPlannerUtil;

class CarrotPlanner
{
public:
  struct Parameters
  {
    double carrot_distance;
    double carrot_distance_scaling;
    double p_angle;
    double slow_down_margin;
  };

  enum class Outcome : uint32_t
  {
    OK = 0,
    MISSED_GOAL = 107,
    BLOCKED_PATH = 109,
    TF_ERROR = 111,
  };

  /**
   * @brief  Constructor for the planner
   * @param name The name of the planner
   * @param costmap_ros A pointer to the costmap instance the planner should use
   * @param global_frame the frame id of the tf frame to use
   */
  CarrotPlanner(ros::NodeHandle private_nh, Simulator* simulator, LocalPlannerUtil* planner_util);

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
   * @param  new_plan The new global plan
   */
  void updatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan);

  Outcome computeVelocityCommands(const tf::Stamped<tf::Pose>& global_pose, const geometry_msgs::Twist& global_vel,
                                  geometry_msgs::Twist& cmd_vel, std::string& message,
                                  base_local_planner::Trajectory& trajectory);

private:
  enum class State
  {
    DRIVING = 1,
    ARRIVING = 2,
  } state_ = State::DRIVING;

  tf::Stamped<tf::Pose> computeCarrot(const std::vector<geometry_msgs::PoseStamped>& path,
                                      std::vector<geometry_msgs::PoseStamped>::const_iterator it,
                                      double carrot_distance);

  void publishDebugCarrot(const tf::Stamped<tf::Pose>& carrot);

  // input to the algorithm
  Simulator* simulator_;
  LocalPlannerUtil* planner_util_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  Parameters parameters_;
  double sim_period_;

  // local state
  double arriving_angle_;

  // utilities
  boost::mutex configuration_mutex_;

  ros::Publisher debug_pub_;
};
}  // namespace ruvu_carrot_local_planner
