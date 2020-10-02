// Copyright 2019 RUVU BV.

#pragma once

#include <mbf_costmap_core/costmap_controller.h>
#include <ruvu_carrot_local_planner/CarrotPlannerConfig.h>

#include "./local_planner_util.hpp"
#include "./latched_stop_rotate_controller.hpp"

namespace ruvu_carrot_local_planner
{
// forward declare
class CarrotPlanner;
class Simulator;

/**
 * @class CarrotPlannerROS
 * @brief ROS Wrapper for the CarrotPlanner that adheres to the
 * BaseLocalPlanner interface and can be used as a plugin for move_base.
 */
class CarrotPlannerROS : public mbf_costmap_core::CostmapController
{
public:
  /**
   * @brief Constructs the local planner
   * @param name The name to give this instance of the local planner
   * @param tf A pointer to a transform listener
   * @param costmap_ros The cost map to use for assigning costs to local plans
   */
  void initialize(std::string name, TF* tf, costmap_2d::Costmap2DROS* costmap_ros) override;

  /**
   * @brief Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base
   * @remark New on MBF API; replaces version without output code and message
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
   * @param message Optional more detailed outcome as a string
   * @return Result code as described on ExePath action result, as this is a wrapper to the nav_core,
   *         only 0 (SUCCESS) and 100 (FAILURE) are supported.
   */
  uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& robot_pose,
                                   const geometry_msgs::TwistStamped& robot_velocity,
                                   geometry_msgs::TwistStamped& cmd_vel, std::string& message) override;

  /**
   * @brief  Set the plan that the controller is following
   * @param orig_global_plan The plan to pass to the controller
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;

  /**
   * @brief Check if the goal pose has been achieved by the local planner within tolerance limits
   * @remark New on MBF API
   * @param xy_tolerance Distance tolerance in meters
   * @param yaw_tolerance Heading tolerance in radians
   * @return True if achieved, false otherwise
   */
  bool isGoalReached(double xy_tolerance, double yaw_tolerance) override;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time
   * @remark New on MBF API
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  bool cancel() override;

  bool isInitialized()
  {
    return initialized_;
  }

private:
  /**
   * @brief Callback to update the local planner's parameters based on dynamic reconfigure
   */
  void reconfigureCB(CarrotPlannerConfig& config, uint32_t level);

  void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

  void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

  // for visualisation, publishers of global and local plan
  ros::Publisher g_plan_pub_, l_plan_pub_;

  LocalPlannerUtil planner_util_;

  std::unique_ptr<CarrotPlanner> carrot_planner_;
  std::unique_ptr<Simulator> simulator_;

  costmap_2d::Costmap2DROS* costmap_ros_;

  std::unique_ptr<dynamic_reconfigure::Server<CarrotPlannerConfig>> dsrv_;
  ruvu_carrot_local_planner::CarrotPlannerConfig default_config_;
  bool setup_ = false;

  std::unique_ptr<LatchedStopRotateController> latchedStopRotateController_;
  bool is_overshoot = false;

  bool initialized_ = false;

  base_local_planner::OdometryHelperRos odom_helper_{ "odom" };
  std::string odom_topic_;
};
}  // namespace ruvu_carrot_local_planner
