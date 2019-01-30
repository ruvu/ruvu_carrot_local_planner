#pragma once

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <ruvu_carrot_local_planner/CarrotPlannerConfig.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

namespace ruvu_carrot_local_planner
{
struct CarrotPlannerParameters
{
  double carrot_distance;
  double p_angle;
};

/**
 * @class CarrotPlannerROS
 * @brief ROS Wrapper for the CarrotPlanner that adheres to the
 * BaseLocalPlanner interface and can be used as a plugin for move_base.
 */
class CarrotPlannerROS : public nav_core::BaseLocalPlanner
{
public:
  /**
   * @brief  Constructor for CarrotPlannerROS wrapper
   */
  CarrotPlannerROS();

  /**
   * @brief  Constructs the ros wrapper
   * @param name The name to give this instance of the trajectory planner
   * @param tf A pointer to a transform listener
   * @param costmap The cost map to use for assigning costs to trajectories
   */
  void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Destructor for the wrapper
   */
  ~CarrotPlannerROS();

  /**
   * @brief  Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
   * @return True if a valid trajectory was found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief  Set the plan that the controller is following
   * @param orig_global_plan The plan to pass to the controller
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief  Check if the goal pose has been achieved
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

  bool isInitialized()
  {
    return initialized_;
  }

private:
  /**
   * @brief Helper function
   */
  double getSimPeriodParam(ros::NodeHandle private_nh);

  /**
   * @brief Callback to update the local planner's parameters based on dynamic reconfigure
   */
  void reconfigureCB(CarrotPlannerConfig& config, uint32_t level);

  void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

  void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

  bool carrotComputeVelocityCommands(const std::vector<geometry_msgs::PoseStamped>& path,
                                     const tf::Stamped<tf::Pose>& global_pose, geometry_msgs::Twist& cmd_vel);

  void computeCarrot(const std::vector<geometry_msgs::PoseStamped>& path, const tf::Stamped<tf::Pose>& global_pose,
                     tf::Stamped<tf::Pose>& carrot, double& goal_distance);

  tf::TransformListener* tf_;  ///< @brief Used for transforming point clouds

  // for visualisation, publishers of global and local plan
  ros::Publisher g_plan_pub_, l_plan_pub_, debug_pub_;

  base_local_planner::LocalPlannerUtil planner_util_;

  costmap_2d::Costmap2DROS* costmap_ros_;

  dynamic_reconfigure::Server<CarrotPlannerConfig>* dsrv_;
  ruvu_carrot_local_planner::CarrotPlannerConfig default_config_;
  bool setup_ = false;

  base_local_planner::LatchedStopRotateController latchedStopRotateController_;

  bool initialized_ = false;
  bool goal_reached_ = false;

  base_local_planner::OdometryHelperRos odom_helper_;
  std::string odom_topic_;

  CarrotPlannerParameters parameters;
  double sim_period_;
};
};
