// Copyright 2019 RUVU BV.

#include "./carrot_planner_ros.h"

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>

#include "./carrot_planner.h"
#include "./simulator.h"
#include "./parameter_magic.h"
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
  LocalPlannerLimits limits;
  limits.max_vel_trans = config.max_vel_trans;
  limits.min_vel_trans = config.min_vel_trans;
  limits.max_vel_x = config.max_vel_x;
  limits.min_vel_x = config.min_vel_x;
  limits.max_vel_y = config.max_vel_y;
  limits.min_vel_y = config.min_vel_y;
  limits.max_vel_theta = config.max_vel_theta;
  limits.min_vel_theta = config.min_vel_theta;
  limits.acc_lim_x = config.acc_lim_x;
  limits.acc_lim_y = config.acc_lim_y;
  limits.acc_lim_theta = config.acc_lim_theta;
  limits.acc_lim_trans = config.acc_lim_trans;
  limits.xy_goal_tolerance = config.xy_goal_tolerance;
  limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
  limits.prune_plan = config.prune_plan;
  limits.trans_stopped_vel = config.trans_stopped_vel;
  limits.theta_stopped_vel = config.theta_stopped_vel;
  planner_util_.reconfigureCB(limits, config.restore_defaults);

  // initialize parameters specific to this module
  CarrotPlanner::Parameters parameters;
  parameters.carrot_distance = config.carrot_distance;
  parameters.p_angle = config.p_angle;
  parameters.slow_down_margin = config.slow_down_margin;
  carrot_planner_->reconfigure(parameters);

  Simulator::Parameters sim_parameters;
  sim_parameters.sim_time = config.sim_time;
  sim_parameters.sim_granularity = config.sim_granularity;
  sim_parameters.angular_sim_granularity = config.angular_sim_granularity;
  sim_parameters.occdist_scale = config.occdist_scale;
  sim_parameters.scaling_speed = config.scaling_speed;
  sim_parameters.max_scaling_factor = config.max_scaling_factor;
  sim_parameters.use_dwa = config.use_dwa;
  simulator_->reconfigure(sim_parameters);
}

void CarrotPlannerROS::initialize(std::string name, TF* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!isInitialized())
  {
    ros::NodeHandle private_nh("~/" + name);
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    costmap_ros_ = costmap_ros;

    // make sure to update the costmap we'll use for this cycle
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

    planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

    // create the actual planner that we'll use.. it'll configure itself from the parameter server
    simulator_.reset(new Simulator(private_nh, &planner_util_));
    carrot_planner_.reset(new CarrotPlanner(private_nh, simulator_.get(), &planner_util_));

    if (private_nh.getParam("odom_topic", odom_topic_))
    {
      odom_helper_.setOdomTopic(odom_topic_);
    }

    initialized_ = true;

    // Warn about deprecated parameters
    warnRenamedParameter(private_nh, "max_vel_trans", "max_trans_vel");
    warnRenamedParameter(private_nh, "min_vel_trans", "min_trans_vel");
    warnRenamedParameter(private_nh, "max_vel_theta", "max_rot_vel");
    warnRenamedParameter(private_nh, "min_vel_theta", "min_rot_vel");
    warnRenamedParameter(private_nh, "acc_lim_trans", "acc_limit_trans");
    warnRenamedParameter(private_nh, "theta_stopped_vel", "rot_stopped_vel");

    dsrv_.reset(new dynamic_reconfigure::Server<CarrotPlannerConfig>(private_nh));
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

  ROS_INFO_NAMED("ruvu_carrot_local_planner", "Got new plan");
  carrot_planner_->setPlan(orig_global_plan);
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

#ifdef USE_OLD_TF
  tf::Stamped<tf::Pose> current_pose;
#else
  geometry_msgs::PoseStamped current_pose;
#endif

  if (!costmap_ros_->getRobotPose(current_pose))
  {
    ROS_ERROR_NAMED("ruvu_carrot_local_planner", "Could not get robot pose");
    return false;
  }

  geometry_msgs::PoseStamped pose;
#ifdef USE_OLD_TF
  tf::poseStampedTFToMsg(current_pose, pose);
#else
  pose = current_pose;
#endif

  if (latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, pose))
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

uint32_t CarrotPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& robot_pose,
                                                   const geometry_msgs::TwistStamped& robot_velocity,
                                                   geometry_msgs::TwistStamped& cmd_vel, std::string& message)
{
  if (costmap_ros_->getGlobalFrameID() != robot_pose.header.frame_id)
  {
    ROS_ERROR_NAMED("ruvu_carrot_local_planner", "local costmap frame_id != robot_pose frame_id: %s != %s",
                    costmap_ros_->getGlobalFrameID().c_str(), robot_pose.header.frame_id.c_str());
    return static_cast<uint32_t>(CarrotPlanner::Outcome::TF_ERROR);
  }

  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  if (!planner_util_.getLocalPlan(robot_pose, transformed_plan))
  {
    ROS_ERROR_NAMED("ruvu_carrot_local_planner", "Could not get local plan");
    return 108;  // MISSED_PATH
  }

  // If the global plan passed in is empty... we won't do anything
  if (transformed_plan.empty())
  {
    ROS_WARN_NAMED("ruvu_carrot_local_planner", "Received an empty transformed plan.");
    return 108;  // MISSED_PATH
  }

  // update plan in carrot planner even if we just stop and rotate, to allow checkTrajectory
  carrot_planner_->updatePlan(transformed_plan);
  simulator_->updatePlanAndFootprint(transformed_plan, costmap_ros_->getRobotFootprint());

  // Dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close
  // enough to goal
  if (latchedStopRotateController_.isPositionReached(&planner_util_, robot_pose))
  {
    // Publish an empty plan because we've reached our goal position
    std::vector<geometry_msgs::PoseStamped> local_plan;
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    publishGlobalPlan(transformed_plan);
    publishLocalPlan(local_plan);
    auto limits = planner_util_.getCurrentLimits();
    if (latchedStopRotateController_.computeVelocityCommandsStopRotate(
            cmd_vel.twist, limits.getAccLimits(), simulator_->getSimPeriod(), &planner_util_, odom_helper_, robot_pose,
            boost::bind(&Simulator::checkTrajectory, simulator_.get(), _1, _2, _3)))
    {
      return 0;
    }
    else
    {
      return 100;
    }
  }
  else
  {
    tf::Stamped<tf::Pose> robot_pose_tf;
    tf::poseStampedMsgToTF(robot_pose, robot_pose_tf);
    nav_msgs::Odometry odom;
    odom_helper_.getOdom(odom);
    base_local_planner::Trajectory trajectory;
    auto outcome =
        carrot_planner_->computeVelocityCommands(robot_pose_tf, odom.twist.twist, cmd_vel.twist, message, trajectory);
    switch (outcome)
    {
      case CarrotPlanner::Outcome::OK:
        ROS_DEBUG_NAMED("ruvu_carrot_local_planner", "Computed the following cmd_vel: %.3lf, %.3lf, %.3lf",
                        cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);

        {
          std::vector<geometry_msgs::PoseStamped> local_plan;
          for (unsigned int i = 0; i < trajectory.getPointsSize(); ++i)
          {
            double p_x, p_y, p_th;
            trajectory.getPoint(i, p_x, p_y, p_th);

            tf::Stamped<tf::Pose> p =
                tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(p_th), tf::Point(p_x, p_y, 0.0)),
                                      ros::Time::now(), costmap_ros_->getGlobalFrameID());
            geometry_msgs::PoseStamped pose;
            tf::poseStampedTFToMsg(p, pose);
            local_plan.push_back(pose);
          }
          publishLocalPlan(local_plan);
        }
        publishGlobalPlan(transformed_plan);
        break;
      default:
        ROS_WARN_THROTTLE_NAMED(5, "ruvu_carrot_local_planner", "Carrot planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
        break;
    }
    return static_cast<uint32_t>(outcome);
  }
}
}  // namespace ruvu_carrot_local_planner
