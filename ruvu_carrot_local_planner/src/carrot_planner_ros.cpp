// Copyright 2019 RUVU BV.

#include "./carrot_planner_ros.h"

#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

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
  base_local_planner::LocalPlannerLimits limits;
  limits.max_trans_vel = config.max_trans_vel;
  limits.min_trans_vel = config.min_trans_vel;
  limits.max_vel_x = config.max_vel_x;
  limits.min_vel_x = config.min_vel_x;
  limits.max_vel_y = config.max_vel_y;
  limits.min_vel_y = config.min_vel_y;
  limits.max_rot_vel = config.max_rot_vel;
  limits.min_rot_vel = config.min_rot_vel;
  limits.acc_lim_x = config.acc_lim_x;
  limits.acc_lim_y = config.acc_lim_y;
  limits.acc_lim_theta = config.acc_lim_theta;
  limits.acc_limit_trans = config.acc_limit_trans;
  limits.xy_goal_tolerance = config.xy_goal_tolerance;
  limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
  limits.prune_plan = config.prune_plan;
  limits.trans_stopped_vel = config.trans_stopped_vel;
  limits.rot_stopped_vel = config.rot_stopped_vel;
  planner_util_.reconfigureCB(limits, config.restore_defaults);

  // initialize parameters specific to this module
  CarrotPlanner::Parameters parameters;
  parameters.carrot_distance = config.carrot_distance;
  parameters.p_angle = config.p_angle;
  parameters.slow_down_margin = config.slow_down_margin;
  parameters.sim_time = config.sim_time;
  parameters.sim_granularity = config.sim_granularity;
  parameters.angular_sim_granularity = config.angular_sim_granularity;
  parameters.occdist_scale = config.occdist_scale;
  parameters.scaling_speed = config.scaling_speed;
  parameters.max_scaling_factor = config.max_scaling_factor;
  parameters.use_dwa = config.use_dwa;

  carrot_planner_->reconfigure(parameters);
}

void CarrotPlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!isInitialized())
  {
    ros::NodeHandle private_nh("~/" + name);
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    // make sure to update the costmap we'll use for this cycle
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

    planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

    // create the actual planner that we'll use.. it'll configure itself from the parameter server
    carrot_planner_.reset(new CarrotPlanner(private_nh, &planner_util_));

    if (private_nh.getParam("odom_topic", odom_topic_))
    {
      odom_helper_.setOdomTopic(odom_topic_);
    }

    initialized_ = true;

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
  tf::Stamped<tf::Pose> current_pose_;
  if (!costmap_ros_->getRobotPose(current_pose_))
  {
    ROS_ERROR_NAMED("ruvu_carrot_local_planner", "Could not get robot pose");
    return false;
  }

  if (latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_))
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

  // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
  carrot_planner_->updatePlanAndLocalCosts(robot_pose, transformed_plan, costmap_ros_->getRobotFootprint());

  tf::Stamped<tf::Pose> robot_pose_tf;
  tf::poseStampedMsgToTF(robot_pose, robot_pose_tf);
  if (latchedStopRotateController_.isPositionReached(&planner_util_, robot_pose_tf))
  {
    // Publish an empty plan because we've reached our goal position
    std::vector<geometry_msgs::PoseStamped> local_plan;
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    publishGlobalPlan(transformed_plan);
    publishLocalPlan(local_plan);
    auto limits = planner_util_.getCurrentLimits();
    if (latchedStopRotateController_.computeVelocityCommandsStopRotate(
            cmd_vel.twist, limits.getAccLimits(), carrot_planner_->getSimPeriod(), &planner_util_, odom_helper_,
            robot_pose_tf, boost::bind(&CarrotPlanner::checkTrajectory, carrot_planner_.get(), _1, _2, _3)))
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
