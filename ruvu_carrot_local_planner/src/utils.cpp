// Copyright 2019 RUVU BV.

#include "./utils.h"

namespace ruvu_carrot_local_planner
{
double getSimPeriodParam(ros::NodeHandle private_nh)
{
  // https://github.com/ros-planning/navigation/blob/melodic-devel/dwa_local_planner/src/dwa_planner_ros.cpp

  // Assuming this planner is being run within the navigation stack, we can
  // just do an upward search for the frequency at which its being run. This
  // also allows the frequency to be overwritten locally.
  double sim_period;
  std::string controller_frequency_param_name;
  if (!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
  {
    sim_period = 0.05;
  }
  else
  {
    double controller_frequency = 0;
    private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
    if (controller_frequency > 0)
    {
      sim_period = 1.0 / controller_frequency;
    }
    else
    {
      ROS_WARN_NAMED("ruvu_carrot_local_planner", "A controller_frequency less than 0 has been set. Ignoring the "
                                                  "parameter, assuming a rate of 20Hz");
      sim_period = 0.05;
    }
  }
  ROS_INFO_NAMED("ruvu_carrot_local_planner", "Sim period is set to %.2f", sim_period);

  return sim_period;
}

int sgn(double v)
{
  return (v > 0) - (v < 0);
}
}  // namespace ruvu_carrot_local_planner
