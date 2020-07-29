// Copyright 2019 RUVU BV.

#pragma once

#include <ros/node_handle.h>

namespace ruvu_carrot_local_planner
{
/**
 * Find element in iterator with the minimum calculated value
 */
template <typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end)
    return end;
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it)
  {
    auto comp = getCompareVal(*it);
    if (comp < lowest)
    {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

/**
 * Compute the sign of a number
 * https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
 *
 * @return -1 or 1 depending on the number
 */
template <typename T>
int sgn(T val)
{
  ROS_INFO_STREAM("Compare" << (T(0) <= val) << (val < T(0)));
  return (T(0) <= val) - (val < T(0));
}

/**
 * Compute the signum of a number
 * https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
 *
 * * @return -1 0 or 1 depending on the number
 */
template <typename T>
int signum(T val)
{
  return (T(0) < val) - (val < T(0));
}

double getSimPeriodParam(ros::NodeHandle private_nh);
}  // namespace ruvu_carrot_local_planner
