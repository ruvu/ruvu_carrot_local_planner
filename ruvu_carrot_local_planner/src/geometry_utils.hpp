// Copyright 2019 RUVU BV.

#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ruvu_carrot_local_planner
{
inline double distance2(const geometry_msgs::Point& pos1, const geometry_msgs::Point& pos2)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;
  double dz = pos1.z - pos2.z;
  return dx * dx + dy * dy + dz * dz;
}

inline double distance2(const geometry_msgs::Pose& pos1, const geometry_msgs::Pose& pos2)
{
  return distance2(pos1.position, pos2.position);
}

inline double distance2(const geometry_msgs::PoseStamped& pos1, const geometry_msgs::PoseStamped& pos2)
{
  return distance2(pos1.pose, pos2.pose);
}
}  // namespace ruvu_carrot_local_planner
