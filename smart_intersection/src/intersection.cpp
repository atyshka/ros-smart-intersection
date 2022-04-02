#include "intersection.h"

namespace smart_intersection
{

Intersection::Intersection(const tf2::Transform pose, double distance, double speed, double interval)
  : distance_(distance), speed_(speed), interval_(interval), pose_(pose)
{

}

void Intersection::generateTrajectory(std::vector<double>& pos_out, std::vector<double>& vel_out, double pos_f, double vel_0, double vel_f, double duration, int resolution)
{
  double a = (2 * pos_f - duration * (vel_f - vel_0)) / (- duration * duration * duration);
  double b = (vel_f - vel_0 - 3 * a * duration * duration) / (2 * duration);
  double c = vel_0;
  for (int i = 0; i < (int)(duration * resolution); i++)
  {
    double t = i / (double)resolution;
    pos_out.push_back(a * t * t * t + b * t * t + c * t);
    vel_out.push_back(3 * a * t * t + 2 * b * t + c);
  }
}

void Intersection::getTrajectory(const geometry_msgs::PoseStamped& pose, double speed, const Direction& direction)
{
  double target_speed = (speed + speed_) / 2.0;
  // approach point relative to intersection frame
  tf2::Vector3 approach_point_rel;
  switch (direction)
  {
    case Direction::UP:
      approach_point_rel = tf2::Vector3(0, distance_, 0);
      break;
    case Direction::DOWN:
      approach_point_rel = tf2::Vector3(0, -distance_, 0);
      break;
    case Direction::LEFT:
      approach_point_rel = tf2::Vector3(-distance_, 0, 0);
      break;
    case Direction::RIGHT:
      approach_point_rel = tf2::Vector3(distance_, 0, 0);
      break;
  }
  // approach point in global frame
  tf2::Vector3 approach_point_global = pose_ * approach_point_rel;
  double projected_toa = pose.header.stamp.toSec() + distance_ / target_speed;
  if (direction == Direction::UP || direction == Direction::DOWN)
  {
    projected_toa -= (interval_ / 2);
  }
  // align to n-second interval
  double projected_toa_aligned = interval_ * ceil(projected_toa / interval_);
  uint64_t key = projected_toa_aligned * 1000;
  int idx = static_cast<std::underlying_type<Direction>::type>(direction);
  while(slotmap_[key][idx]) {
    ROS_INFO("Slot already occupied, delaying");
    key += interval_ * 1000;
  }
  slotmap_[key][idx] = true;
  ROS_INFO("Added slot at %lu", key);
  std::vector<double> pos, vel;
  std::stringstream ss;
  generateTrajectory(pos, vel, distance_, speed, target_speed, projected_toa_aligned - pose.header.stamp.toSec(), 10);
  for (int i = 0; i < pos.size(); i++)
  {
    ss << pos[i] << ", ";
  }
  ROS_INFO("size %d, content %s", pos.size(), ss.str().c_str());

  for (int i = 0; i < vel.size(); i++)
  {
    ss << vel[i] << ", ";
  }
  ROS_INFO("size %d, content %s", vel.size(), ss.str().c_str());
}

} // namespace smart_intersection