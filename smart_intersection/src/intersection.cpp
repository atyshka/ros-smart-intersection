#include "intersection.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace smart_intersection
{

  Intersection::Intersection(const tf2::Transform pose, double distance, double speed, double interval)
      : distance_(distance), speed_(speed), interval_(interval), pose_(pose)
  {
  }

  void Intersection::generateTrajectory(std::vector<double> &pos_out, double pos_f, double vel_0, double vel_f, double duration, int resolution)
  {
    ROS_INFO("pos final %f, vel0 %f, velf %f, duration %f, resolution %d", pos_f, vel_0, vel_f, duration, resolution);
    double a = (-2 / (duration * duration * duration)) * (pos_f) + (1 / (duration * duration)) * (vel_0 + vel_f);
    double b = (3 / (duration * duration)) * pos_f - (2 / duration) * vel_0 - (1 / duration) * vel_f;
    double c = vel_0;
    ROS_INFO("a %f, b %f, c %f\n", a, b, c);
    for (int i = 0; i < (int)(duration * resolution); i++)
    {
      double t = i / (double)resolution;
      pos_out.push_back(a * t * t * t + b * t * t + c * t);
    }
  }

  void Intersection::getTrajectory(std::vector<geometry_msgs::PoseStamped> &result, const geometry_msgs::PoseStamped &pose, double speed, const Direction &direction)
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
    // tf2::Vector3 approach_point_global = pose_ * approach_point_rel;
    double projected_toa = pose.header.stamp.toSec() + distance_ / target_speed;
    if (direction == Direction::UP || direction == Direction::DOWN)
    {
      projected_toa -= (interval_ / 2);
    }
    // align to n-second interval
    double projected_toa_aligned = interval_ * ceil(projected_toa / interval_);
    uint64_t key = projected_toa_aligned * 1000;
    int idx = static_cast<std::underlying_type<Direction>::type>(direction);
    while (slotmap_[key][idx])
    {
      ROS_INFO("Slot already occupied, delaying");
      key += interval_ * 1000;
    }
    slotmap_[key][idx] = true;
    ROS_INFO("Added slot at %lu", key);
    std::vector<double> pos, vel;
    std::stringstream ss;
    // this generates the first half, cubic in 1d space, 0-1
    generateTrajectory(pos, distance_, speed, target_speed, projected_toa_aligned - pose.header.stamp.toSec(), 100);
    tf2::Vector3 intersection_center;
    // second half, 1-2
    for (int i = 0; i < (100 * distance_ / target_speed); i++)
    {
      pos.push_back(1.0 + (double)i / (100 * distance_ / target_speed));
    }
    for (int i = 0; i < pos.size(); i++)
    {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.stamp = ros::Time(projected_toa_aligned + i / 100.0);
      tf2::Vector3 position = (intersection_center * (pos[i] / distance_) + approach_point_rel * ((distance_ - pos[i]) / distance_));
      tf2::toMsg(position, pose_msg.pose.position);
      result.push_back(pose_msg);
    }
  }
} // namespace smart_intersection