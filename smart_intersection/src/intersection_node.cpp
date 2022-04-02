#include "intersection.h"
 
using namespace smart_intersection;
// {

int main(int argc, char** argv)
{
  ros::init(argc, argv, "intersection_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  tf2::Transform pose;
  pose.setIdentity();
  Intersection intersection(pose);
  geometry_msgs::PoseStamped vehicle_pose;
  vehicle_pose.pose.position.y = 50;
  vehicle_pose.header.stamp = ros::Time::now();
  intersection.getTrajectory(vehicle_pose, 20.0, Direction::UP);
  ros::spin();
  return 0;
}
// }