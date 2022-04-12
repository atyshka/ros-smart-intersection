#include "intersection.h"

using namespace smart_intersection;
// {

int main(int argc, char **argv)
{
  ros::init(argc, argv, "intersection_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  tf2::Transform pose;

  //load the necessary parameters from the launch file, in case we want to later have
  //more than one intersection. 
  double x_pose, y_pose;
  int intersection_id;

  bool x_set = nh_private.getParam("pose_x", x_pose);
  bool y_set = nh_private.getParam("pose_y", y_pose);
  bool id_set = nh_private.getParam("intersection_id", intersection_id);

  //double check that everything is loaded correctly, if not shout at programmer
  if (x_set && y_set && id_set)
    ROS_INFO("Initilization complete for intersection %d at position (%f, %f)", intersection_id, x_pose, y_pose);
  else
    ROS_ERROR("Could not load parameters intersection");

  // pose.setIdentity();
  //set pose from the launch file parameter
  pose.setOrigin(tf2::Vector3(x_set, y_set, 0));

  Intersection intersection(pose);
  geometry_msgs::PoseStamped vehicle_pose;
  vehicle_pose.pose.position.y = 50;
  vehicle_pose.header.stamp = ros::Time::now();
  intersection.getTrajectory(vehicle_pose, 20.0, Direction::UP);
  ros::spin();
  return 0;
}
// }