#include "intersection_node.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <smart_intersection/GuidedPath.h>
#include <visualization_msgs/Marker.h>

namespace smart_intersection
{

  IntersectionNode::IntersectionNode(ros::NodeHandle &n, ros::NodeHandle &pn) : tf_listener_(buf_)
  {
    std::string intersection_id;
    ROS_INFO("Use system time: %d", (int)ros::Time::useSystemTime());
    ROS_INFO("Namespace: %s", pn.getNamespace().c_str());
    ROS_INFO("Param returned %d", (int)pn.getParam("intersection_id", intersection_id));
    frame_ = "intersection_" + intersection_id;
    tf2::Transform intersection_tf;
    try
    {
      geometry_msgs::TransformStamped intersection_tf_msg;
      intersection_tf_msg = buf_.lookupTransform("world", frame_, ros::Time(0), ros::Duration(5));
      tf2::fromMsg(intersection_tf_msg.transform, intersection_tf);
    }
    catch (const tf2::LookupException &e)
    {
      ROS_ERROR("No intersection frame published for intersection %s", intersection_id.c_str());
      throw e;
    }
    intersection_ = std::make_unique<smart_intersection::Intersection>(Intersection(intersection_tf));
    pub_path_ = n.advertise<GuidedPath>("/requested_path", 10);
    pub_raw_path_ = n.advertise<nav_msgs::Path>("/raw_path", 10, true);
    pub_markers_ = n.advertise<visualization_msgs::Marker>("/intersection_markers", 10, true);
    sub_path_req_ = n.subscribe("/path_req", 10, &IntersectionNode::recvPathRequest, this);
    pubMarkers();
  }

  void IntersectionNode::pubMarkers()
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = frame_;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    geometry_msgs::Point p;
    p.x = 50;
    p.y = 2;
    marker.points.push_back(p);
    p.x = -50;
    p.y = -2;
    marker.points.push_back(p);
    p.x = -2;
    p.y = 50;
    marker.points.push_back(p);
    p.x = 2;
    p.y = -50;
    marker.points.push_back(p);
    pub_markers_.publish(marker);
  }

  void IntersectionNode::recvPathRequest(const smart_intersection::PathRequest& path_req)
  {
    if (intersection_) {
      GuidedPath path_out;
      Direction direction;
      switch (path_req.direction)
      {
      case path_req.UP:
        direction = Direction::UP;
        break;
      case path_req.DOWN:
        direction = Direction::DOWN;
        break;
      case path_req.LEFT:
        direction = Direction::LEFT;
        break;
      case path_req.RIGHT:
        direction = Direction::RIGHT;
        break;
      default:
        break;
      }
      intersection_->getTrajectory(path_out.path.poses, path_req.pose, path_req.velocity, direction);
      path_out.header.frame_id = frame_;
      path_out.header.stamp = ros::Time::now();
      path_out.vehicle_id = path_req.vehicle_id;
      pub_path_.publish(path_out);
      path_out.path.header.frame_id = frame_;
      ROS_INFO("Use system time: %d", (int)ros::Time::useSystemTime());
      path_out.path.header.stamp = ros::Time::now();
      ROS_INFO("Path time: %f", path_out.path.header.stamp.toSec());
      ROS_INFO("Sim time: %d", (int)path_out.path.header.stamp.isSimTime());
      pub_raw_path_.publish(path_out.path);
    }
  }
}
// {
using namespace smart_intersection;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "intersection_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  IntersectionNode node(n, pn);
  ros::spin();
  return 0;
}
// }