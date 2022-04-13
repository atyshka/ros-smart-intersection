#include "intersection_node.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <smart_intersection/GuidedPath.h>

namespace smart_intersection
{

  IntersectionNode::IntersectionNode(ros::NodeHandle &n, ros::NodeHandle &pn) : tf_listener_(buf_)
  {
    std::string intersection_id;
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
    sub_path_req_ = n.subscribe("/path_req", 10, &IntersectionNode::recvPathRequest, this);
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