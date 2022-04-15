#include <ros/ros.h>
#include "intersection.h"
#include <tf2_ros/transform_listener.h>
#include <smart_intersection/PathRequest.h>
#include <memory>

namespace smart_intersection
{

class IntersectionNode
{
  public:
    IntersectionNode(ros::NodeHandle& n, ros::NodeHandle& pn);
    void recvPathRequest(const smart_intersection::PathRequest& path_req);
  private:
    void pubMarkers();
    std::string frame_;
    tf2_ros::Buffer buf_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<Intersection> intersection_;
    std::map<std::string, bool> active_vehicles_;
    ros::Subscriber sub_path_req_;
    ros::Publisher pub_path_, pub_markers_, pub_raw_path_;
};

} // namespace smart_intersection
