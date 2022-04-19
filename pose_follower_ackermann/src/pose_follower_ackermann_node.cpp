#include <ros/ros.h>
#include <math.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>

#include <smart_intersection/GuidedPath.h>
#include <smart_intersection/PathRequest.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <dynamic_reconfigure/server.h>
#include <pose_follower_ackermann/PidConfig.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PI 3.1415f

ros::Timer timer;
ros::Publisher twist_pub, bool_pub, path_req_pub, path_vis_pub;

geometry_msgs::Twist command, actual;
smart_intersection::GuidedPathConstPtr latest_path;
tf2_ros::Buffer tf_buffer;
std::vector<geometry_msgs::PoseStamped>::const_iterator current_node;
int vehicle_id;
bool exiting_intersection = false;
bool in_intersection = false;
double approach_intersection_distance = 80.0f;
char vehicle_frame_id[64]; //no idea how big this should be, 64 is probably enough

// PID
double Kp = 3.5;
double Ki = 0.1;
double Kd = 0.5;
double integral = 0;
double integral_x = 0;

void twistCallback(const geometry_msgs::TwistStamped& msg)
{
  actual = msg.twist;
}

void pathCallback(const smart_intersection::GuidedPathConstPtr &msg)
{
  if (msg->vehicle_id == vehicle_id)
  {
    ROS_INFO("Vehicle %d received path...starting intersection control", vehicle_id);
    integral = 0;
    integral_x = 0;
    latest_path = msg;
    current_node = latest_path->path.poses.begin();
    timer.start();
    std_msgs::Bool bmsg;
    bmsg.data = true;
    bool_pub.publish(bmsg);

    //latest_path->path.header.frame_id = "intersection_1";
    path_vis_pub.publish(latest_path->path);
  }
}


// double dr_speed = 15;
void drCallback(pose_follower_ackermann::PidConfig &config, uint32_t level)
{

  Kp = config.kp;
  Ki = config.ki;
  Kd = config.kd;
  integral = 0;
}

void locationUpdate(const ros::TimerEvent &event){

  if(tf_buffer.canTransform("intersection_1", vehicle_frame_id, ros::Time(0))){
    geometry_msgs::TransformStamped iframe_base = tf_buffer.lookupTransform("intersection_1", vehicle_frame_id, ros::Time(0));
    geometry_msgs::PoseStamped in, out;
    in.header.frame_id = vehicle_frame_id;
    in.pose.position.x = 0;
    in.pose.position.y = 0;
    in.pose.position.z = 0;
    in.pose.orientation.w = 1;
    in.pose.orientation.x = 0;
    in.pose.orientation.y = 0;
    in.pose.orientation.z = 0;
    //tf_buffer.transform(in, out, "intersection_1");
    tf2::doTransform(in, out, iframe_base);

    double intersection_distance = sqrt(out.pose.position.x * out.pose.position.x + out.pose.position.y * out.pose.position.y);

    //ROS_INFO_THROTTLE(2,"intersection distance: %f, in intersection: %d, exiting intersection: %d", intersection_distance, in_intersection, exiting_intersection);
    //ROS_INFO_THROTTLE(2,"pos: %f %f %f orientation: %f %f %f %f", out.pose.position.x, out.pose.position.y, out.pose.position.z, out.pose.orientation.w, out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z);

    if(exiting_intersection){
      if(intersection_distance > approach_intersection_distance){
        exiting_intersection = false;
      }     
    } else {
      if(intersection_distance < approach_intersection_distance && !in_intersection){
        out.header.frame_id = "intersection_1";
        smart_intersection::PathRequest req;
        req.header.frame_id = "intersection_1";
        req.pose = out;

        //in the path request we need to know the direction we're approaching intersection from
        //we can calculate this out of the intersection position
        double approach_angle = atan2(out.pose.position.y, out.pose.position.x);

        //map to range 0*PI to 2*PI 
        approach_angle = (approach_angle > 0) ? approach_angle : 2*PI + approach_angle;

        if (approach_angle < (3.0 * M_PI / 4.0) && approach_angle > (M_PI / 4.0) ) 
          req.direction = 0; //up  
        else if(approach_angle < (7.0 * M_PI / 4.0)  && approach_angle > (5.0 * M_PI / 4.0))
          req.direction = 1; //down
        else if (approach_angle < (5.0 * M_PI / 4.0) && approach_angle > (3.0 * M_PI / 4.0) )
          req.direction = 2; // left 
        else 
          req.direction = 3; //right 
        
        ROS_INFO("Vehicle %d is approaching from angle %f, selecting direction %d", vehicle_id, approach_angle, req.direction);
        req.velocity = actual.linear.x;
        req.vehicle_id = vehicle_id;
        path_req_pub.publish(req);
        in_intersection = true;
      }
    }
  } 
  else ROS_WARN("Vehicle %d: Could not get transform from base_link to intersection", vehicle_id);
}


void cmdUpdate(const ros::TimerEvent &event)
{

  int n = latest_path->path.poses.size();
  ROS_DEBUG("Vehicle %d: Looking up transform", vehicle_id);
  auto pos = tf_buffer.lookupTransform("intersection_1", vehicle_frame_id, ros::Time(0));
  tf2::Transform pos_tf, target_tf;
  tf2::fromMsg(pos.transform, pos_tf);
  tf2::Vector3 target, prev_pos, current_pos;

  double target_speed;
  geometry_msgs::PoseStamped target_pose;
  geometry_msgs::PoseStamped current_pose_stamped;

  while (current_node->header.stamp < ros::Time::now())
  {
    if (std::next(current_node) == latest_path->path.poses.end())
    {
      ROS_INFO("Vehicle %d: Intersection path following complete", vehicle_id);
      timer.stop();
      std_msgs::Bool bmsg;
      bmsg.data = false;
      in_intersection = false;
      exiting_intersection = true;
      bool_pub.publish(bmsg);

      command.angular.z = 0;
      command.linear.x = 0;
      twist_pub.publish(command);

      nav_msgs::Path empty_path;
      empty_path.header.frame_id = "intersection_1";
      path_vis_pub.publish(empty_path);
      return;
    }
    tf2::fromMsg(current_node->pose.position, prev_pos);
    geometry_msgs::PoseStamped prev_pose_stamped = *current_node;
    current_node++;
    tf2::fromMsg(current_node->pose.position, current_pos);
    current_pose_stamped = *current_node;

    /*if(current_pos.getX() == 0 && current_pos.getY() == 0){
      ROS_WARN("Vehicle %d: Error loading next path pose", vehicle_id);
      return; //Check for errors
    }*/
    // Make rotation matrix orienting path
    tf2::Matrix3x3 rotation;
    rotation[0] = (current_pos - prev_pos).normalized();
    rotation[2] = tf2::Vector3(0, 0, 1);
    rotation[1] = rotation[2].cross(rotation[0]);
    target_tf.setBasis(rotation.transpose());
    target_tf.setOrigin(current_pos);

    double pose_dist = hypot(current_pose_stamped.pose.position.x - prev_pose_stamped.pose.position.x, current_pose_stamped.pose.position.y - prev_pose_stamped.pose.position.y);
    double pose_time = current_pose_stamped.header.stamp.toSec() - prev_pose_stamped.header.stamp.toSec();
    target_speed = pose_dist / pose_time;

    geometry_msgs::TransformStamped iframe_base = tf_buffer.lookupTransform("intersection_1", vehicle_frame_id, ros::Time(0));
    tf2::doTransform(current_pose_stamped, target_pose, iframe_base);
  }
  auto vehicle_in_target_frame = (target_tf.inverse() * pos_tf).getOrigin();
  //ROS_INFO("Vehicle %d: vehicle in target frame: x: %f, y: %f, z: %f", vehicle_id, vehicle_in_target_frame.getX(), vehicle_in_target_frame.getY(), vehicle_in_target_frame.getZ());
  //ROS_INFO("Vehicle %d: target_pose: x: %f, y: %f, z: %f", vehicle_id, target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
  //ROS_INFO("Vehicle %d: current_pose: x: %f, y: %f, z: %f", vehicle_id, current_pose_stamped.pose.position.x, current_pose_stamped.pose.position.y, current_pose_stamped.pose.position.z);
  //ROS_DEBUG("Node time: %f", current_node->header.stamp.toSec());
  //ROS_DEBUG("Current time: %f", ros::Time::now().toSec());




  // PID Heading controller
  static double d_err_x = 0;
  double dt = 0.01;

  // Get pose in vehicle frame
  //ROS_DEBUG("Target x %f, actual x %f", current_node->pose.position.x, pos.transform.translation.x);
  /*double err_x = -vehicle_in_target_frame.x();
  ROS_DEBUG("Error x: %f", err_x);

  double Poutx = Kp * err_x;

  integral_x += err_x * dt;
  double Ioutx = Ki * integral;

  double dx = (err_x - d_err_x) / dt;
  double Doutx = Kd * dx;
  float target_speed_diff = Poutx + Ioutx + Doutx;

  d_err_x = err_x;*/


  //float target_speed = actual.linear.x + target_speed_diff; // 18
  float steep_turn = 0.3;        // 0.3
  float turn_speed = target_speed;
  float heading_thresh = 1.5; // 1.5

  static double d_err = 0;

  double err = -vehicle_in_target_frame.y();
  if(!std::isnan(err)){
    //double err = atan2(target_pose.pose.position.y, target_pose.pose.position.x);

    double Pout = Kp * err;

    integral += err * dt;
    double Iout = 0;//Ki * integral;

    double d = (err - d_err) / dt;
    double Dout = Kd * d;

    ROS_INFO_THROTTLE(0.5, "Vehicle: %d: Error: %f, Pout: %f, Iout: %f, Dout: %f", vehicle_id, err, Pout, Iout, Dout);

    float heading_adjust = Pout + Iout + Dout;

    d_err = err;

    if (heading_adjust > heading_thresh)
      heading_adjust = heading_thresh;
    else if (heading_adjust < -1 * heading_thresh)
      heading_adjust = -1 * heading_thresh;

    // Pack the cmd_vel message
    command.angular.z = heading_adjust;
    command.linear.x = target_speed;

    twist_pub.publish(command);
  } else ROS_WARN("Vehicle %d: Calculated error is NaN", vehicle_id);
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "pose_follower_ackermann");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  nh_private.getParam("vehicle_id", vehicle_id);
  tf2_ros::TransformListener tf_listener(tf_buffer);

  //create the string representing the frame ID for the vehicle based on the vehicle_id parameter
  sprintf(vehicle_frame_id, "audibot_%d/base_link", vehicle_id);

  // Creates the dynamic reconfigure server and callback function
  dynamic_reconfigure::Server<pose_follower_ackermann::PidConfig> server;
  dynamic_reconfigure::Server<pose_follower_ackermann::PidConfig>::CallbackType f;
  f = boost::bind(&drCallback, _1, _2);
  server.setCallback(f);

  // Publishers
  twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  bool_pub = nh.advertise<std_msgs::Bool>("intersection_control", 1);
  path_vis_pub = nh.advertise<nav_msgs::Path>("intersection_path", 1);
  path_req_pub = nh.advertise<smart_intersection::PathRequest>("/path_req", 1);

  // Subscribers
  ros::Subscriber path_sub = nh.subscribe("/requested_path", 1, pathCallback);
  ros::Subscriber vel_sub = nh.subscribe("twist", 1, twistCallback);

  timer = nh.createTimer(ros::Duration(0.01), cmdUpdate, false); // 100Hz
  timer.stop();

  ros::Timer location_timer = nh.createTimer(ros::Duration(0.1), locationUpdate, false); // 10Hz

  ros::spin();
}