#include <ros/ros.h>
#include <math.h>

#include <std_msgs/Float64.h>
#include <smart_intersection/GuidedPath.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <dynamic_reconfigure/server.h>
#include <pose_follower_ackermann/PidConfig.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PI 3.1415f

ros::Timer timer;
ros::Publisher twist_pub;
geometry_msgs::Twist command, actual;
smart_intersection::GuidedPathConstPtr latest_path;
tf2_ros::Buffer tf_buffer;
std::vector<geometry_msgs::PoseStamped>::const_iterator current_node;
int vehicle_id;

// PID
double Kp = 3.5;
double Ki = 0.1;
double Kd = 0.5;

void twistCallback(const geometry_msgs::TwistStamped& msg)
{
  actual = msg.twist;
}

void pathCallback(const smart_intersection::GuidedPathConstPtr &msg)
{
  if (msg->vehicle_id == vehicle_id)
  {
    ROS_INFO("Received path");
    latest_path = msg;
    current_node = latest_path->path.poses.begin();
    timer.start();
  }
}

double integral = 0;
double integral_x = 0;
// double dr_speed = 15;
void drCallback(pose_follower_ackermann::PidConfig &config, uint32_t level)
{

  Kp = config.kp;
  Ki = config.ki;
  Kd = config.kd;
  integral = 0;
}

void cmdUpdate(const ros::TimerEvent &event)
{

  int n = latest_path->path.poses.size();
  ROS_INFO("Looking up transform");
  auto pos = tf_buffer.lookupTransform("intersection_1", "audibot_0/base_link", ros::Time(0));
  tf2::Transform pos_tf, inv_pos_tf;
  tf2::fromMsg(pos.transform, pos_tf);
  inv_pos_tf = pos_tf.inverse();
  while (current_node->header.stamp < ros::Time::now())
  {
    if (std::next(current_node) == latest_path->path.poses.end())
    {
      command.angular.z = 0;
      command.linear.x = 0;
      twist_pub.publish(command);
      return;
    }
    ROS_INFO("Advancing to next node");
    current_node++;
  }
  ROS_INFO("Node time: %f", current_node->header.stamp.toSec());
  ROS_INFO("Current time: %f", ros::Time::now().toSec());

  // PID Heading controller
  static double d_err_x = 0;
  double dt = 0.01;

  // Get pose in vehicle frame
  tf2::Vector3 target_pos;
  tf2::fromMsg(current_node->pose.position, target_pos);
  tf2::Vector3 target_in_vehicle_frame = inv_pos_tf * target_pos;
  ROS_INFO("Target x %f, actual x %f", current_node->pose.position.x, pos.transform.translation.x);
  double err_x = target_in_vehicle_frame.x();
  ROS_INFO("Error x: %f", err_x);

  double Poutx = Kp * err_x;

  integral_x += err_x * dt;
  double Ioutx = Ki * integral;

  double dx = (err_x - d_err_x) / dt;
  double Doutx = Kd * dx;

  float target_speed_diff = Poutx + Ioutx + Doutx;

  d_err_x = err_x;


  float target_speed = actual.linear.x + target_speed_diff; // 18
  float steep_turn = 0.3;        // 0.3
  float turn_speed = target_speed;
  float heading_thresh = 1.5; // 1.5

  static double d_err = 0;

  double err = target_in_vehicle_frame.y();
  ROS_INFO("Error: %f", err);

  double Pout = Kp * err;

  integral += err * dt;
  double Iout = Ki * integral;

  double d = (err - d_err) / dt;
  double Dout = Kd * d;

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
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pose_follower_ackermann");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  nh_private.getParam("vehicle_id", vehicle_id);
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Creates the dynamic reconfigure server and callback function
  dynamic_reconfigure::Server<pose_follower_ackermann::PidConfig> server;
  dynamic_reconfigure::Server<pose_follower_ackermann::PidConfig>::CallbackType f;
  f = boost::bind(&drCallback, _1, _2);
  server.setCallback(f);

  // Publishers
  twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // Subscribers
  ros::Subscriber path_sub = nh.subscribe("/requested_path", 1, pathCallback);
  ros::Subscriber vel_sub = nh.subscribe("twist", 1, twistCallback);
  timer = nh.createTimer(ros::Duration(0.01), cmdUpdate, false); // 20Hz
  timer.stop();

  ros::spin();
}