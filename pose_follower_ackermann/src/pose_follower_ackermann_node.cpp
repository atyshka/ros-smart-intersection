#include <ros/ros.h>
#include <math.h>

#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <pose_follower_ackermann/PidConfig.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#define PI 3.1415f

ros::Timer timer;
ros::Publisher twist_pub;
geometry_msgs::Twist command;
nav_msgs::PathConstPtr latest_path;

// PID
double Kp = 3.5;
double Ki = 0.1;
double Kd = 0.5;


void pathCallback(const nav_msgs::PathConstPtr& msg){
  
  int n = msg->poses.size();
  if(n < 2) return;

  latest_path = msg;
  timer.start();
}

double integral = 0;
double dr_speed = 15;
void drCallback(pose_follower_ackermann::PidConfig &config, uint32_t level) {

  Kp = config.kp;
  Ki = config.ki;
  Kd = config.kd;
  integral = 0;
}

void cmdUpdate(const ros::TimerEvent& event){

  int n = latest_path->poses.size();

  geometry_msgs::PoseStamped first_pose = latest_path->poses[0];
  geometry_msgs::PoseStamped last_pose = latest_path->poses[latest_path->poses.size()-1];
  geometry_msgs::PoseStamped target_pose = latest_path->poses[latest_path->poses.size()/4];


  // PID Heading controller
  float target_speed = dr_speed; //18
  float steep_turn = 0.3; //0.3
  float turn_speed = target_speed;
  float heading_thresh = 1.5; //1.5 

  static double d_err = 0;
  double dt = 0.05;

  double err = atan2(target_pose.pose.position.y, target_pose.pose.position.x);

  double Pout = Kp * err;

  integral += err * dt;
  double Iout = Ki * integral;

  double d = (err - d_err) / dt;
  double Dout = Kd * d;

  float heading_adjust = Pout + Iout + Dout;

  d_err = err;

  if(heading_adjust > heading_thresh)         heading_adjust = heading_thresh;
  else if(heading_adjust < -1*heading_thresh) heading_adjust = -1*heading_thresh;

  
  // Pack the cmd_vel message
  command.angular.z = heading_adjust;
  command.linear.x = target_speed;

  twist_pub.publish(command);
}



int main(int argc, char** argv){

  ros::init(argc, argv, "pose_follower_ackermann");
  ros::NodeHandle nh;

  // Creates the dynamic reconfigure server and callback function
  dynamic_reconfigure::Server<pose_follower_ackermann::PidConfig> server;
  dynamic_reconfigure::Server<pose_follower_ackermann::PidConfig>::CallbackType f;
  f = boost::bind(&drCallback, _1, _2);
  server.setCallback(f);


  // Publishers
  twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // Subscribers
  ros::Subscriber path_sub = nh.subscribe("target_path", 1, pathCallback); 

  timer = nh.createTimer(ros::Duration(0.05), cmdUpdate, false); //20Hz
  timer.stop();

  ros::spin();
}