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
double Kp = 0.15;
double Ki = 0.0;
double Kd = 0.015;


double heading = 0;
void headingCallback(const std_msgs::Float64ConstPtr& msg){
  heading = msg->data;
}

void pathCallback(const nav_msgs::PathConstPtr& msg){
  
  int n = msg->poses.size();
  if(n < 2) return;

  latest_path = msg;
  timer.start();
}

double integral = 0;
double dr_heading = 90;
double dr_heading_set = 90;
double dr_speed = 15;
double dr_turn = 0;
void drCallback(pose_follower_ackermann::PidConfig &config, uint32_t level) {

  Kp = config.kp;
  Ki = config.ki;
  Kd = config.kd;
  if(dr_heading_set != config.heading){
    dr_heading = config.heading;
    dr_heading_set = config.heading;
  }
  dr_speed = config.speed;
  dr_turn = config.turn;
  integral = 0;
}

void cmdUpdate(const ros::TimerEvent& event){

  int n = latest_path->poses.size();

  //ROS_INFO("position: %f, %f, %f, orientation: %f, %f, %f, %f", msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, msg->poses[0].pose.position.z,
  //msg->poses[0].pose.orientation.w, msg->poses[0].pose.orientation.x, msg->poses[0].pose.orientation.y, msg->poses[0].pose.orientation.z);
  

  geometry_msgs::PoseStamped first_pose = latest_path->poses[0];
  geometry_msgs::PoseStamped last_pose = latest_path->poses[latest_path->poses.size()-1];
  geometry_msgs::PoseStamped target_pose = latest_path->poses[latest_path->poses.size()/4];


  // P Heading controller
  dr_heading = fmod(dr_heading + dr_turn, 360);
  float desired_heading = dr_heading;//atan2(target_pose.pose.position.y, target_pose.pose.position.x) * -180/PI + 90;
  
  float kp = 0.075; //0.0588
  float err = fmod(heading - desired_heading + 180, 360) - 180;
  float p_out = kp * err;

  float heading_adjust = p_out;
  float heading_thresh = 1.5; //1.5 

  float target_speed = dr_speed; //18
  float steep_turn = 0.3; //0.3
  float turn_speed = target_speed;


  static double pre_error = 0;
  double dt = 0.05;

  // Calculate error
  double error = fmod(heading - desired_heading + 180, 360) - 180;

  // Proportional term
  double Pout = Kp * error;

  // Integral term
  integral += error * dt;
  double Iout = Ki * integral;

  // Derivative term
  double derivative = (error - pre_error) / dt;
  double Dout = Kd * derivative;

  // Calculate total output
  heading_adjust = Pout + Iout + Dout;

  // Save error to previous error
  pre_error = error;

  //if(heading_adjust > heading_thresh)         heading_adjust = heading_thresh;
  //else if(heading_adjust < -1*heading_thresh) heading_adjust = -1*heading_thresh;


  // Best fit circle https://goodcalculators.com/best-fit-circle-least-squares-calculator/
  Eigen::MatrixXf l(3,3);
  Eigen::VectorXf r(3), out(3);  
  double sum_xx = 0, sum_yy = 0, sum_xy = 0, sum_x = 0, sum_y = 0, sum_x_xx_yy = 0, sum_y_xx_yy = 0, sum_xx_yy = 0;

  for(int i = 0; i < n; i++){
    double x = latest_path->poses[i].pose.position.x;
    double y = latest_path->poses[i].pose.position.y;
    sum_xx      += x*x;
    sum_yy      += y*y;
    sum_xy      += x*y;
    sum_x       += x;
    sum_y       += y;
    sum_x_xx_yy += x*(x*x + y*y);
    sum_y_xx_yy += y*(x*x + y*y);
    sum_xx_yy   += x*x + y*y;
  }

  l << sum_xx, sum_xy, sum_x,
       sum_xy, sum_yy, sum_y,
       sum_x,  sum_y,  n+1;

  r << sum_x_xx_yy, sum_y_xx_yy, sum_xx_yy;

  out = l.inverse() * r;

  float c_x = out(0)/2;
  float c_y = out(1)/2;
  float c_r = sqrt(4*out(2)+out(0)*out(0)+out(1)*out(1))/2;

  //ROS_INFO("%f, %f, %f", c_x, c_y, c_r);

  float w = -1*target_speed / c_y;

  
  // Pack the cmd_vel message
  //command.angular.z = (w + heading_adjust)/2;
  command.angular.z = heading_adjust;
  command.linear.x = target_speed;

  ROS_INFO("heading: %f, desired_heading: %f, adjust: %f", heading, desired_heading, heading_adjust);

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
  ros::Subscriber heading_sub = nh.subscribe("gps/heading", 1, headingCallback);

  timer = nh.createTimer(ros::Duration(0.05), cmdUpdate, false); //20Hz
  timer.stop();

  ros::spin();
}