#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

bool intersection_control = false;

geometry_msgs::Twist intersection_cmd;
geometry_msgs::Twist lane_cmd;

ros::Publisher cmd_pub;


void cmdOutput(const ros::TimerEvent& event){
  if(intersection_control)
    cmd_pub.publish(intersection_cmd);
  else
    cmd_pub.publish(lane_cmd);
}  

void recvControl(const std_msgs::BoolConstPtr& msg){
  intersection_control = msg->data;
}

void recvTwistIntersection(const geometry_msgs::TwistConstPtr& msg){
  intersection_cmd = *msg;
}

void recvTwistLane(const geometry_msgs::TwistConstPtr& msg){
  lane_cmd = *msg;
}

int main(int argc, char** argv){
  ros::init(argc,argv,"control_multiplexer");
  ros::NodeHandle nh;

  ros::Subscriber sub_control_bool = nh.subscribe("intersection_control", 1, recvControl);
  ros::Subscriber sub_twist_intersection = nh.subscribe("cmd_vel_intersection", 1, recvTwistIntersection);
  ros::Subscriber sub_twist_lane = nh.subscribe("cmd_vel_lane", 1, recvTwistLane);

  cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Timer timer = nh.createTimer(ros::Duration(0.05), cmdOutput, false);

  ros::spin();
}