#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>
#include <edumip_msgs/EduMipState.h>
#include <string>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
class EdumipRobot 
{
public:
  EdumipRobot();

private:
  void Callback(const edumip_msgs::EduMipState::ConstPtr& state);

 // ros::Timer timer;
  ros::NodeHandle n;

  ros::Subscriber state_sub;
  ros::Publisher joint_pub; 
  ros::Publisher odom_pub;
  tf::TransformBroadcaster broadcaster;
  
  //robot state
  double pos_x, pos_y, rad_l, rad_r, tilt, theta;
  
  // message declarations
 
  geometry_msgs::TransformStamped odom_trans;
  sensor_msgs::JointState joint_state;
  nav_msgs::Odometry odom;
};


EdumipRobot::EdumipRobot():
  pos_x(0),
  pos_y(0),
  rad_l(0),
  rad_r(0),
  tilt(0),
  theta(0)
{
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "edumip_body";


  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states",100);
  state_sub = n.subscribe<edumip_msgs::EduMipState>("/edumip/state",1000,&EdumipRobot::Callback,this);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

}  


void EdumipRobot::Callback(const edumip_msgs::EduMipState::ConstPtr& state)
{
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states",100);
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.name[0] ="jointL";
    joint_state.position[0] = rad_l;
    joint_state.name[1] ="jointR";
    joint_state.position[1] = rad_r;
        
    tilt = state->theta;
    rad_l = state->wheel_angle_L;
    rad_r = state->wheel_angle_R;
    pos_x = state->body_frame_easting;
    pos_y = state->body_frame_northing;
    theta = 1.57075-state->body_frame_heading;

    // set tf    
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = pos_x;
    odom_trans.transform.translation.y = pos_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(tilt,0,theta);
    joint_pub.publish(joint_state);
    broadcaster.sendTransform(odom_trans);
    
    //set odom
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(tilt,0,theta);

    //set the velocity
    odom.child_frame_id = "edumip_body";
    odom.twist.twist.linear.x = state->setpoint_phi_dot*sin(theta);
    odom.twist.twist.linear.y = state->setpoint_phi_dot*cos(theta);
    odom.twist.twist.angular.z = state->setpoint_gamma_dot;

    //publish the message
    odom_pub.publish(odom);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "edumip_my_robot");
  const double degree = M_PI/180;
  EdumipRobot edumiprobot;

  ros::spin();

  return 0;
}


























