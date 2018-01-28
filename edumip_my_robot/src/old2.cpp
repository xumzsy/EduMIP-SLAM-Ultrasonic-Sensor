#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>
#include <edumip_msgs/EduMipState.h>
#include <string>
#include <sensor_msgs/JointState.h>

class EdumipRobot 
{
public:
  EdumipRobot();
  void timepub();
private:
  void Callback(const edumip_msgs::EduMipState::ConstPtr& state);

  
  ros::NodeHandle n;

  ros::Subscriber state_sub;
  ros::Publisher joint_pub; 
  tf::TransformBroadcaster broadcaster;

  //robot state
  double pos_x, pos_y, rad_l, rad_r, tilt;
  
  // message declarations
 

  geometry_msgs::TransformStamped odom_trans;
  sensor_msgs::JointState joint_state;

};


EdumipRobot::EdumipRobot():
  pos_x(0),
  pos_y(0),
  rad_l(0),
  rad_r(0),
  tilt(0)
{
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "edumip_body";

  ros::Subscriber state_sub = n.subscribe<edumip_msgs::EduMipState>("/edumip/state",10,&EdumipRobot::Callback,this);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states",1);
     
  const double degree = M_PI/180;
  ros::Rate loop_rate(30);

    while (ros::ok()) {
    ros::Subscriber state_sub = n.subscribe<edumip_msgs::EduMipState>("/edumip/state",10,&EdumipRobot::Callback,this);
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(2);
        joint_state.position.resize(2);
        joint_state.name[0] ="jointL";
        joint_state.position[0] = rad_l;
        joint_state.name[1] ="jointR";
        joint_state.position[1] = rad_r;
        

        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = pos_x;
        odom_trans.transform.translation.y = pos_y;
        odom_trans.transform.translation.z = .7;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(tilt,0,0);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

}  


void EdumipRobot::Callback(const edumip_msgs::EduMipState::ConstPtr& state)
{
    ROS_INFO("R");
    rad_l = state->wheel_angle_L;
    rad_r = state->wheel_angle_R;
    pos_x += state->body_frame_northing;
    pos_y += state->body_frame_easting;
    tilt += state->body_frame_heading;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "edumip_my_robot");
  ROS_INFO("S");
  EdumipRobot edumiprobot;

 

  return 0;
}


























