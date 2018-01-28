#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>
#include <edumip_msgs/EduMipState.h>
#include <string>
#include <sensor_msgs/JointState.h>

void Callback(const edumip_msgs::EduMipState::ConstPtr& state)
{
  ROS_INFO("I heard: [%d]", state->phi);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Subscriber state_sub = n.subscribe<edumip_msgs::EduMipState>("/edumip/state",10,Callback);
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states",1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;
    double pos_x =0, pos_y=0, rad_l=0, rad_r=0;
    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "edumip_body";

    while (ros::ok()) {
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
        odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(sin(angle),0,0);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // Create new robot state
        tilt += tinc;
        if (tilt<-.5 || tilt>0) tinc *= -1;
        height += hinc;
        if (height>.2 || height<0) hinc *= -1;
        rad_l += degree;
        rad_r += degree;
        angle += degree/4;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
