#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include "edit_serial/getRange.h"
#include <tf/transform_listener.h>

class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void timepub(const ros::TimerEvent&);
  ros::NodeHandle nh_;
  ros::Timer timer;
  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Publisher scan_pub_;
  ros::Publisher goal_pub_;
  ros::Subscriber joy_sub_;


  ros::ServiceClient client;
  tf::TransformListener listener;

  geometry_msgs::Twist twist;
  sensor_msgs::LaserScan scan;
  geometry_msgs::PoseStamped home_goal;
  tf::StampedTransform transform;

  edit_serial::getRange srv;
  int N;
  int flag;
  int count;
  double last_time;
};


TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  N = 180;
  flag = 0;
  count = 0;
  timer = nh_.createTimer(ros::Duration(0.1), &TeleopTurtle::timepub,this);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/edumip/cmd", 10);
  scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan",10);
  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
  client = nh_.serviceClient<edit_serial::getRange>("getRange");

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(joy->buttons[0]==0 & flag==0) // normal case
    {
       twist.angular.z = a_scale_*joy->axes[angular_];
       twist.linear.x = l_scale_*joy->axes[linear_];
       vel_pub_.publish(twist);
    }
  if(joy->buttons[0]==1) // rotate around itself by 2*pi
    {
       flag = 1;
       scan.header.frame_id = "range_finder";
    

       scan.time_increment = 0.125;
       scan.scan_time = ros::Time::now().toSec();   
       scan.range_min = 0.3;
       scan.range_max = 5.0;
       scan.ranges.clear();
          
       twist.angular.z = 2.0*3.1415/3.0*8.0/N;
       twist.linear.x = 0;

       ros::Rate loop_rate(8);
       loop_rate.reset(); 
       last_time = ros::Time::now().toSec();
       vel_pub_.publish(twist);

       for(int i=0;i<N;i++){
         if (client.call(srv))
         {  
         	twist.angular.z = 2.0*3.1415*8.0/N;
            twist.linear.x = 0;
         	vel_pub_.publish(twist);
            scan.ranges.push_back(srv.response.range/1000.0);
            loop_rate.sleep();
         }
       }
       twist.angular.z = 0;
       twist.linear.x = 0;
       vel_pub_.publish(twist);
       // error time = ros::Time::now().toSec()-last_time;
       {
       	scan.angle_min = 0*3.1415-0.1-(ros::Time::now().toSec()-last_time)*2.0*3.1415*8.0/N;
        scan.angle_max = 2*3.1415-0.1-(ros::Time::now().toSec()-last_time)*2.0*3.1415*8.0/N;
        scan.angle_increment = 2*3.1415/(N-1);
        scan.header.stamp = ros::Time::now();
        scan_pub_.publish(scan);
        count = count+1;
       }
       last_time = ros::Time::now().toSec();
       flag = 0;

       if(count==1)
       {
       	 listener.waitForTransform("/map", "/edumip_body", ros::Time(), ros::Duration(10.0));
       	 try
       	 {   
       	 	listener.lookupTransform("/map", "/edumip_body", ros::Time(0), transform);
       	 }
       	 catch (tf::TransformException ex)
       	 {   
       	 	 home_goal.pose.position.x = transform.getOrigin().getX();
       	 	 home_goal.pose.position.y = transform.getOrigin().getY();
       	 	 home_goal.pose.position.z = transform.getOrigin().getZ();
             home_goal.pose.orientation.x = transform.getRotation().getX();
             home_goal.pose.orientation.y = transform.getRotation().getY();
             home_goal.pose.orientation.z = transform.getRotation().getZ();
             home_goal.pose.orientation.w = transform.getRotation().getW();

       	 }
 
       }
    }

    if(joy->buttons[1]==1)
    {
    	home_goal.header.stamp = ros::Time::now();
    	home_goal.header.frame_id = "map";
    	home_goal.pose.position.x = transform.getOrigin().getX();
       	home_goal.pose.position.y = transform.getOrigin().getY();
       	home_goal.pose.position.z = transform.getOrigin().getZ();

        home_goal.pose.orientation.x = 0;
        home_goal.pose.orientation.y = 0;
        home_goal.pose.orientation.z = transform.getRotation().getZ();

        home_goal.pose.orientation.w = transform.getRotation().getW();

        goal_pub_.publish(home_goal);
    }
}

void TeleopTurtle::timepub(const ros::TimerEvent&)
{
  vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
