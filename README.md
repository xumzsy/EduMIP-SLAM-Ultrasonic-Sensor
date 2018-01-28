# EduMIP-SLAM-Ultrasonic-Sensor
Toy example of doing SLAM using EduMIP (two wheel self-balance robot) and single ultrasonic sensor. 

This project is quite simple and mainly to get experience with ROS. Single ultrasonic sensor is really cheap compared to laser 
scanner but is really terrible for SLAM. A not bad map is achieved in toy environment but no quantive result is obtained. See the link 
below for demo.

https://www.youtube.com/watch?v=zd1okAbZ_yY&t=1s

Package slam_gmapping, navigation, joy_twist, serial comes from ROS packages. Package to control EduMIP is provided by 
Dr. Whitcomb, see https://git.lcsr.jhu.edu/lwhitco1/edumip_balance_ros

Package edit_serial and edumip_my_robot is implemented and joy_twist is highly modified. The data from different topics are 
synced directly in callback function, but it should be better to sync data in seperate function through message stamp.


