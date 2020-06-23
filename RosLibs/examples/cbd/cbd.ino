/* 
 * rosserial ADC Example
 * 
 * This is a poor man's Oscilloscope.  It does not have the sampling 
 * rate or accuracy of a commerical scope, but it is great to get
 * an analog value into ROS in a pinch.
 */
#include <Arduino.h>
#include <ros.h>

#include <riki_msgs/Velocities.h>
#include <geometry_msgs/Twist.h>
#include <riki_msgs/PID.h>
#include <riki_msgs/Imu.h>
#include <riki_msgs/Battery.h>
#include <ros/time.h>
void pid_callback( const riki_msgs::PID& pid);
void command_callback( const geometry_msgs::Twist& cmd_msg);
ros::NodeHandle  nh;
riki_msgs::Imu raw_imu_msg;
riki_msgs::Velocities raw_vel_msg;
riki_msgs::Battery raw_battery_msg;



ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<riki_msgs::PID> pid_sub("pid", pid_callback);
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
ros::Publisher raw_battery_pub("battery", &raw_battery_msg);

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(raw_vel_pub);
  nh.advertise(raw_imu_pub);
  nh.advertise(raw_battery_pub);
  nh.subscribe(pid_sub);
  nh.subscribe(cmd_sub);

  
  #if 1
  while (!nh.connected()){
        nh.spinOnce();
  } 
  nh.loginfo("Rikibase Connected!");
  #endif
}

void loop()
{
  raw_battery_msg.battery = 11.2;
  raw_vel_msg.linear_x = 1.1;
  raw_vel_msg.linear_y = 0.0;
  raw_vel_msg.angular_z = 0.9;
  raw_vel_pub.publish(&raw_vel_msg);
  raw_battery_pub.publish(&raw_battery_msg);
  nh.loginfo("I love you jecica!");
  nh.spinOnce();
  delay(500);
}
void pid_callback( const riki_msgs::PID& pid)
{
  
}
void command_callback( const geometry_msgs::Twist& cmd_msg){}
