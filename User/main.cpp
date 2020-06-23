#include <stdio.h>
#include "hardwareserial.h"
#include "gy85.h"
#include "motor.h"
#include "servo.h"
#include "encoder.h"
#include "battery.h"
#include "led.h"
#include "PID.h"
#include "Kinematics.h"
#include <ros.h>
#include <riki_msgs/Velocities.h>
#include <geometry_msgs/Twist.h>
#include <riki_msgs/PID.h>
#include <riki_msgs/Imu.h>
#include <riki_msgs/Battery.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
#include "algorithm.h"

#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 65.536
 
#define M_PI 3.14159265359	    
 
#define dt 0.02							// 20 ms sample rate! 


int Motor::counts_per_rev_ = COUNTS_PER_REV;

double required_angular_vel = 0;
double required_linear_vel = 0;
uint32_t previous_command_time = 0;

bool is_first = true;
bool accel, gyro, mag;

#if 0
float pid_p1  = 1.025/4;
float pid_i1  = 3.142/4;
float pid_d1  = 2.034/4;


float pid_p2  = 1.025/4;
float pid_i2  = 3.142/4;
float pid_d2  = 2.034/4;
#else
float pid_p1  = 20.0;
float pid_i1  = 0.0;
float pid_d1  = 0.0;

float pid_p2  = 20.0;
float pid_i2  = 0.0;
float pid_d2  = 0.0;
#endif

PID motor1_pid(-3890, 3890, K_P, K_I, K_D);
PID motor2_pid(-3890, 3890, K_P_2, K_I_2, K_D_2);

Motor motor1(MOTOR1, 4095, 0);
Motor motor2(MOTOR2, 4095, 0);

Servo servo1(SERVO1,true);
Servo servo2(SERVO2,false);

Encoder encoder1(ENCODER1, 0xffff, 0);
Encoder encoder2(ENCODER2, 0xffff, 0);
Battery bat(25, 10.6, 12.6);
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, BASE_WIDTH, PWM_BITS);
Gy85  imu;
Led led;


void pid_callback( const riki_msgs::PID& pid);
void command_callback( const geometry_msgs::Twist& cmd_msg);

ros::NodeHandle  nh;

riki_msgs::Imu raw_imu_msg;
riki_msgs::Velocities raw_vel_msg;
riki_msgs::Battery raw_battery_msg;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<riki_msgs::PID> pid_sub("pid", pid_callback);
//ros::Subscriber<riki_msgs::PID> pid_sub("pid", pid_callback);
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
ros::Publisher raw_battery_pub("battery", &raw_battery_msg);


void pid_callback( const riki_msgs::PID& pid)
{
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
}

void command_callback( const geometry_msgs::Twist& cmd_msg)
{
    required_linear_vel = cmd_msg.linear.x;
    required_angular_vel = cmd_msg.angular.z;
    previous_command_time = millis();
}

// Kinematics::output req_rpm ={65,65,0,0};
Kinematics::output req_rpm;

void move_base()
{
	int pwm1,pwm2;
	char str[20];
    req_rpm = kinematics.getRPM(required_linear_vel, 0.0, required_angular_vel);

      if(abs(req_rpm.motor1 - motor1.rpm) < 5){
          pwm1 = motor1_pid.compute(req_rpm.motor1, motor1.rpm);
		  motor1.spin((int)((float)req_rpm.motor1* 27.9 + pwm1)); 
      	}
       else motor1.spin((int)((float)req_rpm.motor1* 27.9));
		  	
	  if(abs(req_rpm.motor2 - motor2.rpm) < 5){
	  	   pwm2 = motor2_pid.compute2(req_rpm.motor2, motor2.rpm);
           motor2.spin((int)((float)req_rpm.motor2* 26.8 + pwm2)); 
	  	}
	  else motor2.spin((int)((float)req_rpm.motor2* 26.8)); 
      
	#if 1
    {
      static char count = 0;
	  if(count ++ == 50){	
	      char buffer[50];
		  count = 0;
	      sprintf (buffer, "req_rpm.motor1: %d", req_rpm.motor1);
	      nh.loginfo(buffer);

	      sprintf(buffer, "motor1.rpm: %d", motor1.rpm);
	      nh.loginfo(buffer); 
	
		  sprintf (buffer, "req_rpm.motor2: %d", req_rpm.motor2);
	      nh.loginfo(buffer);
	      sprintf (buffer, "motor2.rpm: %d", motor2.rpm);
	      nh.loginfo(buffer);
	
	   }
   }
   #endif 
}

void publish_linear_velocity()
{
    motor1.updateSpeed(encoder1.read());
    motor2.updateSpeed(encoder2.read());
    #if 1
    Kinematics::velocities vel;
    vel = kinematics.getVelocities(motor1.rpm, motor2.rpm);
//   vel = kinematics.getVelocities(req_rpm.motor1, req_rpm.motor2);
    raw_vel_msg.linear_x = vel.linear_x;
    raw_vel_msg.linear_y = 0.0;
    raw_vel_msg.angular_z = vel.angular_z;
    raw_vel_pub.publish(&raw_vel_msg);
	#endif
}

void check_imu()
{
    gyro = imu.check_gyroscope();
    accel = imu.check_accelerometer();
    mag = imu.check_magnetometer();
    if (!accel){
        nh.logerror("Accelerometer NOT FOUND!");
    }   

    if (!gyro){
        nh.logerror("Gyroscope NOT FOUND!");
    }   

    if (!mag){
        nh.logerror("Magnetometer NOT FOUND!");
    }
    is_first = false;  
}

void publish_imu()
{
    //geometry_msgs::Vector3 acceler, gyro, mag;
    //this function publishes raw IMU reading
    //measure accelerometer
    if (accel){
        imu.measure_acceleration();
        raw_imu_msg.linear_acceleration = imu.raw_acceleration;
    }

    //measure gyroscope
    if (gyro){
        imu.measure_gyroscope();
        raw_imu_msg.angular_velocity = imu.raw_rotation;
    }

    //measure magnetometer
    if (mag){
        imu.measure_magnetometer();
        raw_imu_msg.magnetic_field = imu.raw_magnetic_field;
    }

    //publish raw_imu_msg object to ROS
    raw_imu_pub.publish(&raw_imu_msg);

}
  
 //»¥²¹ÂË²¨Ëã·¨¼ÆËãpitchºÍroll
void ComplementaryFilter(float accData[3], float gyrData[3], float *pitch, float *roll)
{
    float pitchAcc, rollAcc;               
 
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += ((float)gyrData[0] ) * dt * 180 / M_PI; // Angle around the X-axis
    *roll -= ((float)gyrData[1] ) * dt * 180 / M_PI;    // Angle around the Y-axis
 
    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
   // int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
  //  if (forceMagnitudeApprox > 0.5 && forceMagnitudeApprox < 2)
  //  {
	// Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
        *pitch = *pitch * 0.90 + pitchAcc * 0.10;
 
	// Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
        *roll = *roll * 0.90 + rollAcc * 0.10;
  //  }
} 

void stop_base()
{
    required_linear_vel = 0;
    required_angular_vel = 0;
}

void publishBAT()
{
	raw_battery_msg.battery = bat.get_volt();
	raw_battery_pub.publish(&raw_battery_msg);
}

void print_debug()
{
    char buffer[50]; 
    sprintf (buffer, "Encoder Left: %ld", encoder1.read());
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder Right: %ld", encoder2.read());
    nh.loginfo(buffer);
    //sprintf (buffer, "get line speed : %f, pwm: %d", required_linear_vel, pwm);
    //nh->loginfo(buffer);
}

int main(void) 
{
	bool OnOff = true;
	uint32_t previous_battery_debug_time = 0;
	uint32_t previous_debug_time = 0;
	uint32_t previous_imu_time = 0;
	uint32_t previous_control_time = 0;
	uint32_t publish_vel_time = 0;
	char battery_buffer[]= "The voltage is lower than 11.3V,Please charge! ";
	
	float pitch = 0;
	float roll = 0;
	float accData[3] = {};
	float gyrData[3] = {};
	
	SystemInit();
	initialise();
	motor1.init();
	motor2.init();
	servo1.init();
	servo2.init();
	encoder1.init();
	encoder2.init();
	led.init();
	imu.init();
	bat.init();
    nh.initNode();
	#if 1
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);
    nh.advertise(raw_battery_pub);
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
	while (!nh.connected()){
		    nh.spinOnce();
	} 
	nh.loginfo("Rikibase Connected!");
	#endif
	led.on_off(OnOff);
	while(1){
		if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE)){
			       move_base();
             previous_control_time = millis();
			       #if 0 
              {
               char str[50];
			         sprintf(str,"i love you!!!");
			         Serial.print(str);
               	   
              }
			       #endif	

			
			
    }
    //continue;
    if ((millis() - previous_command_time) >= 400){
       stop_base();
    }
	    
		if ((millis() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE)){
			   publish_linear_velocity();
			   publish_vel_time = millis();
		}

		if ((millis() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE)){
			 
			 char str[20];
			
				if(is_first){
						check_imu();
				} else{
						publish_imu();
						accData[0] = imu.raw_acceleration.x;
						accData[1] = imu.raw_acceleration.y;
						accData[2] = imu.raw_acceleration.z;
						gyrData[0] = imu.raw_rotation.x;
						gyrData[1] = imu.raw_rotation.y;
						gyrData[2] = imu.raw_rotation.z;
						ComplementaryFilter(accData, gyrData, &pitch, &roll);
						servo1.pos(pitch);
						servo2.pos(roll);

				}
				
				previous_imu_time = millis();
		}
	 
		#if 0
		if( (millis() - previous_battery_debug_time) >= (1000 / BAT_PUBLISH_RATE)){
			if(bat.get_volt() < 11.300000){
				OnOff = !OnOff;
				led.on_off(OnOff);
				nh.logwarn(battery_buffer);			
			}
			publishBAT();
			previous_battery_debug_time = millis();		
		}
		#endif
		if(DEBUG){
			if ((millis() - previous_debug_time) >= (1000 / DEBUG_RATE)) {
						print_debug();
						previous_debug_time = millis();
			}
    }

		nh.spinOnce();
  }

}


