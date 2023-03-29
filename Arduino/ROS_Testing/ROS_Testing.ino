#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

ros::NodeHandle  nh;

std_msgs::Int16 int_msg_right;
ros::Publisher motor_right_pub("/lidarbot/motor_right", &int_msg_right);

std_msgs::Int16 int_msg_left;
// ros::Publisher motor_left_pub("/lidarbot/motor_left", &int_msg_left);


void cmd_vel_cb(const geometry_msgs::Twist & msg) {
  // Read the message. Act accordingly.
  // We only care about the linear x, and the rotational z.
  int vr = 0, vl=0;
  const float v = msg.linear.x;
  const float omega = msg.angular.z;

  if(== 0){
    vr = v;
    vl = v;
  }else{
    vr = v * 1 + 0.3255 * omega;//equation from the inverted matrix math solving simultaneous equations
    vl = v * 1 - 0.3255 * omega;
  }
  // Flipped r and l. Added steering scaler.
  float right_cmd = (-z_rotation*1.8)/2.0 + x;
  float left_cmd = 2.0*x - right_cmd;
  bool right_dir = (right_cmd>0)? right_fwd : !right_fwd;
  bool left_dir = (left_cmd>0)? left_fwd : !left_fwd;

  digitalWrite(right_dir_pin, right_dir);
  digitalWrite(left_dir_pin, left_dir);
  
  int right_write = int( default_vel * right_cmd);
  int left_write = int( default_vel * left_cmd );
 
  if (x == 0 && z_rotation == 0){
      MoveStop();
  }
  
  // Advertise the arduino command.
  int abs_left_write =  abs(left_write);
  int abs_right_write = abs(right_write);

  int_msg_right.data = right_write;
  int_msg_left.data = left_write;
  // motor_right_pub.publish(&int_msg_right);
  // motor_left_pub.publish(&int_msg_left);

  analogWrite(right_pwm_pin, abs_right_write);
  analogWrite(left_pwm_pin,  abs_left_write);
}


void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(sub);
  //nh.advertise(motor_right_pub);
  //nh.advertise(motor_left_pub );
}

void loop() {
  nh.spinOnce();
  delay(1);
}