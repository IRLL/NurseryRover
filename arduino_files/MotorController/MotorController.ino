/* 
 * rosserial::geometry_msgs::PoseArray Test
 * Sums an array, publishes sum 
 */

#include <ros.h>
#include <SoftwareSerial.h>
#include <geometry_msgs/Point.h>
#include <SabertoothSimplified.h>

ros::NodeHandle nh;

SoftwareSerial SWSerial(NOT_A_PIN, 14); // RX on no pin (unused), TX on pin 11 S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

//Motor speeds
int m1Speed = 0;
int m2Speed = 0;

void messageCb(const geometry_msgs::Point& msg){
  digitalWrite(13, HIGH);
  m1Speed = msg.x;
  m2Speed = msg.y;
  
  writeMotorCommands();
}

ros::Subscriber<geometry_msgs::Point> s("/command_converter/commands",messageCb);

void setup()
{ 
  pinMode(13, OUTPUT);
  //Sabertooth communication
  SWSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
  
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(s);
}

void loop()
{  
  nh.spinOnce();
}

void writeMotorCommands()
{
 ST.motor(1, m1Speed);
 ST.motor(2, m2Speed); 
}

