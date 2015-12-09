#include <ros.h>

#include <SoftwareSerial.h>
#include <geometry_msgs/Point.h>
#include <SabertoothSimplified.h>

#include <std_msgs/String.h>
#include <string.h>

#include <Herkulex.h>

ros::NodeHandle  nh;

SoftwareSerial SWSerial(NOT_A_PIN, 14); // RX on no pin (unused), TX on pin 11 S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

//Variables
int count = 0;
int temp_count = 0;
int degree = 0;
int temp_degree = 0;
unsigned int pulse_widthL = 0;
unsigned int pulse_widthR = 0;
char temp_message[100];

//Servo Id
int servoID = 253;

std_msgs::String message;
ros::Publisher pub("/arduino/data", &message);

void messageCb(const geometry_msgs::Point& msg)
{
  //Write to motor
  ST.motor(1, msg.x);
  ST.motor(2, msg.y);
}

ros::Subscriber<geometry_msgs::Point> sub_1("/command_converter/commands",messageCb);

void setup()
{
  
  //Ros Publish and Subscribing
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub_1);
  
  //Sabertooth communication
  SWSerial.begin(9600); // This is the baud rate you chose with the DIP switches.

  //Set up servo
  Herkulex.beginSerial2(115200);
  Herkulex.reboot(servoID);
  Herkulex.initialize();
  delay(250);
  Herkulex.torqueON(servoID);
  
  //Initialize servo to 0 degrees
  Herkulex.moveOneAngle(servoID, 2, 1000, LED_GREEN);
  delay(100);
  
  //Setup two Lidar-Lite
  pinMode(2, OUTPUT); // Set pin 2 as trigger pin
  pinMode(3, INPUT); // Set pin 3 as monitor pin
  pinMode(4, OUTPUT); // Set pin 4 to control power enable line
  digitalWrite(4,HIGH); //Turn sensor on5760
  digitalWrite(2, LOW); // Set trigger LOW for continuous read

  pinMode(5, OUTPUT); // Set pin 8 as trigger pin
  pinMode(6, INPUT); // Set pin 9 as monitor pin
  pinMode(7, OUTPUT); // Set pin 7 to control power enable line
  digitalWrite(7,HIGH); //Turn sensor on
  digitalWrite(6, LOW); // Set trigger LOW for continuous read
}

void loop()
{
  //Set servo degree
  Herkulex.moveOneAngle(servoID, temp_degree, 70, LED_RED);
    
  //Get lidar data
  getLidarData();
  
  //Get servo degree
  getServoData();
  
  //Set data
  assignData();
  
  //Publish data
  pub.publish(&message); 
  
  if(temp_degree < 180 && count == temp_count)
    temp_degree++;
  else if(temp_degree > 0)
    temp_degree--;
  
  //Increase count
  if(temp_degree == 0)
  {
    count++;
    temp_count += 2;
  }
  else if(temp_degree == 180)
    count++;
  
  nh.spinOnce();  
}

//Functions
void assignData()
{
  char temp[10];
  
  //Convert to string
  sprintf(temp, "%d", count);
  strcpy(temp_message, temp);
  strcat(temp_message, ",");
  
  sprintf(temp, "%d", degree);
  strcat(temp_message, temp);
  strcat(temp_message, ",");
  
  sprintf(temp, "%d", int(pulse_widthL));
  strcat(temp_message, temp);
  strcat(temp_message, ",");
  
  sprintf(temp, "%d", int(pulse_widthR));
  strcat(temp_message, temp);
  
  message.data = temp_message;
}
void getLidarData()
{
  pulse_widthL = pulseIn(3, HIGH);
  pulse_widthR = pulseIn(6, HIGH);
  
  if(pulse_widthL != 0 && pulse_widthR != 0) // Both pulses are good data
  {
    pulse_widthL = (pulse_widthL/10) - 28; // 10usec = 1 cm of distance for LIDAR-Lite
    pulse_widthR = ((pulse_widthR)/10) -35;
  }
  else if(pulse_widthL == 0 && pulse_widthR != 0)
  {
    pulse_widthR = ((pulse_widthR)/10)-37; // 10usec = 1 cm of distance for LIDAR-Lite
    digitalWrite(4,LOW); // Turn off the sensor
    delay(1);// Wait 1ms
    digitalWrite(4,HIGH); //Turn on the sensor
    delay(1); //Wait 1ms for it to turn on.
  }
  else if(pulse_widthL != 0 && pulse_widthR == 0)
  {
    pulse_widthL = (pulse_widthL/10)-30; // 10usec = 1 cm of distance for LIDAR-Lite
    digitalWrite(7,LOW); // Turn off the sensor
    delay(1);// Wait 1ms 
    digitalWrite(7,HIGH); //Turn on the sensor
    delay(1);//Wait 1ms for it to turn on.
  } 
  else // We read a zero which means we're locking up. 
  { 
    digitalWrite(4,LOW); // Turn off the sensor 1
    digitalWrite(7,LOW); // Turn off the sensor 2
    delay(1); // Wait 1ms
    digitalWrite(4,HIGH); //Turn on the sensor 1
    digitalWrite(7,HIGH); //Turn on the sensor 2
    delay(1); //Wait 1ms for it to turn on.
  }
}
void getServoData()
{
  degree = Herkulex.getAngle(servoID);
  degree = degree + 90;
}
