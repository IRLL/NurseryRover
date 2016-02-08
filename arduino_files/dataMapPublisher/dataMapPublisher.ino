#include <ros.h>

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <Herkulex.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

ros::NodeHandle  nh;

SoftwareSerial SWSerial(NOT_A_PIN, 14); // RX on no pin (unused), TX on pin 11 S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//Variables
int count = 0;
int temp_count = 0;
int degree = 0;
int temp_degree = 0;
unsigned int pulse_widthL = 0;
unsigned int pulse_widthR = 0;

//Servo Id
int servoID = 253;

std_msgs::Float32MultiArray lidarData;
ros::Publisher pubData("/arduino/data", &lidarData);
std_msgs::Float32 compassData;
ros::Publisher pubCompass("/arduino/compass", &compassData);
std_msgs::String errorMessage;
ros::Publisher pubError("/arduino/error", &errorMessage);

void messageCb(const std_msgs::Int32MultiArray& msg)
{
  //Write to motor
  ST.motor(1, msg.data[0]);
  ST.motor(2, msg.data[1]);
}

ros::Subscriber<std_msgs::Int32MultiArray> subMotorCommands("/command_converter/commands",messageCb);

void setup()
{
  //Ros Publish and Subscribing
  nh.initNode();
  lidarData.data_length = 4;
  nh.advertise(pubData);
  nh.advertise(pubCompass);
  nh.advertise(pubError);
  nh.subscribe(subMotorCommands);
  
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
  
  /* Initialise the sensor */
  if(!mag.begin())
  {
    //Publish error maybe?
  }
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
  pubData.publish(&lidarData); 
  
  if(temp_degree < 180 && count == temp_count)
    temp_degree++;
  else if(temp_degree > 0)
    temp_degree--;
  
  //Increase count
  if(temp_degree == 0)
  {
    count++;
    temp_count += 2;
    getCompassData();
  }
  else if(temp_degree == 180)
  {
    count++;
    getCompassData();
  }
  
  nh.spinOnce();  
}

//Functions
void assignData()
{
  lidarData.data[0] = degree;
  lidarData.data[1] = int(pulse_widthL);
  lidarData.data[2] = int(pulse_widthR);
  lidarData.data[3] = count;
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
void getCompassData()
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
  
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  compassData.data = heading * 180/M_PI; 
  
  pubCompass.publish(&compassData);
}
