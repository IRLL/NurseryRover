#include <ros.h>

#include <std_msgs/String.h>

#include <SabertoothSimplified.h>

#include <Herkulex.h>

#include <SoftwareSerial.h>

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

SabertoothSimplified ST;

//Variables
int count = 0;
int degree = 0;
unsigned int pulse_widthL = 0;
unsigned int pulse_widthR = 0;
char temp_message[100];

//Motor speeds
int m1Speed = 0;
int m2Speed = 0;

//Servo Id
int servoID = 253;

/*
void roverCommandsCb( const std_msgs::String msg)
{
  //See this link to split string
  http://answers.ros.org/question/81386/rosserial-arduino-unpack-requires-a-string-arguement-of-length-4/
  
  //This is where you parse msg
  
  //Write motor commands
  writeMotorCommands();
}*/

std_msgs::String message;
ros::Publisher pub("/arduino/data", &message);
//ros::Subscriber<std_msgs::String> sub("/rover_commands", roverCommandsCb);

void setup()
{
  //Sabertooth communication
  //SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
  
  //Ros Publish and Subscribing
  nh.initNode();
  nh.advertise(pub);
  
  Herkulex.beginSerial2(115200);
  Herkulex.reboot(servoID);
  
  //Set up servo
  Herkulex.initialize();
  delay(250);
  Herkulex.torqueON(servoID);
  
  //Initialize servo to 0 degrees
  Herkulex.moveOneAngle(servoID, degree, 1000, LED_GREEN);
  
  //Setup Lidar
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
  int deg = 0;
  
  for(deg = -90; deg < 90; deg++)
  {
    //Set servo degree
    Herkulex.moveOneAngle(servoID, deg, 1000, LED_RED);
    
    //Get lidar data
    getLidarData();
  
    //Get servo degree
    getServoData();
  
    //Set data
    assignData();
  
    //Publish data
    pub.publish(&message); 
    
    delay(50);
    
    nh.spinOnce();
  }

  for(int deg = 90; deg > -90; deg--)
  {
    //Set servo degree
    Herkulex.moveOneAngle(servoID, deg, 1000, LED_RED);
    
    //Get lidar data
    getLidarData();
  
    //Get servo degree
    getServoData();
  
    //Set data
    assignData();
  
    //Publish data
    pub.publish(&message); 
    
    delay(50);
    
    nh.spinOnce();
  }

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
  
  if(pulse_widthL != 0 && pulse_widthR != 0){ // If we get a reading that isn't zero, let's print it
    pulse_widthL = (pulse_widthL/10) - 28; // 10usec = 1 cm of distance for LIDAR-Lite
    pulse_widthR = ((pulse_widthR)/10) -35;
  }
  else if(pulse_widthL == 0 && pulse_widthR != 0){ // If we get a reading that isn't zero, let's print it
    pulse_widthR = ((pulse_widthR)/10)-37; // 10usec = 1 cm of distance for LIDAR-Lite
    digitalWrite(4,LOW); // Turn off the sensor
    delay(1);// Wait 1ms
    digitalWrite(4,HIGH); //Turn on te sensor
    delay(1); //Wait 1ms for it to turn on.
  }
  else if(pulse_widthL != 0 && pulse_widthR == 0){ // If we get a reading that isn't zero, let's print it
    pulse_widthL = (pulse_widthL/10)-30; // 10usec = 1 cm of distance for LIDAR-Lite
    digitalWrite(7,LOW); // Turn off the sensor
    delay(1);// Wait 1ms 
    digitalWrite(7,HIGH); //Turn on te sensor
    delay(1);//Wait 1ms for it to turn on.
  } 
  else{ // We read a zero which means we're locking up. 
   digitalWrite(4,LOW); // Turn off the sensor 1
    digitalWrite(7,LOW); // Turn off the sensor 2
    delay(1); // Wait 1ms
    digitalWrite(4,HIGH); //Turn on te sensor 1
    digitalWrite(7,HIGH); //Turn on te sensor 2
    delay(1); //Wait 1ms for it to turn on.
  }
}
void getServoData()
{
  degree = Herkulex.getAngle(servoID);
  degree = degree + 90;
}
void writeMotorCommands()
{
 ST.motor(1, m1Speed);
 ST.motor(2, m2Speed); 
}


