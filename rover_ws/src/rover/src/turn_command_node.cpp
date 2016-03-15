/*
 * References:
 *	
 *
*/

/*
 * Notes:
 * 	1. Message Notes:
 *		A. 1 = left distance
 *		B. 2 = right distance
 * 	2. Motor Speed:
 * 		A. Values range from 127 to -127
 * 
*/

#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

#include <ros/console.h>

class CommandConverter
{
private:
  ros::NodeHandle _nh;
  
  std_msgs::Int32MultiArray command;
  
  //Declare Publisher and Subscriber
  ros::Publisher pub;
  ros::Subscriber sub;
  
  int _left_distance;
  int _right_distance;
  
  int _left_motor_speed;
  int _right_motor_speed;
  int turn_significance;

public:
  CommandConverter()
  {
    turn_significance = 0;

    pub = _nh.advertise<std_msgs::Int32MultiArray>("/command_converter/commands", 2);
    sub = _nh.subscribe("/lidar_data/cluster", 4, &CommandConverter::commandConverterCallback, this);
  }
	
private:
  void commandConverterCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
  {
    int temp_array[4];
    int i = 0;
    
    //Set values
    for(std::vector<int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
    {
      temp_array[i] = *it;
      i++;
      
    }
    
    //Set left and right distances
    if(temp_array[0] < temp_array[2])
    {
      _left_distance = abs(temp_array[0] - 400);
      _right_distance = abs(temp_array[2] - 400); 
    }
    else
    {
      _left_distance = abs(temp_array[2] - 400);
      _right_distance = abs(temp_array[0] - 400);
    }
    
    setMotorSpeed(_left_distance, _right_distance);
    
    assignMotorSpeedData();
    
    pub.publish(command);
    
    // TESTING
    //ROS_INFO("Left: %d	Right: %d", _left_distance, _right_distance);
  }
  
  void setMotorSpeed(int left_d, int right_d)
  {
    if(left_d == 0 && right_d == 0)
    {
      //Stop
      _left_motor_speed = 0;
      _right_motor_speed = 0;
    }
    else if(40 > abs(left_d - right_d))
    {
      if(turn_significance > 0)
      {
	_left_motor_speed = 60 + abs(8 * turn_significance);
	_right_motor_speed = 57;
	
	turn_significance = 0;
	
	// TESTING
	//ROS_INFO("Go Sharp Right	Left: %d	TS: %d", _left_motor_speed, turn_significance);
      }
      else if(turn_significance < 0)
      {
	_left_motor_speed = 57;
	_right_motor_speed = 60 + abs(8 * turn_significance);
	
	turn_significance = 0;
	
	// TESTING
	//ROS_INFO("Go Sharp Left	Right: %d	TS: %d", _right_motor_speed, turn_significance);
      }
      else
      {
	//Go straight
	_left_motor_speed = 50;
	_right_motor_speed = 53;
	
	// TESTING
	//ROS_INFO("Go Straight	Left: %d	Right: %d", _left_motor_speed, _right_motor_speed);
      }
    }
    else if(left_d < right_d)
    {
      //Go right
      _left_motor_speed = 60;
      _right_motor_speed = 57;
      
      // TESTING
      //ROS_INFO("Turn Right	Left: %d	Right: %d", _left_motor_speed, _right_motor_speed);
      
      turn_significance--;
    }
    else if(left_d > right_d)
    {
      //Go left
      _left_motor_speed = 55;
      _right_motor_speed = 70;
      
      // TESTING
      //ROS_INFO("Turn Left	Left: %d	Right: %d", _left_motor_speed, _right_motor_speed);

      turn_significance++;
    }
  }
  
  void assignMotorSpeedData()
  {
    command.data[0] = _right_motor_speed;
    command.data[1] = _left_motor_speed;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "command_node");
  
  CommandConverter c;
  ros::spin();
  
  return 0;
} 
