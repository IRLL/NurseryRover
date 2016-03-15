/*
 * References:
 *	
 *
*/

/*
 * Notes:
 * 	1. Message Notes:
 *		A. 4 points
 *		B. First two on left
 *		C. Second two on right
 * 	2. Motor Speed:
 * 		A. Values range from 127 to -127
 * 
*/

#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

#include <ros/console.h>

class StraightCommandConverter
{
private:
  ros::NodeHandle _nh;
  
  std_msgs::Int32MultiArray command;
  
  //Declare Publisher and Subscriber
  ros::Publisher pub;
  ros::Subscriber sub;
  
  int _left_motor_speed;
  int _right_motor_speed;

  int roverX, roverY;

public:
  StraightCommandConverter()
  {
    turn_significance = 0;

    roverX = 400;
    roverY = 400;

    pub = _nh.advertise<std_msgs::Int32MultiArray>("/command_converter/commands", 2);
    sub = _nh.subscribe("/lidar_data/cluster", 4, &CommandConverter::commandConverterCallback, this);
  }
	
private:
  void commandConverterCallback(const geometry_msgs::PoseArray msg)
  {
    double slope, leftSlope, rightSlope, leftDistance, rightDistance;
    double leftX, rightX;
    //Calculate average slope of left and right lines
    leftSlope = (msg.poses.at(0).position.x - msg.poses.at(1).position.x)/(msg.poses.at(0).position.y - msg.poses.at(1).position.y);
    rightSlope = (msg.poses.at(2).position.x - msg.poses.at(3).position.x)/(msg.poses.at(2).position.y - msg.poses.at(3).position.y);

    slope = (leftSlope + rightSlope) / 2;
  
    //Calculate x of left and right
    //Formula : y-y1=m(x-x1) -> x1=x-(y-y1)/m  
    leftX = msg.poses.at(0).position.x - (msg.poses.at(0).position.y - roverY) / leftSlope;
    rightX = msg.poses.at(2).position.x - (msg.poses.at(2).position.y - roverY) / rightSlope;

    //Calculate distances
    leftDistance = abs(roverX - leftX);
    rightDistance = abs(roverX - rightX);

    //TODO: redo this function and have it integrate slope in speeds
    //Set motor speed
    setMotorSpeed(leftDistance, rightDistance);
    
    assignMotorSpeedData();
    
    pub.publish(command);
    
    // TESTING
    //ROS_INFO("Left: %d	Right: %d", _left_distance, _right_distance);
  }
  
  void setMotorSpeed(double left_d, double right_d)
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
