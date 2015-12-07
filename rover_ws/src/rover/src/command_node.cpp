/*
 * References:
 *	Values range from 127 to -127
 *
*/

/*
 * Message references:
 *	1 = left distance
 *	2 = right distance
*/

#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

#include "geometry_msgs/Point.h"

#include <ros/console.h>

class CommandConverter
{
private:
	ros::NodeHandle _nh;

        geometry_msgs::Point command;

        //Initialize Publisher and Subscriber
        ros::Publisher pub;
        ros::Subscriber sub;

	int _left_distance;
	int _right_distance;

	int _left_motor_speed;
	int _right_motor_speed;

public:
	CommandConverter()
	{
		pub = _nh.advertise<geometry_msgs::Point>("/command_converter/commands", 1);
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

		//ROS_INFO(command.x);
	}
	void setMotorSpeed(int left_d, int right_d)
	{
		if(left_d == 0 && right_d == 0)
		{
		  //Stop
		  _left_motor_speed = 0;
		  _right_motor_speed = 0;
		}
		else if(left_d == right_d)
		{
			//Go straight
			_left_motor_speed = 50;
			_right_motor_speed = 50;
		}
		else if(left_d < right_d)
		{
			//Go left
			_left_motor_speed = 50;
			_right_motor_speed = 45;
		}
		else if(left_d > right_d)
		{
			//Go right
			_left_motor_speed = 45;
			_right_motor_speed = 50;
		}
	}
	void assignMotorSpeedData()
	{
		command.x = _left_motor_speed;
		command.y = _right_motor_speed;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "command_node");

	CommandConverter c;
	ros::spin();
	
	return 0;
} 
