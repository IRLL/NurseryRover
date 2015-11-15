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

#include "std_msgs/String.h"
#include <sstream>

class CommandConverter
{
private:
	ros::NodeHandle _nh;

        std_msgs::String _command;

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
		pub = _nh.advertise<std_msgs::String>("/command_converter/commands", 100);
		sub = _nh.subscribe("/lidar_data/distances", 4, &CommandConverter::commandConverterCallback, this);

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
		if(temp_array[0] < temp_array[3])
		{
		  _left_distance =  temp_array[0];
		  _right_distance = temp_array[3];	
		}
		else 
		{
		  _left_distance =  temp_array[3];
		  _right_distance = temp_array[0];
		}
		
		setMotorSpeed(_left_distance, _right_distance);

		assignMotorSpeedData();

		pub.publish(_command);


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
		std::stringstream message;
		message << _left_motor_speed << "," << _right_motor_speed;
		_command.data = message.str();
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "command_node");

	CommandConverter c;
	ros::spin();
	
	

	return 0;
} 
