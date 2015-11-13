/*
 * References:
 *
*/

#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "std_msgs/String.h"
#include <sstream>

//Open window
static const std::string OPENCV_WINDOW = "Image";

class ImageAnalysis
{
private:
	ros::NodeHandle nh;
	
	image_transport::ImageTransport it;
        image_transport::Subscriber sub;

	//cv_bridge::CvImagePtr image_ptr[3];
	cv_bridge::CvImagePtr image_ptr;
	int image_count;
	int max_saved_image_count;

	ros::Publisher pub;
	std_msgs::String rover_commands;

public:
	ImageAnalysis():it(nh)
	{
		image_count = -1;
		max_saved_image_count = 5;
		
		pub = nh.advertise<std_msgs::String>("/image_analysis_node/rover_commands", 100);
		//sub = it.subscribe("/lidar_data_node/output_image", 1, &ImageAnalysis::analysisCallback, this);
		sub = it.subscribe("/test_image", 1, &ImageAnalysis::analysisCallback, this);
	
		cv::namedWindow(OPENCV_WINDOW);

	}
	void analysisCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		//There are n amount of images
		//The oldest image is erased and replaced with the most recent
		if(image_count < max_saved_image_count)
			image_count++;
		else
			image_count = 0;
		  
		try
		{
			
			//image_ptr[image_count] = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                }
                catch(cv_bridge::Exception& e)
                {
                        ROS_ERROR("cv_bridge exception: %s", e.what());
			
			//Reset image count to what it was before callback
			if(image_count > 0)
			  image_count--;
			else
			  image_count = max_saved_image_count;
			
                        return;
                }
               
		
		//Show image
		//cv::imshow(OPENCV_WINDOW, image_ptr[image_count]->image);		
		cv::imshow(OPENCV_WINDOW, image_ptr->image);	
		cvWaitKey(1);
	}
	void commanGenerator(int left_speed, int right_speed)
	{
	  std::stringstream ss;
	  
	  ss << left_speed << "," << right_speed;
	  
	  rover_commands.data = ss.str();
	  
	  pub.publish(rover_commands);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_analysis_node");
	
	ImageAnalysis ia;
	ros::spin();
	
	return 0;
}