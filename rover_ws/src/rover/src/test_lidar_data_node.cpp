
/*
 * References:
 * 
 * 
 *
*/

/*
 * Variable references:
 *	msg[0] is count
 *	msg[1] is theta
 *	msg[2] is degree from 0-180
 *	msg[3] is degree from 180-360
 *
*/

#include <ros/ros.h>
#include <ros/console.h>

//Image Analysis
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <std_msgs/String.h>

#include <math.h>

//String splitter
#include <sstream>
#include <string>


//Open window
static const std::string OPENCV_WINDOW = "Image";

class LidarDataConverter
{
private:
	ros::NodeHandle nh;
	
        ros::Subscriber sub;

	//Ros Image Trans
	image_transport::ImageTransport _it;
	image_transport::Publisher pub;	

	double data[4];
	
	//Clusters
	std::vector<cv::Point2f> frontCluster;
	std::vector<cv::Point2f> rearCluster;
	
	//Points
	std::vector<cv::Point2f> frontPoints;
	std::vector<cv::Point2f> rearPoints;
	
	//values
	int front;
	int rear;
	
	int maxPointDifference;
	int max_distance;
	int rows;
	int cols;

	int object_rows;
	int object_cols;
	int rover_rows;
	int rover_cols;

	double count;
	bool program_start;
	
	cv::Mat image;
	
public:

	LidarDataConverter():_it(nh)
	{
		//Initialize front and back
		front = 1;
		rear = 2;
		
		//Initialize size of image
		rows = 800;
		cols = 800;
		
		//Initialize image
		image = cv::Mat::zeros(cols, rows, CV_8UC3);
		image.setTo(cv::Scalar(255, 255, 255));

		//Initialize location of rover
		rover_rows = rows/2;
		rover_cols = cols/2;

		//Initialize max max_distance
		maxPointDifference = 20;
		max_distance = 400;

		program_start = true;
		pub = _it.advertise("/lidar_data_node/output_image", 1);
		sub = nh.subscribe("/arduino/data", 100, &LidarDataConverter::converterCallback, this);
		
		cv::namedWindow(OPENCV_WINDOW);
		
	}
	void converterCallback(const std_msgs::String::ConstPtr& msg)
	{
	  
		//Convert data from string to double
		//String splitter
	  std::stringstream ss(msg->data.c_str());
	  std::string token;
	  
	  for(int temp_count = 0; std::getline(ss, token, ','); temp_count++)
	  {
	    data[temp_count] = std::atof(token.c_str());
	  }
		
			
			//Draw points on image
			//From 0-180 degrees
			
			calculatePoint(data[1], data[2]);
			
			//if(object_cols != 0)
			cv::circle(image, cv::Point(object_cols, object_rows), 3, cv::Scalar(0,0,255), CV_FILLED, 8, 0);
			
			//From 180-360 degrees
			calculatePoint(data[1] + 180, data[3]);
			
			//if(object_cols != 0)
			cv::circle(image, cv::Point(object_cols, object_rows), 3, cv::Scalar(0,0,255), CV_FILLED, 8, 0);

			

//ROS_INFO("dis = %d	cos = %d	sin = %d", int(data[3]), int(data[3] * cos(90 * (M_PI / 180))), int(data[3] * sin(90  * (M_PI / 180))));

	
			
			//Show Image
		cv::imshow(OPENCV_WINDOW, image);
		
		cvWaitKey(1);
	}

private:	
	void calculatePoint(int deg, int dis)
	{
	  object_cols = 400 + (dis * cos(deg * (M_PI / 180)));
	  object_rows = 400 - (dis * sin(deg * (M_PI / 180)));
	}
	void sendImage()
	{
		//Convert to message	
		sensor_msgs::ImagePtr image_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		
		//Show Image
		//cv::imshow(OPENCV_WINDOW, image_ptr);
		
		cvWaitKey(1);

		//Publish Image
		pub.publish(image_ptr);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_data_node");

	LidarDataConverter ld;
	ros::spin();

	return 0;
}

