
/*
 * References:
 * 
 * 	How to run arduino: rosrun rosserial_python serial_node.py /dev/ttyUSB0
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
	std::vector<cv::Point2f> Cluster;
	
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
		max_distance = 200;

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

//Representing start of program and initializing count 
		if(program_start == true)
		{
			count = data[0];
			
			//Setting start as untrue
			program_start = false;
		}
		else
		{
			//If we are starting a new set of points
			if(count != data[0])
			{
				count = data[0];
				
				//Reinitialize image to zeros
				image.setTo(cv::Scalar(255, 255, 255));
				
				evaluateCluster();
				
				sendImage();
			
				//Reset Points and Cluster
				Cluster.clear();
			}
			
			//Draw points on image
			//From 0-180 degrees
			cv::Point2f tempPoint;
			calculatePoint(data[1], data[2]);
			
			tempPoint.x = object_cols;
			tempPoint.y = object_rows;
			cv::circle(image, tempPoint, 3, cv::Scalar(0,255,255), CV_FILLED, 8, 0);
			
			//From 180-360 degrees
			calculatePoint(data[1] + 180, data[3]);
			
			tempPoint.x = object_cols;
			tempPoint.y = object_rows;
			cv::circle(image, tempPoint, 3, cv::Scalar(0,255,255), CV_FILLED, 8, 0);
			
			//Show Image
		cv::imshow(OPENCV_WINDOW, image);
		
		cvWaitKey(1);
		}
	}

private:	
	void evaluateCluster()
	{
	  int K = 2, attempts = 30, flags = cv::KMEANS_PP_CENTERS;
	  cv::Mat labels, centers;
	  cv::Point2f tempPoint;
	  cv::Mat points(Cluster.size(), 2, CV_32F);
	  
	  for(int i = 0; i < Cluster.size(); i++)
	  {
	    tempPoint = Cluster[i];
	    points.at<float>(i, 0) = tempPoint.x;
	    points.at<float>(i, 1) = tempPoint.y;
	  }
	  
	  cv::kmeans(points,K,labels,cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0),attempts,flags, centers);
	  
	 
	  for(int i = 0; i < centers.rows; i++)
	  {
	   tempPoint.x = abs(centers.at<float>(i, 0));
	   tempPoint.y = abs(centers.at<float>(i, 1));
	      
	   cv::circle(image, tempPoint, 3, cv::Scalar(0,0,255), CV_FILLED, 8, 0);
	  }
	}
	void calculatePoint(int deg, int dis)
	{
	  object_cols = 400 + (dis * cos(deg * (M_PI / 180)));
	  object_rows = 400 - (dis * sin(deg * (M_PI / 180)));
	  
	  if((object_cols >= 0 && object_cols <= 800) && (object_rows >= 0 && object_rows <= 800))
	  { 
	   Cluster.push_back(cv::Point2f(object_cols,object_rows)); 
	  }
	}
	void sendImage()
	{
		//Convert to message	
		sensor_msgs::ImagePtr image_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

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

