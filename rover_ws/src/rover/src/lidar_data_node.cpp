
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
		max_distance = 325;

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
				
				sendImage();
				
				//Reinitialize image to zeros
				image.setTo(cv::Scalar(255, 255, 255));
				
				//Reset Points and Cluster
				frontPoints.clear();
				rearPoints.clear();
				frontCluster.clear();
				rearCluster.clear();
			}
			
			//Draw points on image
			//From 0-180 degrees
			calculatePoint(data[1], data[2]);
			
			if(object_cols != 0)
			  checkCluster(front);
			  addPointToCluster(front);

			//From 180-360 degrees
			calculatePoint(data[1] + 180, data[3]);
			
			if(object_cols != 0)
			{
			  checkCluster(rear);
			  addPointToCluster(rear);
			}  
		}
	}

private:
	//Checks for new cluster
	void checkCluster(int direction)
	{
	  cv::Point2f point;
	  
	  //dirction = 1 when facing forward
	  if(direction == 1)
	  {
	   //check distances to see if this is a new cluster
	    for(int i = 0; i < frontCluster.size(); i++)
	    {
	      point = frontCluster[i];
	      
	      //if new cluster then eval last cluster
	      if(std::abs(object_cols - point.x) > maxPointDifference)
	      {
		evaluateCluster(direction);
		break;
	      }
	      else if(std::abs(object_rows - point.y) > maxPointDifference)
	      {
	        evaluateCluster(direction);
		break;
	      }
	    }
	  }
	  //dirction = 2 when facing forward
	  else
	  {
	   //check distances to see if this is a new cluster
	    for(int i = 0; i < rearCluster.size(); i++)
	    {
	      point = rearCluster[i];
	      
	      //if new cluster then eval last cluster
	      if(std::abs(object_cols - point.x) > maxPointDifference)
	      {
		evaluateCluster(direction);
		break;
	      }
	      else if(std::abs(object_rows - point.y) > maxPointDifference)
	      {
	        evaluateCluster(direction);
		break;
	      }
	    }
	  }
	}
	
	void evaluateCluster(int direction)
	{
	  std::vector<cv::Point2f> center(1);
	  cv::Mat labels;
	  int clusterCount = 1;
	  int sampleCount;
	  
		    
	  if(direction == 1)
	  {
	    sampleCount = frontCluster.size();
	    
	    //EvaluateCluster
	    cv::kmeans(frontCluster, clusterCount, labels, cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 3, cv::KMEANS_PP_CENTERS, center);
	    //Add point
	    frontPoints.push_back(center[0]);
	    
	    //Clear cluster to start fresh
	    frontCluster.clear();
	  }
	  else
	  {
	    sampleCount = rearCluster.size();
	    
	    //EvaluateCluster
	    cv::kmeans(rearCluster, clusterCount, labels, cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 3, cv::KMEANS_PP_CENTERS, center);
	    //Add point
	    rearPoints.push_back(center[0]);
	    
	    //Clear cluster to start fresh
	    rearCluster.clear();
	    
	    //Draw lines between points
	    processPoints(direction);
	  }
	}
	void processPoints(int direction)
	{
	  if(direction == 1)
	  {
	    //check if more than two points
	    if(frontPoints.size() > 1)
	    {
	      cv::Point2f oldPoint = frontPoints[frontPoints.size() - 1];
	      cv::Point2f newPoint = frontPoints[frontPoints.size() - 2];
	      
	      double pointDistance = calculateDistance(oldPoint, newPoint);
		
	      //check if it is in the same row
	      if(pointDistance < maxPointDifference)
	      {
		//In same row then draw lines
		cv::line(image, oldPoint, newPoint, cv::Scalar(0,0,255), 3, 8,0);
	      }
	    }
	  }
	  else
	  {
	    //check if more than two points
	    if(rearPoints.size() > 1)
	    {
	      cv::Point2f oldPoint = rearPoints[rearPoints.size() - 1];
	      cv::Point2f newPoint = rearPoints[rearPoints.size() - 2];
	      
	      double pointDistance = calculateDistance(oldPoint, newPoint);
		
	      //check if it is in the same row
	      if(pointDistance < maxPointDifference)
	      {
		//In same row then draw lines
		cv::line(image, oldPoint, newPoint, cv::Scalar(0,0,255), 3, 8,0);
	      }
	    }
	  }
	}
	double calculateDistance(cv::Point2f oldPoint, cv::Point2f newPoint)
	{
	  double y, x;
	  
	  y = abs(oldPoint.y - newPoint.y);
	  x = abs(oldPoint.x - newPoint.x);
	  
	  return y/x;	  
	}
	void addPointToCluster(int direction)
	{
	  cv::Point2f point;
	  point.x = object_cols;
	  point.y = object_rows;
	  
	  if(direction == 1)
	    frontCluster.push_back(point);
	  else
	    rearCluster.push_back(point);
	}
	
	void calculatePoint(int deg, int dis)
	{
	  int temp_distance;
	  
		//Get point_rows
		if(deg > 180)
		{
		      //Front of rover
		      object_rows = 400 - (dis * cos(deg) + .5);
		}
		else
		{
		      //Rear of rover
		      object_rows = 400 + (dis * cos(deg) + .5);
		}
		
		//Get point_cols
		if(deg < 90)
		{
		      //Left side of rover
		      temp_distance = dis * sin(deg) + .5;
		      
		      //Checks if object is out of range
		      if(temp_distance <= max_distance)
			object_cols = 400 - temp_distance;
		      else
			object_cols = 0;
		}
		else
		{
		      //Right side of rover
		      temp_distance = dis * sin(deg) + .5;
		      
		      //Checks if object is out of range
		      if(temp_distance <= max_distance)
			object_cols = 400 + temp_distance;
		      else
			object_cols = 0;
		}
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












