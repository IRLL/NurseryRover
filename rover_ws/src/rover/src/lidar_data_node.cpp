/*
 * References:
 * 
 * 	How to run arduino: rosrun rosserial_python serial_node.py /dev/ttyUSB0
 *
*/

/*
 * Variable references:
 *	msg[0] is theta
 *	msg[1] is degree from 0-180
 *	msg[2] is degree from 180-360
 *	msg[3] is count
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

#include <std_msgs/Float32MultiArray.h>

#include "geometry_msgs/PoseArray.h"

//Open window
static const std::string OPENCV_WINDOW = "Image";

class LidarDataConverter
{
private:
  ros::NodeHandle nh;
  
  ros::Subscriber sub;
  ros::Publisher pub;
  
  double data[4];
  
  //Clusters
  std::vector<cv::Point2f> Cluster;
  
  int max_distance;
  int rows;
  int cols;

  int object_rows;
  int object_cols;

  double count;
  bool program_start;
  
  cv::Mat image;
  geometry_msgs::PoseArray points;
	
public:
  LidarDataConverter()
  {    
    //Initialize size of image
    rows = 800;
    cols = 800;
    
    //Initialize image
    image = cv::Mat::zeros(cols, rows, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255));
    
    //Initialize max_distance
    max_distance = 1000;
    
    program_start = true;
    pub = nh.advertise<geometry_msgs::PoseArray>("/lidar_data/points_data", 360);
    sub = nh.subscribe("/arduino/data", 4, &LidarDataConverter::converterCallback, this);
    
    cv::namedWindow(OPENCV_WINDOW);
    
  }
  void converterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
  {
    for(int i = 0; i < 4; i++)
      data[i] = msg->data[i];
    
    //Representing start of program and initializing count 
    if(program_start == true)
    {
      count = (int)data[3];
	
      //Setting start as untrue
      program_start = false;
    }
    else
    {
      //If we are starting a new set of points
      if(count != (int)data[3])
      {
	count = (int)data[3];
	
	//Reinitialize image to zeros
	image.setTo(cv::Scalar(255, 255, 255));
	
	pub.publish(points);
	
	//Reset Points
	points.poses.clear();
      }
      
      //Draw points on image
      //From 0-180 degrees
      calculatePoint(data[0], data[1]);
      
      //From 180-360 degrees
      calculatePoint(data[0] + 180, data[2]);
      
      //Show Image
      cv::imshow(OPENCV_WINDOW, image);
      
      cvWaitKey(1);
      
    }
    
  }

private:
  void calculatePoint(int deg, int dis)
  {
    object_cols = 400 + (dis * cos(deg * (M_PI / 180)));
    object_rows = 400 - (dis * sin(deg * (M_PI / 180)));
    
    if((object_cols >= 0 && object_cols <= 800) && (object_rows >= 0 && object_rows <= 800))
    { 
      geometry_msgs::Pose pose;
      pose.position.x = object_cols;
      pose.position.y = object_rows;
      points.poses.push_back(pose);
      
      cv::circle(image, cv::Point2f(object_cols,object_rows), 3, cv::Scalar(255,0,255), CV_FILLED, 8, 0);
    }
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_data_node");
  
  LidarDataConverter ld;
  ros::spin();
  
  return 0;
}
