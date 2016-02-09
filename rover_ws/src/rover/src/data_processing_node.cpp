/*
 * References:
 * 
 * 	
 *
*/

/*
 * Variable references:
 *	
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

#include <math.h>

#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Float32.h"

//Open window
static const std::string OPENCV_WINDOW = "Image";

class DataProcessing
{
private:
  ros::NodeHandle nh;
  
  ros::Subscriber sub;
  ros::Publisher pub;
  
  double data[4];
  
  //Clusters
  std::vector<cv::Point2f> RightCluster;
  std::vector<cv::Point2f> LeftCluster;
  
  //values
  int maxPointDifference;
  int max_distance;
  int rows;
  int cols;

  int rover_rows;
  int rover_cols;
  int compassValue;
  int compassStartPoint;
  
  cv::Mat image;
	
public:
  DataProcessing()
  {    
    //Initialize size of image
    rows = 800;
    cols = 800;
    
    //Initialize image
    image = cv::Mat::zeros(cols, rows, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255));
    
    //Initialize location of rover
    rover_rows = rows/2;
    rover_cols = cols/2;
    
    //Initialize compass
    compassValue = 0;
    compassStartPoint = -1;
    
    //Initialize max max_distance
    maxPointDifference = 20;
    max_distance = 1000;//100;
    
    //TODO: Correct this advertise - KAYL
    pub = nh.advertise<geometry_msgs::PoseArray>("/data_processing/clusters", 2);
    sub = nh.subscribe("/lidar_data/points_data", 360, &DataProcessing::ProcessingCallback, this);
    sub = nh.subscribe("/arduino/compass_value", 1, &DataProcessing::CompassCallback, this);
    sub = nh.subscribe("/arduino/compass_start_point", 1, &DataProcessing::CompassStartPointCallback, this);
    
    cv::namedWindow(OPENCV_WINDOW);
    
  }
  
  void ProcessingCallback(const geometry_msgs::PoseArray msg)
  {
    //Reinitialize image to zeros
    image.setTo(cv::Scalar(255, 255, 255));
    
    //Reset Points and Cluster
    LeftCluster.clear();
    RightCluster.clear();
    
    //Set valid points
    setClusters(msg);
    
    //Evaluates cluster 
    evaluateCluster();
    
    //Show Image
    cv::imshow(OPENCV_WINDOW, image);
    
    cvWaitKey(1);
  }
  void CompassCallback(const std_msgs::Float32 msg)
  {
    compassValue = msg.data;
  }
  void CompassStartPointCallback(const std_msgs::Float32 msg)
  {
    compassStartPoint = msg.data;
  }

private:
  void setClusters(const geometry_msgs::PoseArray msg)
  {
    geometry_msgs::Pose pose;
    
    //Check if compassStartPoint has been initialized
    if(compassStartPoint == -1)
      return;
    
    double tempXFront, tempXBack, poseDegree;
    
    for(int i =  0; i < msg.poses.size(); i++)
    {
      pose = msg.poses.at(i);
      poseDegree = atan(pose.position.y/pose.position.x);
      
      //Set temporary variables
      if(compassStartPoint >= 180)
      {
	tempXFront = pose.position.y/tan(compassStartPoint) + cols/2;
	tempXBack = pose.position.y/tan(compassStartPoint - 180) + cols/2;
      }
      else
      {
	tempXFront = pose.position.y/tan(compassStartPoint) + cols/2;
	tempXBack = pose.position.y/tan(compassStartPoint) + cols/2;
      }
      
      if(max_distance >= abs(tempXFront - pose.position.x))
      {
	if(tempXFront < pose.position.x) 
	{
	  RightCluster.push_back(cv::Point2f(pose.position.x,pose.position.y));
	}
	else
	{
	  LeftCluster.push_back(cv::Point2f(pose.position.x,pose.position.y));
	}
      }
      else if(max_distance >= abs(tempXBack - pose.position.x))
      {
	if(tempXBack < pose.position.y) 
	{
	  RightCluster.push_back(cv::Point2f(pose.position.x,pose.position.y));
	}
	else
	{
	  LeftCluster.push_back(cv::Point2f(pose.position.x,pose.position.y));
	}
      }
    }
  }
  void evaluateCluster()
  {
    cv::Mat leftPoints(LeftCluster.size(), 2, CV_32F);
    cv::Mat rightPoints(RightCluster.size(), 2, CV_32F);
    
    evaluatePoints(leftPoints, LeftCluster);
    evaluatePoints(rightPoints, RightCluster);
    
    
  }
  void evaluatePoints(cv::Mat points,  std::vector<cv::Point2f>& cluster)
  {
    int K = 2, attempts = 30, flags = cv::KMEANS_PP_CENTERS;
    cv::Mat labels, centers;
    cv::Point2f tempPoint;
    
    for(int i = 0; i < cluster.size(); i++)
    {
      tempPoint = cluster[i];
      points.at<float>(i, 0) = tempPoint.x;
      points.at<float>(i, 1) = tempPoint.y;
    }
    cv::kmeans(points,K,labels,cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0),attempts,flags, centers);
    cv::Point2f point1 = cv::Point2f(abs(centers.at<float>(0, 0)), abs(centers.at<float>(0, 1)));
    cv::Point2f point2 = cv::Point2f(abs(centers.at<float>(1, 0)), abs(centers.at<float>(1, 1)));

    //draw line between two points
    cv::line(image, point1, point2, cv::Scalar(0,0,255), 2, 8, 0);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_processing_node");
  
  DataProcessing dp;
  ros::spin();
  
  return 0;
}