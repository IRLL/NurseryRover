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
  int compass_value;
  
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
    compass_value = 0;
    
    //Initialize max max_distance
    maxPointDifference = 20;
    max_distance = 1000;//100;
    
    //TODO: Correct this advertise - KAYL
    pub = nh.advertise<geometry_msgs::PoseArray>("/data_processing/clusters", 2);
    sub = nh.subscribe("/lidar_data/points_data", 360, &DataProcessing::ProcessingCallback, this);
    //sub = nh.subscribe("/arduino/compass_value", &DataProcessing::CompassCallback, this);
    
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
    evaluatePoints(msg);
    
    //Evaluates cluster 
    evaluateCluster();
    
    //Show Image
    cv::imshow(OPENCV_WINDOW, image);
    
    cvWaitKey(1);
  }
  /*
  void CompassCallback(const std_msgs::Float32 msg)
  {
    compass_value = msg.data;
  }*/

private:
  void evaluatePoints(const geometry_msgs::PoseArray msg)
  {
    geometry_msgs::Pose tempPose;
    
    for(int i =  0; i < msg.poses.size(); i++)
    {
      tempPose = msg.poses.at(i);
      //TODO: Make condition for boundaries - KAYL
      if(1)
      {
	//TODO: Make condition to decide which cluster to put data in - SHIVAM
	if(1) 
	{
	  RightCluster.push_back(cv::Point2f(tempPose.position.x,tempPose.position.y));
	}
	else if(1)
	{
	  LeftCluster.push_back(cv::Point2f(tempPose.position.x,tempPose.position.y));
	}
      }
    }
  }
  void evaluateCluster()
  {
    int K = 2, attempts = 30, flags = cv::KMEANS_PP_CENTERS;
    cv::Mat labels, centers;
    cv::Point2f tempPoint;
    cv::Mat leftPoints(LeftCluster.size(), 2, CV_32F);
    cv::Mat rightPoints(RightCluster.size(), 2, CV_32F);
    
    //TODO: Make into function so we can just call the function - ANYONE
    for(int i = 0; i < LeftCluster.size(); i++)
    {
      tempPoint = LeftCluster[i];
      leftPoints.at<float>(i, 0) = tempPoint.x;
      leftPoints.at<float>(i, 1) = tempPoint.y;
    }
    cv::kmeans(leftPoints,K,labels,cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0),attempts,flags, centers);

    cv::line(image, cv::Point2f(abs(centers.at<float>(0, 0)), abs(centers.at<float>(0, 1))), cv::Point2f(abs(centers.at<float>(1, 0)), abs(centers.at<float>(1, 1))), cv::Scalar(0,0,255), 2, 8, 0);
    
    for(int i = 0; i < RightCluster.size(); i++)
    {
      tempPoint = RightCluster[i];
      rightPoints.at<float>(i, 0) = tempPoint.x;
      rightPoints.at<float>(i, 1) = tempPoint.y;
    }
    cv::kmeans(rightPoints,K,labels,cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0),attempts,flags, centers);
    cv::line(image, cv::Point2f(abs(centers.at<float>(0, 0)), abs(centers.at<float>(0, 1))), cv::Point2f(abs(centers.at<float>(1, 0)), abs(centers.at<float>(1, 1))), cv::Scalar(0,0,255), 2, 8, 0);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_processing_node");
  
  DataProcessing dp;
  ros::spin();
  
  return 0;
}