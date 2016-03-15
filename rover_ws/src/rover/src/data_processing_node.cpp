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
  ros::Publisher straightPub;
  ros::Publisher switchRowsPub;
 
  //Clusters
  std::vector<cv::Point2f> RightCluster;
  std::vector<cv::Point2f> LeftCluster;
  
  //Published data
  geometry_msgs::PoseArray straightPoses;
  geometry_msgs::PoseArray switchRowPoses;

  //values
  int maxDistance;
  int rows;
  int cols;

  int rover_rows;
  int rover_cols;
  int compassValue;
  int compassStartPoint;
  
  double slope;

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
    
    //Pub and Sub
    straightPub = nh.advertise<geometry_msgs::PoseArray>("/data_processing/clusters", 4);
    switchRowsPub = nh.advertise<geometry_msgs::PoseArray>("/data_processing/points", 2);
    sub = nh.subscribe("/lidar_data/points_data", 360, &DataProcessing::ProcessingCallback, this);
    sub = nh.subscribe("/arduino/compass_value", 1, &DataProcessing::CompassCallback, this);
    sub = nh.subscribe("/arduino/compass_start_point", 1, &DataProcessing::CompassStartPointCallback, this);
    
    cv::namedWindow(OPENCV_WINDOW);
  }

private: 
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
    int x, y;
    compassValue = msg.data;
    
    //Calculate front and rear degree
    frontDegree = (compassStartPoint - compassValue) + 90;
    backDegree = frontDegree + 180;

    //Calculate slope
    x = sin(frontDegree) + 400;
    y = cos(frontDegree) + 400;
    slope = (x - rover_cols) / (y - rover_rows); 
  }
  void CompassStartPointCallback(const std_msgs::Float32 msg)
  {
    compassStartPoint = msg.data;
  }

  void setClusters(const geometry_msgs::PoseArray msg)
  {
    geometry_msgs::Pose pose;
    
    //Check if compassStartPoint has been initialized
    if(compassStartPoint == -1)
      return;
    
    double xFront;

    for(int i =  0; i < msg.poses.size(); i++)
    {
      pose = msg.poses.at(i);
      poseDegree = atan(pose.position.y/pose.position.x);
     

      //For the pose find the x value of the center of the rows
      xFront =  pose.position.y / slope;

      //Compare the two x values and see if its good data or not
      if(maxDistance >= abs(xFront - pose.position.x))
      {
        //Data is in range
        //Determine which cluster to add to
        if(xFront < pose.position.x)
        {
          //Add to right cluster
	  RightCluster.push_back(cv::Point2f(pose.position.x,pose.position.y));
        }
        else
        {
          //Add to left cluster
	  LeftCluster.push_back(cv::Point2f(pose.position.x,pose.position.y));
        }
      }
    }
  }
  void evaluateCluster()
  {
    int turn;
    cv::Mat leftPoints(LeftCluster.size(), 2, CV_32F);
    cv::Mat rightPoints(RightCluster.size(), 2, CV_32F);
    
    //TODO: see if need to turn SHIVAM

    if(turn)
    {
      switchRows(leftPoints, LeftCluster);
      swithcRows(rightPoints, RightCluster);

      //TODO: Publish two points SHIVAM 
    }
    else
    {
      straight(leftPoints, LeftCluster);
      straingt(rightPoints, RightCluster);

      //Publish four points KAYL
      straightPub.publish(straightPoses);
      straightPoses.poses.clear();  
    }
  }
  void straight(cv::Mat points,  std::vector<cv::Point2f>& cluster)
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
    
    //Convert points to poses and push to pose array
    geometry_msgs::Pose pose;
    pose.position = point1;
    straightPoses.poses.push_back(pose);
    
    pose.position = point2;
    straightPoses.poses.push_back(pose);
    
  }
  void switchRows(cv::Mat points,  std::vector<cv::Point2f>& cluster)
  {
    cv::Point2f point;

    point.x = 0;
    //TODO: get last point and publish KAYL
    for(int i = 0; i < cluster.size(); i++)
    { 
      //iterate through and find greatest point  
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_processing_node");
  
  DataProcessing dp;
  ros::spin();
  
  return 0;
}
