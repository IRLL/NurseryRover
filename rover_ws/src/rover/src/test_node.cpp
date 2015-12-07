#include "ros/ros.h"
#include "geometry_msgs/Point.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv, "talker");
  ros::NodeHandle n;
  
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Point>("poses", 1);
  ros::Rate loop_rate(10);
  
  geometry_msgs::Point p;
  p.x = 11;
  p.y = 12;
  
  while (ros::ok())
  {
    chatter_pub.publish(p);
  }
  
  return 0;
}