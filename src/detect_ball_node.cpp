#include "BallDetectorRgb.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detection_rgb_node");
  ros::NodeHandle nh_;
  BallDetectorRgb* detectBall=new BallDetectorRgb(nh_);

  ros::Rate r(60); 
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  
}