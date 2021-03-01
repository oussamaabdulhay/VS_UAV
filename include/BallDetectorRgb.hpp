#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <image_transport/image_transport.h>
#include <cmath>
#include "geometry_msgs/Point.h"
#include <sstream>
#include <ros/ros.h>
#include <iostream>
#include "geometry_msgs/PointStamped.h"
#include "ros/ros.h"

class BallDetectorRgb
{


public:
    BallDetectorRgb(ros::NodeHandle&);
    ~BallDetectorRgb();

private:
    cv::Ptr<cv::SimpleBlobDetector> detector;
    ros::NodeHandle nh_;
    ros::Publisher raw_detection;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    geometry_msgs::PointStamped c_pt;
    const std::string OPENCV_WINDOW = "Image window";
    std::vector<cv::KeyPoint> keypoints;
    int iLowH = 0;
    int iHighH = 49;

    int iLowS = 56;
    int iHighS = 203;

    int iLowV = 186;
    int iHighV = 255;    
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    cv::SimpleBlobDetector::Params params;

};