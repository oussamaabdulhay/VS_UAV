#include "BallDetectorRgb.hpp"


BallDetectorRgb::BallDetectorRgb(ros::NodeHandle &main_nodehandle)
    : it_(nh_)
{
  nh_=main_nodehandle;

  image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &BallDetectorRgb::imageCb, this);
  raw_detection = nh_.advertise<geometry_msgs::PointStamped>("/ball_center", 1);
    
  // puby = nh_.advertise<std_msgs::Float32>("camera_provider_y", 1);
  // pubx = nh_.advertise<std_msgs::Float32>("camera_provider_x", 1);
  
  cv::namedWindow(OPENCV_WINDOW);

  params.filterByArea = true;
  params.minArea = 100;
  params.maxArea = 5000;

  // Filter by Circularity
  params.filterByCircularity = true;
  params.minCircularity = 0.5;

  // Filter by Convexity
  params.filterByConvexity = true;
  params.minConvexity = 0.5;

  // Filter by Inertia
  params.filterByInertia = false;
  params.minInertiaRatio = 0.6;
  detector = cv::SimpleBlobDetector::create(params);

}

BallDetectorRgb::~BallDetectorRgb()
{
  cv::destroyWindow(OPENCV_WINDOW);
}

void BallDetectorRgb::imageCb(const sensor_msgs::ImageConstPtr &msg)

{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {

        ROS_ERROR("cv_bridge exception: %s", e.what());

        return;
    }
    cv::Mat imgOriginal = cv_ptr->image;

    cv::Mat imgHSV;

    cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV);

    cv::Mat blurred,im_with_keypoints;
    cv::GaussianBlur(imgHSV, blurred, cv::Size(11, 11), 0, 0);

    cv::Mat imgThresholded;

    cv::inRange(blurred, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);


    cv::bitwise_not(imgThresholded, imgThresholded);
     
    detector->detect(imgThresholded, keypoints);
    std::cout<<keypoints.size()<<std::endl;

  if (keypoints.size() == 1) // to be removed
  {
    c_pt.header = msg->header;
    c_pt.point.x = keypoints[0].pt.x;
    c_pt.point.y = keypoints[0].pt.y;

    raw_detection.publish(c_pt);
    
  }
  else{
    std::cout << "Not Detected\n";

  }

    // cv::createTrackbar("LowH", OPENCV_WINDOW, &iLowH, 255);
    // cv::createTrackbar("HighH", OPENCV_WINDOW, &iHighH, 255);

    // cv::createTrackbar("LowS", OPENCV_WINDOW, &iLowS, 255);
    // cv::createTrackbar("HighS", OPENCV_WINDOW, &iHighS, 255);

    // cv::createTrackbar("LowV", OPENCV_WINDOW, &iLowV, 255);
    // cv::createTrackbar("HighV", OPENCV_WINDOW, &iHighV, 255);
    // cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
    // cv::imshow("Original", imgOriginal);             //show the original image
    // cv::imshow("im_with_keypoints", im_with_keypoints); 
    // cv::waitKey(1);
}

