#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <opencv2/core/types.hpp>

class medianFilter
{
    public:
    std::vector<float> points;

    float getStdDev(std::vector<cv::Point2f>);
    cv::Point2f getMedian(std::vector<cv::Point2f>,cv::Point2f);
    std::vector<float> getL2Norm(std::vector<cv::Point2f>);
    medianFilter();
    ~medianFilter();
};