#ifndef MOTIONDETECTOR_H
#define MOTIONDETECTOR_H

#include <opencv2/imgproc.hpp>


class MotionDetector
{
public:
    MotionDetector(cv::Mat& initFrame);
    cv::Scalar Detector(cv::Mat& Frame);
private:
    cv::Mat preFrame;
};

#endif // MOTIONDETECTOR_H
