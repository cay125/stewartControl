#include "motiondetector.h"

MotionDetector::MotionDetector(cv::Mat& initFrame)
{
    cv::Mat gray;
    cv::cvtColor(initFrame,gray,CV_BGR2GRAY);
    preFrame=gray;
}
cv::Scalar MotionDetector::Detector(cv::Mat &Frame)
{
    cv::Mat gray, diff;
    cv::cvtColor(Frame,gray,CV_BGR2GRAY);
    absdiff(preFrame, gray, diff);
    cv::Scalar diff_sum=cv::sum(diff)/diff.rows/diff.cols;
    preFrame=gray;
    return diff_sum;
}
