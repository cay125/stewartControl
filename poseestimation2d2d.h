#ifndef POSEESTIMATION2D2D_H
#define POSEESTIMATION2D2D_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <QVector>

class PoseEstimation
{
public:
    PoseEstimation();
    QVector<cv::Mat> GetPose2d2d(cv::Mat& img1,cv::Mat& img2);
};

#endif // POSEESTIMATION2D2D_H
