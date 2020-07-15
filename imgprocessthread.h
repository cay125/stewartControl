#ifndef IMGPROCESSTHREAD_H
#define IMGPROCESSTHREAD_H

#include <QObject>
#include <QThread>
#include <functional>
#include "poseestimation2d2d.h"
#include "poseestimation3d2d.h"
#include "motiondetector.h"

#define __USE_2D2D__ 0

class ImgProcessThread : public QThread
{
public:
    ImgProcessThread(int _method);
    void run() override;
private:
    std::function<bool(cv::Mat&)> GetFrame;
    PoseEstimation estimation_module;
    PoseEstimation3d2d estimation_module3d2d;
    MotionDetector* motion_detector=nullptr;
};

#endif // IMGPROCESSTHREAD_H
