#ifndef IMGPROCESSTHREAD_H
#define IMGPROCESSTHREAD_H

#include <QObject>
#include <QThread>
#include <functional>
#include "poseestimation2d2d.h"

class ImgProcessThread : public QThread
{
public:
    ImgProcessThread(int _method);
    void run() override;
private:
    std::function<bool(cv::Mat&)> GetFrame;
    PoseEstimation estimation_module;
};

#endif // IMGPROCESSTHREAD_H
