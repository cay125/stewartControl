#ifndef POSEESTIMATION3D2D_H
#define POSEESTIMATION3D2D_H

#include <opencv2/imgproc.hpp>
#include <QVector>
#include <MatlabDataArray.hpp>
#include <MatlabEngine.hpp>

using namespace matlab::engine;

class PoseEstimation3d2d
{
public:
    PoseEstimation3d2d();
    bool InitTriangular(cv::Mat &init1, cv::Mat &init2, double factor);
    QVector<double> GetWorldLoc(cv::Mat& src);
private:
    matlab::data::ArrayFactory factory;
    std::unique_ptr<MATLABEngine> matlabPtr;
};

#endif // POSEESTIMATION3D2D_H
