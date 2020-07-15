#include "poseestimation3d2d.h"
#include <QDebug>

PoseEstimation3d2d::PoseEstimation3d2d()
{
    qDebug()<<"starting matlab engine";
    matlabPtr = startMATLAB();
    qDebug()<<"Engine start!!";
}
bool PoseEstimation3d2d::InitTriangular(cv::Mat &_init1, cv::Mat &_init2, double factor)
{
    cv::Mat init1, init2;
    cvtColor(_init1, init1, CV_BGR2GRAY);
    cvtColor(_init2, init2, CV_BGR2GRAY);

    auto img1 = factory.createArray<unsigned char>({ 752,480 }, init1.data, init1.data + init1.rows*init1.cols);
    matlabPtr->setVariable(u"I1", std::move(img1));
    matlabPtr->eval(u"I1=I1';");

    auto img2 = factory.createArray<unsigned char>({ 752,480 }, init2.data, init2.data + init2.rows*init2.cols);
    matlabPtr->setVariable(u"I2", std::move(img2));
    matlabPtr->eval(u"I2=I2';");

    matlabPtr->eval(u"cd 'C:/Users/xiang/Desktop/VisionDeploy'");
    matlabPtr->eval(u"load('../CameraParams/camera2Params.mat')");

    auto _factor = factory.createScalar<double>(factor);
    matlabPtr->setVariable(u"factor", std::move(_factor));

    matlabPtr->eval(u"ScriptInitTriangular");

    matlab::data::TypedArray<double> goodimg = matlabPtr->getVariable(u"GoodImg");
    if(goodimg[0] > 0.5)
        return true;
    else
        return false;
}
QVector<double> PoseEstimation3d2d::GetWorldLoc(cv::Mat& simg)
{
    static int cnt=3;
    cv::Mat src;
    cvtColor(simg, src, CV_BGR2GRAY);
    auto img = factory.createArray<unsigned char>({ 752,480 }, src.data, src.data + src.rows*src.cols);
    matlabPtr->setVariable(u"Irgb", std::move(img));
    matlabPtr->eval(u"Irgb=Irgb';");
    auto viewId = factory.createScalar<int>(cnt);
    matlabPtr->setVariable(u"viewId", std::move(viewId));
    matlabPtr->eval(u"ScriptGetWorldLoc");
    matlab::data::TypedArray<double> goodimg = matlabPtr->getVariable(u"GoodImg");
    if(goodimg[0] < 0.5)
    {
        qDebug()<<"failed procee";
        return QVector<double>();
    }
    matlab::data::TypedArray<double> res = matlabPtr->getVariable(u"position");
    cnt++;
    QVector<double> loc;
    for(auto item : res)
        loc.push_back(item);
    return loc;
}
