#include <QDebug>
#include <windows.h>
#include <algorithm>
#include "imgprocessthread.h"
#include "MVCamera.h"
#include "controller.h"

static void start_camera()
{
    MVCamera::Init();
    MVCamera::Play();
    MVCamera::SetExposureTime(false, 20000);
    MVCamera::SetLargeResolution(true);
}

ImgProcessThread::ImgProcessThread(int _method)
{
    connect(this, &ImgProcessThread::finished, this, &QObject::deleteLater);
    if(_method==0)
    {
        qDebug()<<"using laptop camera to get image";
        GetFrame=[](cv::Mat& frame)->bool
        {
            static cv::VideoCapture cap(0);
            return cap.read(frame);
        };
    }
    else if(_method==1)
    {
        qDebug()<<"using industry camera to get image";
        start_camera();
        GetFrame=&MVCamera::GetFrame;
    }
    else
    {
        qDebug()<<"no this method";
        exit(1);
    }
}
void ImgProcessThread::run()
{
    cv::Mat srcImg;
    cv::Mat preImg;
    GetFrame(srcImg);
#if !__USE_2D2D__
    qDebug()<<"start init trangular measure;";

    Mat init1, init2;
    GetFrame(init1);
    std::reverse(init1.data,init1.data+init1.cols*init1.rows*3);
    for(int i=0;i<init1.rows;i++)
        std::reverse(init1.data+init1.cols*3*i,init1.data+init1.cols*3*(i+1));
    motion_detector=new MotionDetector(init1);

    GetFrame(init2);
    std::reverse(init2.data,init2.data+init2.cols*init2.rows*3);
    for(int i=0;i<init2.rows;i++)
        std::reverse(init2.data+init2.cols*3*i,init2.data+init2.cols*3*(i+1));
    bool res=estimation_module3d2d.InitTriangular(init1,init2,40);
    while(!res)
    {
        qDebug()<<"vision init failed";
        QThread::msleep(200);
        GetFrame(init2);
        std::reverse(init2.data,init2.data+init2.cols*init2.rows*3);
        for(int i=0;i<init2.rows;i++)
            std::reverse(init2.data+init2.cols*3*i,init2.data+init2.cols*3*(i+1));
        res=estimation_module3d2d.InitTriangular(init1,init2,40);
    }
    cv::imshow("init1", init1);
    cv::imshow("init2", init2);
#endif
    while(GetFrame(srcImg))
    {
        std::reverse(srcImg.data,srcImg.data+srcImg.cols*srcImg.rows*3);
        for(int i=0;i<srcImg.rows;i++)
            std::reverse(srcImg.data+srcImg.cols*3*i,srcImg.data+srcImg.cols*3*(i+1));
        if (!srcImg.isContinuous())
        {
            qDebug()<<"img not continues!!";
            srcImg = srcImg.clone();
        }
#if __USE_2D2D__
        cv::Mat temp=srcImg.clone();
        if(!preImg.empty())
        {
            auto mats=estimation_module.GetPose2d2d(preImg,srcImg);
            frame_mutex.lock();
            FrameRotation=mats[0];
            FrameTranslation=mats[1];
            frame_mutex.unlock();
#if __SHOW_SRC__
            cv::imshow("pre",preImg);
#endif
        }
#endif
#if __SHOW_SRC__
        cv::imshow("src",srcImg);
        cv::waitKey(2);
#endif
#if __USE_2D2D__
        preImg=temp;
#endif

//        frame_mutex.lock();
//        bool status=globalStableStatus;
//        frame_mutex.unlock();
//        if(status)
//        {
//            qDebug()<<"stable!!!!!!";
//        }
//        else
//        {
//            qDebug()<<"motion!!!!!!";
//        }
//        auto r=motion_detector->Detector(srcImg);
//        std::cout<<"sum: "<<r;

#if !__USE_2D2D__
        auto pos=estimation_module3d2d.GetWorldLoc(srcImg);
        frame_mutex.lock();
        if(FrameTranslation.empty())
            FrameTranslation.create(3,1,CV_64FC1);
        FrameTranslation.at<double>(0,0)=pos[0];
        FrameTranslation.at<double>(1,0)=pos[1];
        FrameTranslation.at<double>(2,0)=pos[2];
        frame_mutex.unlock();
        std::cout<<"Location: ";
        for(auto item : pos)
            std::cout<<item<<" ";
        std::cout<<"\n";
#endif
        QThread::msleep(50);
    }
    qDebug()<<"image thread finished";
}
