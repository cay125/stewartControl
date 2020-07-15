#include <vector>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include "poseestimation2d2d.h"
#include "timemeasure.h"

using namespace cv;
PoseEstimation::PoseEstimation()
{

}
static void find_feature_matches(const Mat& img_1, const Mat& img_2, std::vector<KeyPoint>& keypoints_1,  std::vector<KeyPoint>& keypoints_2, std::vector< DMatch >& matches)
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    std::vector<DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_1, descriptors_2, match);

    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = match[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    //printf("-- Max dist : %f \n", max_dist);
    //printf("-- Min dist : %f \n", min_dist);

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (match[i].distance <= max(2 * min_dist, 30.0))
        {
            matches.push_back(match[i]);
        }
    }
#if __SHOW_DEBUG_IMG__
    Mat out;
    drawMatches(img_1,keypoints_1,img_2,keypoints_2,matches,out);
    imshow("match",out);
#endif
}
static void pose_estimation_2d2d(std::vector<KeyPoint>& keypoints_1, std::vector<KeyPoint>& keypoints_2, std::vector< DMatch >& matches, Mat& R, Mat& t)
{
    // 相机内参,TUM Freiburg2
    //Mat K = (Mat_<double>(3, 3) << 693.3888, 0, 388.3396, 0, 692.5006, 221.9705, 0, 0, 1);
    Mat K = (Mat_<double>(3, 3) << 3769.9, 0, 400.7, 0, 3885.4, 605.4, 0, 0, 1);

    //-- 把匹配点转换为vector<Point2f>的形式
    std::vector<Point2f> points1;
    std::vector<Point2f> points2;

    for (int i = 0; i < (int)matches.size(); i++)
    {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    //-- 计算本质矩阵
    Point2d principal_point(388, 221);	//相机光心, TUM dataset标定值
    double focal_length = 693.388;			//相机焦距, TUM dataset标定值
    Mat essential_matrix;
    //essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    essential_matrix = findEssentialMat(points1, points2, K);
    //std::cout << "essential_matrix is " << std::endl << essential_matrix << std::endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    //recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    recoverPose(essential_matrix, points1, points2, K, R, t);
    //std::cout << "R is " << std::endl << R << std::endl;
    std::cout << "t is " << std::endl << t << std::endl;

}
static int findcorners(const cv::Mat& ChessImage, const cv::Size& patternsize, std::vector<cv::Point2f>& ChessPoint)
{
    cv::Mat ImageGray;
    if (ChessImage.empty())
    {
        std::cout << "ChessImage Empty" << std::endl;
        return 4;
    }

    if (0 == cv::findChessboardCorners(ChessImage, patternsize, ChessPoint, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK))
    {
        std::cout << "can not find chessboard corners!\n"; //找不到角点
        return false;
    }

    if (ChessImage.channels() == 3)
    {
        cv::cvtColor(ChessImage, ImageGray, CV_RGB2GRAY);
    }
    else
    {
        ChessImage.copyTo(ImageGray);
    }

    cv::find4QuadCornerSubpix(ImageGray, ChessPoint, cv::Size(5, 5));

    //确保角点从上往下，从左往右排列
    if (ChessPoint[0].x > ChessPoint.back().x)
    {
        //交换列
        for (int i = 0; i < (int)patternsize.height; i++)  //行
            for (int j = 0; j < (int)patternsize.width / 2; j++) //列
                std::swap(ChessPoint[i*patternsize.width + j], ChessPoint[(i + 1)*patternsize.width - j - 1]);
    }
    if (ChessPoint[0].y > ChessPoint.back().y)
    {
        //交换行
        for (int i = 0; i < (int)patternsize.width; i++) //列
            for (int j = 0; j < (int)patternsize.height / 2; j++) //行
                std::swap(ChessPoint[j*patternsize.width + i], ChessPoint[(patternsize.height - j - 1)*patternsize.width + i]);
    }
#if __SHOW_DEBUG_IMG__
    cv::drawChessboardCorners(ChessImage, patternsize, ChessPoint, true);
#endif
    return true;
}
QVector<cv::Mat> PoseEstimation::GetPose2d2d(cv::Mat& img1, cv::Mat& img2)
{
    cv::Mat R, t;
#if !__USE_CORNER__
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> matches;
    TIMEBEGIN()
    find_feature_matches(img1, img2, keypoints_1, keypoints_2, matches);
    TIMEEND("find feature matches")
    std::cout << "PoseEstimation: find " << matches.size() << " pair key points" << std::endl;
#if __SHOW_DEBUG_IMG__
    for(auto& kp:keypoints_1)
        cv::circle(img1,kp.pt,5,cv::Scalar(10,100,10));
    for(auto& kp:keypoints_2)
        cv::circle(img2,kp.pt,5,cv::Scalar(100,10,10));
#endif

    //-- 估计两张图像间运动
    TIMEBEGIN()
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);
    TIMEEND("pose eatimation")
#else
    std::vector<cv::Point2f> points1;
    findcorners(img1, Size(7, 10),points1);
#if __SHOW_DEBUG_IMG__
    imshow("view1", img1);
#endif
    std::vector<cv::Point2f> points2;
    findcorners(img2, Size(7, 10), points2);
#if __SHOW_DEBUG_IMG__
    imshow("view2", img2);
#endif
    //Mat K = (Mat_<double>(3, 3) << 693.3888, 0, 388.3396, 0, 692.5006, 221.9705, 0, 0, 1);
    //Mat K = (Mat_<double>(3, 3) << 685.6006, 0, 372.1482, 0, 685.3158, 256.8197, 0, 0, 1);
    //Mat K = (Mat_<double>(3, 3) << 749.6125, 0, 376.6261, 0, 761.2199, 236.4443, 0, 0, 1);
    //Mat K = (Mat_<double>(3, 3) << 737.6866, 0, 391.6638, 0, 737.6866, 269.0722, 0, 0, 1);
    Mat K = (Mat_<double>(3, 3) << 3769.9, 0, 400.7, 0, 3885.4, 605.4, 0, 0, 1);
    Mat essential_matrix = findEssentialMat(points1, points2, K,RANSAC,0.999,10);
    qDebug()<<"points size before: "<<points1.size();
    std::cout<<"pre: "<<points1[0]<<"\n";
    std::cout<<"now: "<<points2[0]<<"\n";
    //correctMatches(essential_matrix,points1,points2,points1,points2);
    //qDebug()<<"points size after: "<<points1.size();
    //std::cout<<"pre: "<<points1[0]<<"\n";
    //std::cout<<"now: "<<points2[0]<<"\n";
    recoverPose(essential_matrix, points1, points2, K, R, t);
    std::cout << "essential matrix is " << essential_matrix << std::endl;
    std::cout << "R is " << std::endl << R << std::endl;
    std::cout << "t is " << std::endl << t << std::endl;
#endif
    QVector<cv::Mat> res;
    res.push_back(R);
    res.push_back(t);
    return res;
}
