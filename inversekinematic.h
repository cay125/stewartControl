#ifndef INVERSEKINEMATIC_H
#define INVERSEKINEMATIC_H
#include <Eigen/Dense>
#include <QVector>

class stewartPara
{
public:
    double radiusB,radiusP,hexSkew;
    double nomialLength;
    double uniJointHeight;
    stewartPara(double _radiusB, double _radiusP, double _hexSkew, double _nom_l, double uniH);
    stewartPara(double _radius, double _hexLen, double _nom_l, double uniH);
};

class inverseKinematic
{
public:
    explicit inverseKinematic(stewartPara* _para);
    QVector<double> GetLength(double x,double y,double z,double rotateX,double rotateY, double rotateZ);
    QVector<double> GetSpeed(double gyrox,double gyroy,double gyroz);
    qint64 Len2Pulse(double);
    qint64 Speed2Pulse(double);
    static Eigen::Matrix3d GetOrientDir(double rotateX,double rotateY,double rotateZ);
    Eigen::Matrix3d rotation3D(double angle, Eigen::Vector3d axis);
    stewartPara *para;
private:
    Eigen::MatrixXd B=Eigen::MatrixXd::Zero(3,6);
    Eigen::MatrixXd P=Eigen::MatrixXd::Zero(3,6);

    Eigen::MatrixXd len_arrow;
    Eigen::Matrix3d R;
    double leg_norm[6]={0};
    void initBase();
};

#endif // INVERSEKINEMATIC_H
