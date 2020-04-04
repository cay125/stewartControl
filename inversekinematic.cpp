#include "inversekinematic.h"
#include <QtMath>
inverseKinematic::inverseKinematic(stewartPara* _para)
{
    para=_para;
    initBase();
}
void inverseKinematic::initBase()
{
    for(int i=0;i<6;i++)
    {
        double theta=M_PI/3*i+para->hexSkew*((i%2)==0?1:-1);
        //calculate Base frame
        B(0,i)=para->radiusB*qCos(theta);
        B(1,i)=para->radiusB*qSin(theta);
        B(2,i)=para->uniJointHeight;
        int j = (i + 5) % 6 ;
        P(0,j)=para->radiusP*qCos(theta);
        P(1,j)=para->radiusP*qSin(theta);
        P(2,j)=-para->uniJointHeight;
    }
    Eigen::Matrix3d R=rotation3D(-M_PI/3, Eigen::Vector3d(0,0,1));
    P=R*P;
}
Eigen::Matrix3d inverseKinematic::rotation3D(double angle, Eigen::Vector3d axis)
{
    Eigen::Matrix3d inverseMat=Eigen::Matrix3d::Zero(3,3);
    inverseMat<<      0 ,-axis(2), axis(1),
                 axis(2),      0 ,-axis(0),
                -axis(1), axis(0),      0 ;
    Eigen::Matrix3d outputR=Eigen::Matrix3d::Identity(3,3)+(1-qCos(angle))*inverseMat*inverseMat+qSin(angle)*inverseMat;
    return outputR;
}
QVector<double> inverseKinematic::GetLength(double x,double y,double z,double rotateX,double rotateY, double rotateZ)
{
    rotateX=rotateX*M_PI/180;
    rotateY=rotateY*M_PI/180;
    rotateZ=rotateZ*M_PI/180;
    Eigen::Matrix3d RX,RY,RZ;
    Eigen::MatrixXd d= Eigen::MatrixXd::Zero(3,6);
    for(int i=0;i<6;i++)
    {
        d(0,i)=x;
        d(1,i)=y;
        d(2,i)=z;
    }
    RX<<1,0,0,
        0,qCos(rotateX),-qSin(rotateX),
        0,qSin(rotateX), qCos(rotateX);
    RY<< qCos(rotateY),0,qSin(rotateY),
                     0,1,0,
        -qSin(rotateY),0,qCos(rotateY);
    RZ<<qCos(rotateZ),-qSin(rotateZ),0,
        qSin(rotateZ), qCos(rotateZ),0,
        0,0,1;
    Eigen::Matrix3d R=RZ*RX*RY;
    Eigen::MatrixXd len_arrow(3,6);
    len_arrow=R*P+d-B;
    QVector<double> len_norm;
    for(int i=0;i<6;i++)
        len_norm.append(len_arrow.col(i).norm());
    return len_norm;
}
stewartPara::stewartPara(double _radiusB, double _radiusP, double _hexSkew, double _nom_l, double _uniH)
{
    radiusB=_radiusB;
    radiusP=_radiusP;
    hexSkew=_hexSkew;
    nomialLength=_nom_l;
    uniJointHeight=_uniH;
}
stewartPara::stewartPara(double _radius, double _hexLen, double _nom_l, double uniH)
{
    radiusB=radiusP=_radius;
    nomialLength=_nom_l;
    uniJointHeight=uniH;
    hexSkew=M_PI/6 - M_PI/2+qAcos(_hexLen/2/_radius);
}
qint64 inverseKinematic::Len2Pulse(double len)
{
    return static_cast<int>((len-para->nomialLength))*(1000/10);
}
