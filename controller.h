#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "modbuscontroller.h"
#include "serialport.h"
#include "inversekinematic.h"
#include "pidcontroller.h"
#include "zerodetector.h"
#include "minsquresolver.h"
#include "imgprocessthread.h"
#include <QString>
#include <QTimer>
#include <QModbusClient>
#include <QModbusDataUnit>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QTcpServer>
#include <QTcpSocket>
#include <map>

#define FEEDBACK_FROM_MORTOR 0
#define HARDLIMITS 1
#define SHOW_CYCLE_TIME 1
#define MULTI_VEL 1
#define MULTI_POS 1
#define ZUPT 1


extern QMutex m_mutex;
extern QMutex imu_mutex;
extern QMutex frame_mutex;
extern QWaitCondition m_cond;
extern QWaitCondition frame_cond;
extern QByteArray globalByteArray;
extern QByteArray globalGyroArray;
extern QByteArray globalTopAngleArray;
extern QByteArray globalAccArray;
extern QByteArray globalTimeArray;
extern double globalVelZ,globalDisZ,globalStaticGravity;
extern cv::Mat FrameRotation,FrameTranslation;
extern ZeroDetector* globaldetector;
extern bool globalStableStatus;

enum Status{Simple,GUIControl,IMUControl};
enum MotionMode{JOG,TRAP,BOTH};
class ImuTime
{
public:
    int year=0;
    int month=0;
    int day=0;
    int hour=0;
    int minite=0;
    int seconds=0;
    int ms=0;
    static int timeStampUntilNow;
    static int GetDuration(ImuTime new_stamp,ImuTime old_stamp);
};

class Controller : public QObject
{
    Q_OBJECT
public:
    Controller(char* com_card, char* com_modbus, char* com_imu, QObject* parent=nullptr);
    void simpleOperationMode();
    void GuiControlMode();
    void IMUControlMode();
    void StartFrameProcess(int method);
    void simpleTrajectory(double diff_x,double diff_y,double diff_z);
    void resetAll();
    void resetAll(int start, int end);
    void setDriverEnable(int addr, bool value);
    Status currentStatus=GUIControl;
private:
    const double processNoise_Q=0.1;
    const double measureNoise_R=10;
    ImgProcessThread *processThread=nullptr;
    minsqureSolver *disSolver=nullptr,*velSolver=nullptr;
    ZeroDetector *detector;
    QTimer *timer;
    modbusController *RS485;
    QTcpServer *tcpServer;
    QTcpSocket *imuClient=nullptr;
    QTcpSocket *visionClient=nullptr;
    QList<QTcpSocket *>tcpClients;
    inverseKinematic *kinematicModule;
    SerialPort* uart;
    PIDController* pid_regulator[6];
    std::map<int,int> legIndex2Motion;
    std::map<int,int> motion2LegIndex;
    double normalZ=353.57+20;
    double currentPos[6]={0};
    QVector<double> refPos;
    QVector<double> refSpeed;
    ImuTime timeStamp;
    float zTransVision=0;
    double gra=9.8,staticAcc=9.8;
    double orientAccZ=0,orientAccZ_AfterFilter=0;
    double disZ_AfterMinSqure=0,velZ_AfterMinSqure=0;
    double angleX=0,angleY=0,angleZ=0,gyroX=0,gyroY=0,gyroZ=0,accX=0,accY=0,accZ=0,velX=0,velY=0,velZ=0,disX=0,disY=0,disZ=0;
    double currentX=0,currentY=0,currentZ=0,currentRx=0,currentRy=0,currentRz=0;
    double simpleKalman(double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
    void correctionGra();
    void GetZphasePos(int addr);
    void GetCurrentPos(int addr);
    void reset(int addr);
    void MoveLegs(QVector<double>& pos,MotionMode mode=MotionMode::TRAP);
    void MoveLeg(int addr, qint64 pos, bool flag=false);
    void MoveLegInJog(int addr,qint64 pos,double currentPos);
    void MoveMultiLegInJog(short addrStart,short addrEnd,QVector<qint64>& poses,double* currentPoses);
    void MoveMultiLegInTrap(short addrStart,short addrEnd,QVector<qint64>& poses);
    void MoveMultiLegInBoth(short addrStart,short addrEnd,QVector<qint64>& poses,double* currentPoses);
    void updateAxis(int start, int end);
    void updatePosition(QByteArray);
    void sendData();
    void analyseData(bool calculateDis=true);
    void initMode(double acc=1,double dec=1,double speed=1,MotionMode mode=MotionMode::TRAP);
private slots:
    void tcpReadDataSlot();
signals:
    void sendReadRequestSignal(int addr, int startAddr, quint16 size);
    void sendWriteRequestSignal(int addr, int startAddr, int cnt, QVariant data);
    void initModbusSignal(char* comModbus);
    void startUartSignal(QString portname,int boudrate,int parity);
    void sendSocketSignal(QTcpSocket* client);
};
#endif // CONTROLLER_H
