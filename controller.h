#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "modbuscontroller.h"
#include "serialport.h"
#include "inversekinematic.h"
#include "pidcontroller.h"
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

extern QMutex m_mutex;
extern QWaitCondition m_cond;

enum Status{Simple,GUIControl,IMUControl};
enum MotionMode{JOG,TRAP};

class Controller : public QObject
{
    Q_OBJECT
public:
    Controller(char* com_card, char* com_modbus, char* com_imu, QObject* parent=nullptr);
    void simpleOperationMode();
    void GuiControlMode();
    void IMUControlMode();
    [[noreturn]] void simpleTrajectory(int mode=0);
    void resetAll();
    void resetAll(int start, int end);
    void setDriverEnable(int addr, bool value);
    void GetZphasePos(int addr);
    Status currentStatus=GUIControl;
private:
    QTimer *timer;
    modbusController *RS485;
    QTcpServer *tcpServer;
    QTcpSocket *imuClient=nullptr;
    QList<QTcpSocket *>tcpClients;
    inverseKinematic *kinematicModule;
    SerialPort* uart;
    PIDController* pid_regulator[6];
    std::map<int,int> legIndex2Motion;
    std::map<int,int> motion2LegIndex;
    double normalZ=353.57+20;
    double currentPos[6]={0};
    QVector<double> refPos;
    double angleX=0,angleY=0,angleZ=0,gyroX=0,gyroY=0,gyroZ=0;
    double currentX=0,currentY=0,currentZ=0,currentRx=0,currentRy=0,currentRz=0;
    //void GetZphasePos(int addr);
    void GetCurrentPos(int addr);
    void reset(int addr);
    void MoveLegs(QVector<double>& pos,MotionMode mode=MotionMode::TRAP);
    void MoveLeg(int addr, qint64 pos, bool flag=false);
    void MoveLegInJog(int addr,qint64 pos,double currentPos);
    void updateAxis(int start, int end);
    void updatePosition(QByteArray);
    void initMode(double acc=1,double dec=1,double speed=1,MotionMode mode=MotionMode::TRAP);
private slots:
    void timerSlot();
    void tcpReadDataSlot();
signals:
    void sendReadRequestSignal(int addr, int startAddr, quint16 size);
    void sendWriteRequestSignal(int addr, int startAddr, int cnt, QVariant data);
    void initModbusSignal(char* comModbus);
    void startUartSignal(QString portname,int boudrate,int parity);
    void sendSocketSignal(QTcpSocket* client);
};
#endif // CONTROLLER_H
