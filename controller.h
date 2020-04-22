#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "modbuscontroller.h"
#include "inversekinematic.h"
#include <QString>
#include <QTimer>
#include <QModbusClient>
#include <QModbusDataUnit>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QTcpServer>
#include <QTcpSocket>

extern QMutex m_mutex;
extern QWaitCondition m_cond;

enum Status{Simple,GUIControl,InetialControl};

class Controller : public QObject
{
    Q_OBJECT
public:
    Controller(char* com_card, char* com_modbus, QObject* parent=nullptr);
    void simpleOperationMode();
    void GuiControlMode();
    void simpleTrajectory();
    void resetAll();
    void resetAll(int start, int end);
    void setDriverEnable(int addr, bool value);
    void GetZphasePos(int addr);
    Status currentStatus=GUIControl;
private:
    QTimer *timer;
    modbusController *RS485;
    QTcpServer *tcpServer;
    QList<QTcpSocket *>tcpClients;
    inverseKinematic *kinematicModule;
    double normalZ=353.57;
    double currentX=0,currentY=0,currentZ=0,currentRx=0,currentRy=0,currentRz=0;
    //void GetZphasePos(int addr);
    void GetCurrentPos(int addr);
    void reset(int addr);
    void MoveLegs(QVector<double>& pos);
    void MoveLeg(int addr, qint64 pos, bool flag=false);
    void updateAxis(int start, int end);
    void initMode(double acc=1,double dec=1,double speed=1);
private slots:
    void timerSlot();
    void tcpReadDataSlot();
signals:
    void sendReadRequestSignal(int addr, int startAddr, quint16 size);
    void sendWriteRequestSignal(int addr, int startAddr, int cnt, QVariant data);
    void initModbusSignal(char* comModbus);
};
#endif // CONTROLLER_H
