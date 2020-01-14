#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "serialport.h"
#include <QString>
#include <QTimer>
#include <QModbusClient>
#include <QModbusDataUnit>

class Controller:public QObject
{
    Q_OBJECT
public:
    Controller(char* com_card, char* com_modbus);
    void simpleOperation();
    void reset();
private:
    SerialPort *uart;
    QTimer *timer;
    QModbusClient *modbusDevice;
    void GetZphasePos(int addr);
private slots:
    void connectedSlot();
    void timerSlot();
    void modbusReadReady();
signals:
    void portStartSignal(QString,int,int);
    void portCloseSignal();
};

#endif // CONTROLLER_H
