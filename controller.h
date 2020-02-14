#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "modbuscontroller.h"
#include <QString>
#include <QTimer>
#include <QModbusClient>
#include <QModbusDataUnit>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>

extern QMutex m_mutex;
extern QWaitCondition m_cond;

class Controller : public QObject
{
    Q_OBJECT
public:
    Controller(char* com_card, char* com_modbus, QObject* parent=nullptr);
    void simpleOperation();
    void resetAll();
    void setDriverEnable(int addr, bool value);
private:
    QTimer *timer;
    QModbusClient *modbusDevice;
    modbusController *RS485;
    void GetZphasePos(int addr);
    void reset(int addr);
private slots:
    void timerSlot();
signals:
    void sendReadRequestSignal(int addr, int startAddr);
    void sendWriteRequestSignal(int addr, int startAddr, int cnt, QVariant data);
    void initModbusSignal(char* comModbus);
};
#endif // CONTROLLER_H
