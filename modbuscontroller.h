#ifndef MODBUSCONTROLLER_H
#define MODBUSCONTROLLER_H

#include <QObject>
#include <QModbusClient>
#include <QThread>
#include <QVector>
#include <QVariant>

#define __SHOW_MODBUS_RES__ 0

class ZPos
{
public:
    ZPos(int addr=1);
    void Reset();
    int GetAddr();
    int GetCurrentPos();
private:
    QModbusDevice::Error error;
    int addr;
    uint currentPos;
    friend class modbusController;
};

class modbusController : public QObject
{
    Q_OBJECT
public:
    ZPos *zPhasePos[6];
    int64_t GeneralData[6]={0};
    QModbusDevice::Error GeneralError[6];
    explicit modbusController(QObject *parent = nullptr);
private:
    QThread *newThread;
    QModbusClient *modbusDevice;
signals:

public slots:
    void initModbusSlot(char* comModbus);
    void sendReadRequestSlot(int addr, int startAddr, quint16 size);
    void sendWriteRequestSlot(int addr, int startAddr, quint16 cnt, QVariant data);
private slots:
    void modbusReadReady();
};

#endif // MODBUSCONTROLLER_H
