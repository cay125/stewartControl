#include "modbuscontroller.h"
#include "controller.h"
#include <QModbusRtuSerialMaster>
#include <QModbusDataUnit>
#include <QSerialPort>
#include <QDebug>
modbusController::modbusController(QObject *parent) : QObject(parent),newThread(new QThread)
{
    for(int i=0;i<6;i++)
        zPhasePos[i]=new ZPos(i);
    this->moveToThread(newThread);
    newThread->start();
}
void modbusController::initModbusSlot(char* comModbus)
{
    modbusDevice = new QModbusRtuSerialMaster(nullptr);
    modbusDevice->setConnectionParameter(QModbusDevice::SerialPortNameParameter, comModbus);
    modbusDevice->setConnectionParameter(QModbusDevice::SerialParityParameter,QSerialPort::NoParity);
    modbusDevice->setConnectionParameter(QModbusDevice::SerialBaudRateParameter,QSerialPort::Baud19200);
    modbusDevice->setConnectionParameter(QModbusDevice::SerialDataBitsParameter,QSerialPort::Data8);
    modbusDevice->setConnectionParameter(QModbusDevice::SerialStopBitsParameter,QSerialPort::OneStop);
    modbusDevice->setTimeout(1000);
    modbusDevice->setNumberOfRetries(3);
    if(!modbusDevice->connectDevice())
        qDebug()<<"modbus connection failed";
    else
        qDebug()<<"modbus connection successful";
}
void modbusController::sendReadRequestSlot(int addr, int startAddr, quint16 size)
{
    Q_ASSERT(addr>=1 && addr<=6);
    QModbusDataUnit dataUnit(QModbusDataUnit::HoldingRegisters, startAddr, size);
    if (auto *reply = modbusDevice->sendReadRequest(dataUnit, addr))
    {
        if (!reply->isFinished())
            connect(reply, &QModbusReply::finished, this, &modbusController::modbusReadReady);
        else
            delete reply; // broadcast replies return immediately
    }
    else
    {
        qDebug() << "send read request error: "<< modbusDevice->errorString();
    }
}
void modbusController::modbusReadReady()
{
    auto reply = qobject_cast<QModbusReply *>(sender());
    if (!reply)
        return;

    if (reply->error() == QModbusDevice::NoError)
    {
        uint64_t tempPos=0;
        const QModbusDataUnit unit = reply->result();
        for (uint i = 0; i < unit.valueCount(); i++)
        {
            tempPos=(unit.value(i)<<(16*i))|tempPos;
#if __SHOW_MODBUS_RES__
            const QString entry = tr("Address: %1, Value: %2").arg(unit.startAddress() + i).arg(QString::number(unit.value(i), unit.registerType() <= QModbusDataUnit::Coils ? 10 : 16));
            qDebug()<<"modbus receive data: "<<entry;
#endif
        }
        if(unit.startAddress() == 4032) //means the returned data is z phase position.
            zPhasePos[reply->serverAddress()-1]->currentPos = static_cast<uint32_t>(tempPos);
        GeneralData[reply->serverAddress()-1] = static_cast<int64_t>(tempPos);
    }
    else if (reply->error() == QModbusDevice::ProtocolError)
    {
        QString msg = tr("Read response error: %1 (Mobus exception: 0x%2)").arg(reply->errorString()).arg(reply->rawResult().exceptionCode(), -1, 16);
        qDebug()<<"error msg: "<<msg;
    }
    else
    {
        QString msg = tr("Read response error: %1 (code: 0x%2)").arg(reply->errorString()).arg(reply->error(), -1, 16);
        qDebug()<<"error msg: "<<msg;
        exit(1);
    }
    zPhasePos[reply->serverAddress()-1]->error=reply->error();
    GeneralError[reply->serverAddress()-1]=reply->error();
    reply->deleteLater();
    const QMutexLocker locker(&m_mutex);
    m_cond.wakeOne();
}
void modbusController::sendWriteRequestSlot(int addr, int startAddr, quint16 cnt, QVariant data)
{
    QVector<quint16> inputData=data.value<QVector<quint16>>();
    QModbusDataUnit writeUnit = QModbusDataUnit(QModbusDataUnit::HoldingRegisters, startAddr, cnt);
    for (uint i = 0; i < writeUnit.valueCount(); i++)
        writeUnit.setValue(i, inputData[i]);

    if (auto *reply = modbusDevice->sendWriteRequest(writeUnit, addr))
    {
        if (!reply->isFinished())
        {
            connect(reply, &QModbusReply::finished, this, [this, reply]()
            {
                if (reply->error() == QModbusDevice::ProtocolError)
                {
                    QString msg = tr("Write response error: %1 (Mobus exception: 0x%2)").arg(reply->errorString()).arg(reply->rawResult().exceptionCode(), -1, 16);
                    qDebug() << "error msg" << msg;
                }
                else if (reply->error() != QModbusDevice::NoError)
                {
                    QString msg =  tr("Write response error: %1 (code: 0x%2)").arg(reply->errorString()).arg(reply->error(), -1, 16);
                    qDebug() << "error msg" << msg;
                }
                GeneralError[reply->serverAddress()-1]=reply->error();
                reply->deleteLater();
                const QMutexLocker locker(&m_mutex);
                m_cond.wakeOne();
            });
        }
        else
        {
            // broadcast replies return immediately
            reply->deleteLater();
        }
    }
    else
    {
        qDebug() << "send write request error: " << modbusDevice->errorString();
    }
}

ZPos::ZPos(int addr)
{
    this->addr=addr;
    Reset();
}
void ZPos::Reset()
{
    currentPos=0;
    error=QModbusDevice::NoError;
}
int ZPos::GetAddr(){return addr;}
int ZPos::GetCurrentPos()
{
    if(error==QModbusDevice::NoError)
        return static_cast<int>(currentPos);
    else
        return -1;
}
