#include <QDebug>
#include <QModbusRtuSerialMaster>
#include "controller.h"
#include "GAS_N.h"
Controller::Controller(char* com_card,char* com_modbus):uart(new SerialPort),timer(new QTimer)
{
    if(GA_Open(1,com_card))
        qDebug()<<"open card failed";
    else
        qDebug()<<"open card successful";
    modbusDevice=new QModbusRtuSerialMaster(this);
    modbusDevice->setConnectionParameter(QModbusDevice::SerialPortNameParameter, com_modbus);
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
void Controller::connectedSlot()
{

}
void Controller::simpleOperation()
{
    short axis=6;
    if(GA_PrfTrap(axis))
    {
        qDebug()<<"set trap model failed";
        return;
    }
    TTrapPrm trapPrm;
    if(GA_GetTrapPrm(axis,&trapPrm))
    {
        qDebug()<<"set trap model failed";
        return;
    }
    trapPrm.acc=0.5;
    trapPrm.dec=0.5;
    trapPrm.velStart=0;
    trapPrm.smoothTime=0;
    if(GA_SetTrapPrm(axis,&trapPrm))
    {
        qDebug()<<"set trap params failed";
        return;
    }
    if(GA_SetVel(axis,1))
    {
        qDebug()<<"set speed failed";
        return;
    }

    connect(timer,&QTimer::timeout,this,&Controller::timerSlot);
    timer->setInterval(50);
    timer->start();
}
void Controller::timerSlot()
{
    short axis=6;
    long status=0;
    if(GA_GetSts(axis,&status))
    {
        qDebug()<<"read status failed";
        GA_Stop(0x0001<<(axis-1),0x0001<<(axis-1));
        return;
    }
    if(status & AXIS_STATUS_RUNNING)
    {
        qDebug()<<"running";
        return;
    }
    double pos;static int cnt=0;
    if(GA_GetAxisPrfPos(axis,&pos))
    {
        qDebug()<<"read pos failed";
        GA_Stop(0x0001<<(axis-1),0x0001<<(axis-1));
        return;
    }
    if(cnt%2)
    {
        if(GA_SetPos(axis,pos+10))
        {
            qDebug()<<"set pos failed";
            GA_Stop(0x0001<<(axis-1),0x0001<<(axis-1));
            return;
        }
        if(GA_Update(0x0001<<(axis-1)))
        {
            qDebug()<<"update failed";
            GA_Stop(0x0001<<(axis-1),0x0001<<(axis-1));
            return;
        }
    }
    else
    {
        if(GA_SetPos(axis,pos-10))
        {
            qDebug()<<"set pos failed";
            GA_Stop(0x0001<<(axis-1),0x0001<<(axis-1));
            return;
        }
        if(GA_Update(0x0001<<(axis-1)))
        {
            qDebug()<<"update failed";
            GA_Stop(0x0001<<(axis-1),0x0001<<(axis-1));
            return;
        }
    }
    cnt++;
    if(cnt>1)
        timer->stop();
}
void Controller::reset()
{
    GetZphasePos(1);
}
void Controller::GetZphasePos(int addr)
{
    QModbusDataUnit dataUnit(QModbusDataUnit::HoldingRegisters, 4032, 2);
    if (auto *reply = modbusDevice->sendReadRequest(dataUnit, addr))
    {
        if (!reply->isFinished())
            connect(reply, &QModbusReply::finished, this, &Controller::modbusReadReady);
        else
            delete reply; // broadcast replies return immediately
    }
    else
    {
        qDebug() << modbusDevice->errorString();
    }
}
void Controller::modbusReadReady()
{
    auto reply = qobject_cast<QModbusReply *>(sender());
    if (!reply)
        return;

    if (reply->error() == QModbusDevice::NoError)
    {
        const QModbusDataUnit unit = reply->result();
        for (uint i = 0; i < unit.valueCount(); i++)
        {
            const QString entry = tr("Address: %1, Value: %2").arg(unit.startAddress() + i).arg(QString::number(unit.value(i), unit.registerType() <= QModbusDataUnit::Coils ? 10 : 16));
            qDebug()<<"modbus receive data: "<<entry;
        }
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
    }

    reply->deleteLater();
}
