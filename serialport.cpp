#include "serialport.h"
#include <QSerialPortInfo>
#include <utility>
SerialPort::SerialPort(QObject *parent) : QObject(parent)
{
    worker_thread = new QThread();
    port = new QSerialPort();
    this->moveToThread(worker_thread);
    port->moveToThread(worker_thread);
    worker_thread->start();
    buildTableCRC16();
    //qDebug()<<"main thread id: "<<QThread::currentThreadId();
}

SerialPort::~SerialPort()
{
    port->close();
    port->deleteLater();
    worker_thread->quit();
    worker_thread->wait();
    worker_thread->deleteLater();
}

void SerialPort::init_port()
{
    port->setDataBits(QSerialPort::Data8);             //数据位
    port->setStopBits(QSerialPort::OneStop);           //停止位
    port->setParity(QSerialPort::NoParity);            //奇偶校验
    port->setFlowControl(QSerialPort::NoFlowControl);  //流控制
}
void SerialPort::start_port(QString portname, int baudrate, int parity)
{
    init_port();
    port->setPortName(portname);
    port->setBaudRate(baudrate);
    if(parity==0)
        port->setParity(QSerialPort::OddParity);
    else if(parity==1)
        port->setParity(QSerialPort::EvenParity);
    else
        port->setParity(QSerialPort::NoParity);
    if (port->open(QIODevice::ReadWrite))
    {
        qDebug() << "open uart successful";
        connect(port, SIGNAL(readyRead()), this, SLOT(handle_data())); //Qt::DirectConnection
        emit connected();
    }
    else
    {
        qDebug() << "open uart failed";
    }
}
void SerialPort::stop_port()
{
    port->close();
    qDebug() << "Port have been closed";
}
uint32_t SerialPort::crc_check(uint8_t* data, uint32_t length)
{
    unsigned short crc_reg = 0xFFFF;
    while (length--)
        crc_reg = (crc_reg >> 8) ^ crc16_table[(crc_reg ^ *data++) & 0xff];
    return (uint32_t)(~crc_reg) & 0x0000FFFF;
}
void SerialPort::buildTableCRC16()
{
    uint16_t i16 , j16;
    uint16_t data16;
    uint16_t accum16;
    for(i16=0;i16<256;i16++)
    {
      data16 = (uint16_t)(i16<<8);
        accum16 = 0;
        for(j16=0;j16<8;j16++)
        {
          if((data16^accum16)&0x8000)
            {
              accum16 = (accum16<<1)^0x1021;
            }
            else
            {
              accum16<<=1;
            }
            data16 <<= 1;
        }
        tableCRC16[i16] = accum16;
    }
}
uint16_t SerialPort::calcCRC16()
{
    uint16_t crc16 = 0;
    for(int i=0;i<35;i++)
    {
        if(i==0)
            crc16 = (crc16<<8)^tableCRC16[(crc16>>8)^(0x23)];
        else if(i==1)
            crc16 = (crc16<<8)^tableCRC16[(crc16>>8)^(0x01)];
        else
            crc16 = (crc16<<8)^tableCRC16[(crc16>>8)^((uint8_t)pointData.at(i-2))];
    }
    return crc16;
}

void SerialPort::handle_data()
{
    static std::pair<int,recieveType> state(0, recieveType::angle);
    auto data=port->readAll();
    if(displayClient!=nullptr)
        displayClient->write(data);
    //qDebug()<<"worker thread id: "<<QThread::currentThreadId();
    for(int i=0;i<data.size();i++)
    {
        uchar num=static_cast<uchar>(data.at(i));
        if(state.first==0 && num==0x55)
        {
            pointData.clear();
            state.first=1;
        }
        else if(state.first==1)
        {
            if(num!=0x52 && num!=0x53)
            {
                state.first=0;
            }
            else if(num==0x52)
            {
                state.first++;
                pointData.append(recieveType::gyro);
                state.second=recieveType::gyro;
            }
            else if(num==0x53)
            {
                state.first++;
                pointData.append(recieveType::angle);
                state.second=recieveType::angle;
            }
        }
        else if(state.first>1)
        {
            state.first++;
            pointData.append(data.at(i));
            if(state.first>=8)
            {
                state.first=0;
                emit receive_data(pointData);
            }
        }
    }
}
void SerialPort::sendSocketSlot(QTcpSocket* client)
{
    displayClient=client;
}

void SerialPort::write_data()
{
    qDebug() << "write_id is:" << QThread::currentThreadId();
    port->write("data", 4);
}
