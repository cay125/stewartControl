#include <iostream>
#include "controller.h"
#include "timemeasure.h"
#include "GAS_N.h"
QMutex m_mutex;
QMutex imu_mutex;
QMutex frame_mutex;
QWaitCondition m_cond;
QWaitCondition frame_cond;
QByteArray globalByteArray;
QByteArray globalGyroArray;
QByteArray globalTopAngleArray;
QByteArray globalAccArray;
QByteArray globalTimeArray;
double globalVelZ=0,globalDisZ=0,globalStaticGravity=0;
cv::Mat FrameRotation(3,3,CV_64F),FrameTranslation(3,1,CV_64F);
int ImuTime::timeStampUntilNow=0;
ZeroDetector* globaldetector=nullptr;
bool globalStableStatus=true;
Controller::Controller(char* com_card, char* com_modbus, char* com_imu, QObject* parent):QObject(parent),timer(new QTimer),RS485(new modbusController),tcpServer(new QTcpServer(this)),uart(new SerialPort),detector(new ZeroDetector)
{
    //stewartPara *para=new stewartPara(170, 80, 290, 47);
    stewartPara *para=new stewartPara(170, 80, 280, 47);
    kinematicModule=new inverseKinematic(para);
    if(GA_Open(0,com_card))
        qDebug()<<"open card failed";
    else
        qDebug()<<"open card successful";
    if(GA_Reset())
        qDebug()<<"reset card failed";
    else
        qDebug()<<"reset card successful";
    connect(this,&Controller::startUartSignal,uart,&SerialPort::start_port);
    connect(uart,&SerialPort::receive_data,this,&Controller::updatePosition);
    connect(this,&Controller::sendReadRequestSignal,RS485,&modbusController::sendReadRequestSlot);
    connect(this,&Controller::sendWriteRequestSignal,RS485,&modbusController::sendWriteRequestSlot);
    connect(this,&Controller::initModbusSignal,RS485,&modbusController::initModbusSlot);
    connect(this,&Controller::sendSocketSignal,uart,&SerialPort::sendSocketSlot);
    emit startUartSignal(QString(com_imu),115200,2);
    emit initModbusSignal(com_modbus);
    //for(int i=1;i<=6;i++)
    //    setDriverEnable(i,true);
    for(int i=0;i<6;i++)
        pid_regulator[i]=new PIDController(0.015,0.001,0.02,50,10,10,40,-40);
    tcpServer->listen(QHostAddress::Any,8088);
    connect(tcpServer,&QTcpServer::newConnection,this,[this]()
    {
        QTcpSocket* tcpClient=tcpServer->nextPendingConnection();
        QString ip = tcpClient->peerAddress().toString();
        quint16 port = tcpClient->peerPort();
        qDebug()<<"one connection established from ip: "<<ip<<" port: "<<port;
        this->tcpClients << tcpClient;
        tcpClient->setParent(this);
        //connect(tcpClient,&QTcpSocket::readyRead,this,&Controller::tcpReadDataSlot);
        if(port == 9001)
        {
            visionClient=tcpClient;
            qDebug()<<"vision client connected";
        }
        else if(port == 9002)
        {
            imuClient=tcpClient;
            qDebug()<<"imu client connected";
        }
        connect(tcpClient,&QTcpSocket::disconnected,this,[this]()
        {
            QTcpSocket *pClient = qobject_cast<QTcpSocket *>(sender());
            QString ip = pClient->peerAddress().toString();
            quint16 port = pClient->peerPort();
            qDebug()<<"one connection closed from ip: "<<ip<<" port: "<<port;
            if (pClient)
            {
                tcpClients.removeAll(pClient);
                if(imuClient==pClient)
                {
                    imuClient=nullptr;
                    qDebug()<<"imu client disconnected";
                }
                if(visionClient==pClient)
                {
                    visionClient=nullptr;
                    qDebug()<<"vision client disconnected";
                }
                pClient->deleteLater();
            }
        });
    });
    legIndex2Motion[0]=1;
    legIndex2Motion[1]=6;
    legIndex2Motion[2]=5;
    legIndex2Motion[3]=4;
    legIndex2Motion[4]=3;
    legIndex2Motion[5]=2;
    for(auto it=legIndex2Motion.begin();it!=legIndex2Motion.end();it++)
        motion2LegIndex[it->second]=it->first;
    refPos.resize(6);
#if HARDLIMITS
    int res=0;
    for(short i=1;i<=6;i++)
    {
        res+=GA_LmtsOn(i,-1);
        res+=GA_SetHardLimP(i,1,0,i-1);
        res+=GA_SetHardLimN(i,1,0,i+7);
        if(res)
            qDebug()<<"set hard limits failed!!";
    }
#endif
}

void Controller::simpleOperationMode()
{
    currentStatus=Status::Simple;
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

    connect(timer,&QTimer::timeout,this,[this]()
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
    });
    timer->setInterval(50);
    timer->start();
}

void Controller::reset(int addr)
{
    QVariant data;
    QVector<quint16> dd;
    dd.append(1);dd.append(0);
    data.setValue(dd);
    m_mutex.lock();
    emit sendWriteRequestSignal(addr,2030,2,data);
    m_cond.wait(&m_mutex);
    m_mutex.unlock();
}
void Controller::resetAll(int start, int end)
{
    for(int i=start;i<=end;i++)
        reset(i);
    QVector<int> t(6,0);
    int cntZros=0;
    while(true)
    {
        for(int i=start;i<=end;i++)
            GetZphasePos(i);
        std::cout<<"Current Pos: ";
        for(int i=start-1;i<end;i++)
        {
            std::cout<<"|addr"<<i+1<<" "<<RS485->zPhasePos[i]->GetCurrentPos()<<"| ";
            if(abs(RS485->zPhasePos[i]->GetCurrentPos())<=50 && t[i]==0)
            {
                cntZros++;
                t[i]=1;
            }
        }
        std::cout<<"\n";
        if(cntZros==(end-start+1))
            break;
        //QThread::msleep(50);
    }
    qDebug()<<"Mortor Reset Finished";
}
void Controller::resetAll()
{
    for(int i=1;i<=6;i++)
    {
        reset(i);
        GA_ZeroPos(i,1);
        GA_ClrSts(i,1);
    }
    QVector<int> t(6,0);
    int cntZros=0;
    while(true)
    {
        for(int i=1;i<=6;i++)
            GetZphasePos(i);
        std::cout<<"Current Pos: ";
        for(int i=0;i<6;i++)
        {
            std::cout<<"|addr"<<i+1<<" "<<RS485->zPhasePos[i]->GetCurrentPos()<<"| ";
            if((abs(RS485->zPhasePos[i]->GetCurrentPos())<=50 || abs(10000-RS485->zPhasePos[i]->GetCurrentPos())<=50) && t[i]==0)
            {
                cntZros++;
                t[i]=1;
            }
        }
        std::cout<<"\n";
        if(cntZros==6)
            break;
        //QThread::msleep(50);
    }
    qDebug()<<"Mortor Reset Finished";
}

void Controller::GetZphasePos(int addr)
{
    m_mutex.lock();
    emit sendReadRequestSignal(addr,4032,2);
    m_cond.wait(&m_mutex);
    m_mutex.unlock();
}
void Controller::setDriverEnable(int addr, bool value)
{
    QVariant data;
    QVector<quint16> dd;
    dd.append(value==true?1:0);dd.append(0);
    data.setValue(dd);
    m_mutex.lock();
    emit sendWriteRequestSignal(addr,1008,2,data);
    m_cond.wait(&m_mutex);
    m_mutex.unlock();
}
void Controller::GetCurrentPos(int addr)
{
    m_mutex.lock();
    emit sendReadRequestSignal(addr,4008,4);
    m_cond.wait(&m_mutex);
    m_mutex.unlock();
}
void Controller::MoveLegs(QVector<double>& pos,MotionMode mode)
{
#if !MULTI_POS || !MULTI_VEL
    if(mode==MotionMode::Both)
    {
        qDebug()<<"Error: Set wrong motion mode";
        exit(1);
    }
#endif
    TIMEBEGIN()
    if(mode==MotionMode::JOG || mode==MotionMode::Both)
        GA_GetPrfPos(1,currentPos,6);
    TIMEEND("Get Pos:")
    TIMEBEGIN()
#if MULTI_VEL || MULTI_POS
    QVector<qint64> poses(6);
#endif
    for(int i=1;i<=6;i++)
    {
        if(mode==MotionMode::TRAP)
        {
#if !MULTI_POS
            MoveLeg(legIndex2Motion[i-1], kinematicModule->Len2Pulse(pos[i-1]));
#else
            poses[legIndex2Motion[i-1]-1]=-kinematicModule->Len2Pulse(pos[i-1]);
#endif
        }
        else if(mode==MotionMode::JOG)
        {
#if !MULTI_VEL
            MoveLegInJog(legIndex2Motion[i-1],-kinematicModule->Len2Pulse(pos[i-1]),currentPos[legIndex2Motion[i-1]-1]);
#else
            poses[legIndex2Motion[i-1]-1]=-kinematicModule->Len2Pulse(pos[i-1]);
#endif
        }
        else if(mode==MotionMode::Both)
        {
            poses[legIndex2Motion[i-1]-1]=-kinematicModule->Len2Pulse(pos[i-1]);
        }
        //qDebug() << "Leg: "<<i<<" length: "<< pos[i-1];
    }
#if MULTI_VEL
    if(mode==MotionMode::JOG)
        MoveMultiLegInJog(1,6,poses,currentPos);
#endif
#if MULTI_POS
    if(mode==MotionMode::TRAP)
        MoveMultiLegInTrap(1,6,poses);
#endif
    if(mode==MotionMode::Both)
        MoveMultiLegInBoth(1,6,poses,currentPos);
    TIMEEND("Set Speed:")
    TIMEBEGIN()
    updateAxis(1,6);
    TIMEEND("Update Axis:")
}
void Controller::MoveMultiLegInBoth(short addrStart, short addrEnd, QVector<qint64> &poses, double *currentPoses)
{
    MoveMultiLegInJog(addrStart,addrEnd,poses,currentPoses);
    long refPoses[6]={0};
    for(int addr=addrStart;addr<=addrEnd;addr++)
        refPoses[addr-1]=static_cast<long>(poses[addr-1]);
    GA_SetMultiPos(addrStart,refPoses+addrStart-1,addrEnd-addrStart+1);
}
void Controller::MoveMultiLegInTrap(short addrStart, short addrEnd, QVector<qint64> &poses)
{
    long refPoses[6]={0};
    for(int addr=addrStart;addr<=addrEnd;addr++)
        refPoses[addr-1]=static_cast<long>(poses[addr-1]);
    GA_SetMultiPos(addrStart,refPoses+addrStart-1,addrEnd-addrStart+1);
}
void Controller::MoveMultiLegInJog(short addrStart, short addrEnd, QVector<qint64>& poses, double *currentPoses)
{
    double refVels[6]={0};
    for(short addr=addrStart;addr<=addrEnd;addr++)
    {
        pid_regulator[addr-1]->setRef(poses[addr-1]);
        pid_regulator[addr-1]->setFeedBack(currentPoses[addr-1]);
        pid_regulator[addr-1]->calculate();
        refVels[addr-1]=pid_regulator[addr-1]->GetOutput()+refSpeed[motion2LegIndex[addr]]*0.23;
        if(refVels[addr-1]>35)
            refVels[addr-1]=35;
        else if(refVels[addr-1]<-35)
            refVels[addr-1]=-35;
    }
    GA_SetMultiVel(addrStart,refVels+addrStart-1,addrEnd-addrStart+1);
}
void Controller::MoveLegInJog(int addr, qint64 pos,double _currentPos)
{
    pid_regulator[addr-1]->setRef(pos);
    pid_regulator[addr-1]->setFeedBack(_currentPos);
    pid_regulator[addr-1]->calculate();
    qDebug()<<"fdb:"<<_currentPos<<" ref:"<<pos<<" output:"<<pid_regulator[addr-1]->GetOutput();
    double t_vel=pid_regulator[addr-1]->GetOutput()+refSpeed[motion2LegIndex[addr]]*0.23;
    if(t_vel>35)
        t_vel=35;
    else if(t_vel<-35)
        t_vel=-35;
    GA_SetVel(addr,t_vel);
}
void Controller::MoveLeg(int addr, qint64 pos, bool flag)
{
#if FEEDBACK_FROM_MORTOR
    GetCurrentPos(addr);
    qint64 relatedPos=pos-RS485->GeneralData[addr-1];
    qDebug()<<"pos: "<<pos<<" current pos: "<<RS485->GeneralData[addr-1]<<"related pos: "<<relatedPos;
    double currentPos;
    GA_GetAxisPrfPos(static_cast<short>(addr),&currentPos);
    qDebug()<<"read from card: "<<currentPos;
    GA_SetPos(addr, currentPos-relatedPos);
#else
    GA_SetPos(addr, -pos);
#endif
    if(flag)
    {
        if(GA_Update(0x0001<<(addr-1)))
        {
            qDebug()<<"update failed when move leg: "<<addr;
            GA_Stop(0x0001<<(addr-1),0x0001<<(addr-1));
        }
    }
}
void Controller::updateAxis(int start, int end)
{
    uint8_t val=0;
    for(int i=start;i<=end;i++)
        val=val|(0x0001<<(i-1));
    if(GA_Update(val))
        qDebug()<<"update failed when move legs";
}
void Controller::initMode(double acc,double dec,double speed,MotionMode mode)
{
    if(mode==MotionMode::Both)
    {
        qDebug()<<"Error: Init mode wrong";
        exit(1);
    }
    for(short axis=1;axis<=6;axis++)
    {
        if(mode==MotionMode::TRAP)
        {
            if(GA_PrfTrap(axis))
            {
                qDebug()<<"set trap model failed";
                return;
            }
            TTrapPrm trapPrm;
            if(GA_GetTrapPrm(axis,&trapPrm))
            {
                qDebug()<<"read trap model failed";
                return;
            }
            trapPrm.acc=acc;
            trapPrm.dec=dec;
            trapPrm.velStart=0;
            trapPrm.smoothTime=0;
            if(GA_SetTrapPrm(axis,&trapPrm))
            {
                qDebug()<<"set trap params failed";
                return;
            }
            if(GA_SetVel(axis,speed))
            {
                qDebug()<<"set speed failed";
                return;
            }
        }
        else if(mode==MotionMode::JOG)
        {
            if(GA_PrfJog(axis))
            {
                qDebug()<<"set JOG model failed";
                return;
            }
            JogPrm jogPrm;
            if(GA_GetJogPrm(axis,&jogPrm))
            {
                qDebug()<<"read JOG model failed";
                return;
            }
            jogPrm.dAcc=acc;
            jogPrm.dDec=dec;
            jogPrm.dSmooth=0;
            if(GA_SetJogPrm(axis,&jogPrm))
            {
                qDebug()<<"set JOG params failed";
                return;
            }
            if(GA_SetVel(axis,speed))
            {
                qDebug()<<"set speed failed";
                return;
            }
        }
    }
}
void Controller::GuiControlMode()
{
    currentStatus=Status::GUIControl;
    initMode(2,2,5,MotionMode::TRAP);
    QVector<double> len = kinematicModule->GetLength(0,0,normalZ,0,0,0);
    MoveLegs(len);
}
void Controller::IMUControlMode()
{
    currentStatus=Status::IMUControl;
    initMode(2,2,5,MotionMode::TRAP);
    analyseData(false);
    QVector<double> lens = kinematicModule->GetLength(0,0,normalZ,-angleX,-angleY,0);
    qDebug()<<"start move legs home";
    MoveLegs(lens);
    short axis=1;
    long status=0;
    while(axis<=6)
    {
        if(GA_GetSts(axis,&status))
        {
            qDebug()<<"read status failed";
            GA_Stop(0x0001<<(axis-1),0x0001<<(axis-1));
        }
        if(status & AXIS_STATUS_RUNNING)
            QThread::msleep(10);
        else
            axis++;
    }
    qDebug()<<"move home finished";
//    qDebug()<<"wait for platform being stable..";
//    QThread::msleep(3000);
//    qDebug()<<"start correct gra";
//    correctionGra();
//    qDebug()<<"correct gra finished\nconrrection gravity is: "<<staticAcc;
//    detector->setStaticGra(staticAcc);
//    globaldetector=detector;
    initMode(3,3,10,MotionMode::TRAP);
    connect(timer,&QTimer::timeout,this,[this]()
    {
        if(visionClient)
        {
            auto array=visionClient->readAll();
            if(array.size()>0)
            {
                auto new_array=array.mid(array.length()-4);
                union {float f;uint32_t d;};
                d=static_cast<uint32_t>(static_cast<uint8_t>(new_array[3])<<24|static_cast<uint8_t>(new_array[2])<<16|static_cast<uint8_t>(new_array[1])<<8|static_cast<uint8_t>(new_array[0]));
                zTransVision=f*1000;
                qDebug()<<" ZTransVision: "<<zTransVision;
            }
        }
        TIMEBEGIN()
        analyseData();
        //qDebug()<<"angleX: "<<angleX<<" angleY: "<<angleY<<" angleZ: "<<angleZ;
        //qDebug()<<"gyroX: "<<gyroX<<" gyroY: "<<gyroY<<" gyroZ: "<<gyroZ;
        //qDebug()<<"accX: "<<accX<<" accY: "<<accY<<" accZ: "<<accZ;
        //qDebug()<<"OrientAccZ: "<<orientAccZ<<" after filter: "<<orientAccZ_AfterFilter;
        //qDebug()<<"velX: "<<velX<<" velY: "<<velY<<" velZ: "<<velZ;
        //qDebug()<<"disX: "<<disX<<" disY: "<<disY<<" disZ: "<<disZ;
#if HARDLIMITS
        //GA_ClrSts(1,6); //it looks we needn't to clear state
#endif
        //refSpeed = kinematicModule->GetSpeed(gyroX,gyroY,gyroZ);
        refSpeed = kinematicModule->GetSpeed(gyroX,gyroY,0);
        auto xyz=kinematicModule->GetXYOffset(normalZ-zTransVision*0.01, -angleX,-angleY,0);
        refPos = kinematicModule->GetLength(xyz[0],xyz[1],xyz[2],-angleX,-angleY,0);
        //refPos = kinematicModule->GetLength(0,0,normalZ-zTransVision*0.01,-angleX,-angleY,0);
        MoveLegs(refPos,MotionMode::Both);
        TIMEBEGIN()
        sendData();
        TIMEEND("Send Data:")
        TIMEEND("Total:")
    });
    timer->setInterval(1);
    timer->start();
}
void Controller::tcpReadDataSlot()
{
    QTcpSocket *pClient = qobject_cast<QTcpSocket *>(sender());
    QByteArray array = pClient->readAll();
    static int state=0;
    static QVector<double> posData;
    int dataLen=array.size();
    for(int i=0;i<dataLen;i++)
    {
        if(state==0 && static_cast<uint8_t>(array.at(i))==0xaa)
        {
            posData.clear();
            state++;
        }
        else if(state==1)
        {
            auto d=static_cast<uint8_t>(array.at(i));
            if(d!=0xaa && d!=0xbb)
                state=0;
            else if(d==0xaa)
                state++;
            else if(d==0xbb)
            {
                state=0;
                imuClient=pClient;
                qDebug()<<"data display conn established";
                //emit sendSocketSignal(pClient);
            }
        }
        else if(state>=2)
        {
            posData.append(static_cast<char>(array.at(i)));
            state++;
            if(state>=8)
            {
                state=0;
                if(currentStatus==GUIControl)
                {
                    currentX=posData[0];currentY=posData[1];currentZ=posData[2]+normalZ;currentRx=posData[3];currentRy=posData[4];currentRz=posData[5];
                    QVector<double> len = kinematicModule->GetLength(currentX,currentY,currentZ,currentRx,currentRy,currentRz);
                    MoveLegs(len);
                    qDebug()<<"**********************\nrecieve target:"<<currentX<<currentY<<currentZ-normalZ<<currentRx<<currentRy<<currentRz;
                }
            }
        }
    }
    //qDebug()<<array;
}
void Controller::simpleTrajectory(double diff_x,double diff_y,double diff_z)
{
    qDebug()<<"enter simple trajectory mode\n";
    initMode(2,2,5,MotionMode::TRAP);
    connect(timer,&QTimer::timeout,this,[this,diff_x,diff_y,diff_z]()
    {
        sendData();
        long status=0;
        for(short axis=1;axis<=6;axis++)
        {
            if(GA_GetSts(axis,&status))
            {
                qDebug()<<"read status failed";
                GA_Stop(0x0001<<(axis-1),0x0001<<(axis-1));
            }
            if(status & AXIS_STATUS_RUNNING)
                return;
        }
        qDebug()<<"motion finished!!";
        QThread::msleep(50);
        static int flag=0;
        if(flag==0)
        {
            auto lens=kinematicModule->GetLength(+diff_x,+diff_y,normalZ+diff_z,0,0,0);
            MoveLegs(lens);
        }
        else
        {
            auto lens=kinematicModule->GetLength(-diff_x,-diff_y,normalZ-diff_z,0,0,0);
            MoveLegs(lens);
        }
        flag=1-flag;
    });
    timer->setInterval(50);
    timer->start();
}
void Controller::updatePosition(QByteArray data)
{
    auto type=static_cast<recieveType>(data.at(0));
    if(type==recieveType::angle)
    {
        angleX=static_cast<int16_t>(((static_cast<uint8_t>(data.at(2))<<8)|static_cast<uint8_t>(data.at(1))))/32768.0*180.0;
        angleY=static_cast<int16_t>(((static_cast<uint8_t>(data.at(4))<<8)|static_cast<uint8_t>(data.at(3))))/32768.0*180.0;
        angleZ=static_cast<int16_t>(((static_cast<uint8_t>(data.at(6))<<8)|static_cast<uint8_t>(data.at(5))))/32768.0*180.0;
        if(imuClient!=nullptr)
        {
            for(int i=0;i<6;i++)
                data.append(static_cast<char>(pid_regulator[i]->GetOutput()));
            for(int i=0;i<6;i++)
            {
                data.append(static_cast<char>(static_cast<int16_t>(currentPos[i])>>8));
                data.append(static_cast<char>(static_cast<int16_t>(currentPos[i])&0x00ff));
            }
            for(int i=0;i<6;i++)
            {
                auto puls=kinematicModule->Len2Pulse(refPos[motion2LegIndex[i+1]]);
                data.append(static_cast<char>(static_cast<int16_t>(-puls)>>8));
                data.append(static_cast<char>(static_cast<int16_t>(-puls)&0x00ff));
            }
            imuClient->write(data);
        }
    }
    else if(type==recieveType::gyro)
    {
        gyroX=static_cast<int16_t>(((static_cast<uint8_t>(data.at(2))<<8)|static_cast<uint8_t>(data.at(1))))/32768.0*2000.0;
        gyroY=static_cast<int16_t>(((static_cast<uint8_t>(data.at(4))<<8)|static_cast<uint8_t>(data.at(3))))/32768.0*2000.0;
        gyroZ=static_cast<int16_t>(((static_cast<uint8_t>(data.at(6))<<8)|static_cast<uint8_t>(data.at(5))))/32768.0*2000.0;
    }
    //qDebug()<<angleX<<" "<<angleY<<" "<<angleZ;
}
void Controller::analyseData(bool calculateDis)
{
    imu_mutex.lock();
    auto data=globalByteArray;
    auto data2=globalGyroArray;
    auto data3=globalAccArray;
    auto data4=globalTimeArray;
    imu_mutex.unlock();
    if(data.size())
    {
        angleX=static_cast<int16_t>(((static_cast<uint8_t>(data.at(2))<<8)|static_cast<uint8_t>(data.at(1))))/32768.0*180.0;
        angleY=static_cast<int16_t>(((static_cast<uint8_t>(data.at(4))<<8)|static_cast<uint8_t>(data.at(3))))/32768.0*180.0;
        angleZ=static_cast<int16_t>(((static_cast<uint8_t>(data.at(6))<<8)|static_cast<uint8_t>(data.at(5))))/32768.0*180.0;
    }
    if(data2.size())
    {
        gyroX=static_cast<int16_t>(((static_cast<uint8_t>(data2.at(2))<<8)|static_cast<uint8_t>(data2.at(1))))/32768.0*2000.0;
        gyroY=static_cast<int16_t>(((static_cast<uint8_t>(data2.at(4))<<8)|static_cast<uint8_t>(data2.at(3))))/32768.0*2000.0;
        gyroZ=static_cast<int16_t>(((static_cast<uint8_t>(data2.at(6))<<8)|static_cast<uint8_t>(data2.at(5))))/32768.0*2000.0;
    }
    if(data3.size())
    {
        accX=static_cast<int16_t>(((static_cast<uint8_t>(data3.at(2))<<8)|static_cast<uint8_t>(data3.at(1))))/32768.0*16.0*gra;
        accY=static_cast<int16_t>(((static_cast<uint8_t>(data3.at(4))<<8)|static_cast<uint8_t>(data3.at(3))))/32768.0*16.0*gra;
        accZ=static_cast<int16_t>(((static_cast<uint8_t>(data3.at(6))<<8)|static_cast<uint8_t>(data3.at(5))))/32768.0*16.0*gra;
        auto dir=kinematicModule->GetOrientDir(angleX,angleY,0);
        dir.col(0)*=accX;
        dir.col(1)*=accY;
        dir.col(2)*=accZ;
        orientAccZ=dir.row(2).sum();
        orientAccZ_AfterFilter=simpleKalman(orientAccZ,processNoise_Q,measureNoise_R);
    }
    static bool onceFlag=false;
    if(data4.size())
    {
        ImuTime t_stamp;
        t_stamp.year=static_cast<uint8_t>(data4.at(1));
        t_stamp.month=static_cast<uint8_t>(data4.at(2));
        t_stamp.day=static_cast<uint8_t>(data4.at(3));
        t_stamp.hour=static_cast<uint8_t>(data4.at(4));
        t_stamp.minite=static_cast<uint8_t>(data4.at(5));
        t_stamp.seconds=static_cast<uint8_t>(data4.at(6));
        t_stamp.ms=static_cast<int16_t>(((static_cast<uint8_t>(data4.at(8))<<8)|static_cast<uint8_t>(data4.at(7))));
        if(onceFlag)
        {
            if(calculateDis)
            {
                double duration=ImuTime::GetDuration(t_stamp,timeStamp)/1000.0;
                //qDebug()<<"duration: "<<duration*1000;
#if ZUPT
//                if(!detector->DetectZero(orientAccZ_AfterFilter))
//                {
//                    disZ=disZ+velZ*duration+0.5*(orientAccZ_AfterFilter-staticAcc)*duration*duration;
//                    velZ+=duration*(orientAccZ_AfterFilter-staticAcc);
//                    frame_mutex.lock();
//                    globalStableStatus=false;
//                    frame_cond.wakeOne();
//                    frame_mutex.unlock();
//                }
//                else
//                {
//                    frame_mutex.lock();
//                    globalStableStatus=true;
//                    frame_mutex.unlock();
//                    velZ=0;
//                }
#else
                disZ=disZ+velZ*duration+0.5*(orientAccZ-staticAcc)*duration*duration;
                velZ+=duration*(orientAccZ-staticAcc);
                disZ_AfterMinSqure=disSolver->calculate(disZ,ImuTime::timeStampUntilNow);
                velZ_AfterMinSqure=velSolver->calculate(velZ,ImuTime::timeStampUntilNow);
#endif
            }
        }
        else
        {
            onceFlag=true;
        }
        timeStamp=t_stamp;
    }
}
void Controller::sendData()
{
    imu_mutex.lock();
    auto data=globalByteArray;
    auto data2=globalGyroArray;
    imu_mutex.unlock();
    if(data.size() && data2.size())
    {
        if(imuClient!=nullptr)
        {
            data.remove(data.size()-2,2);
            for(int i=0;i<6;i++)
                data.append(static_cast<char>(pid_regulator[i]->GetOutput()));
            for(int i=0;i<6;i++)
            {
                data.append(static_cast<char>(static_cast<int16_t>(currentPos[i])>>8));
                data.append(static_cast<char>(static_cast<int16_t>(currentPos[i])&0x00ff));
            }
            for(int i=0;i<6;i++)
            {
                auto puls=kinematicModule->Len2Pulse(refPos[motion2LegIndex[i+1]]);
                data.append(static_cast<char>(static_cast<int16_t>(-puls)>>8));
                data.append(static_cast<char>(static_cast<int16_t>(-puls)&0x00ff));
            }
            for(int i=0;i<6;i++)
            {
                auto speed=kinematicModule->Speed2Pulse(refSpeed[motion2LegIndex[i+1]]);
                for(int j=0;j<4;j++)
                    data.append(static_cast<char>(static_cast<int32_t>(speed)>>(8*(3-j))));
            }
            data2.remove(data2.size()-2,2);
            data.append(data2);
            double accXmm=accX*1000,accYmm=accY*1000,accZmm=accZ*1000;
            data.append(static_cast<char>(static_cast<int16_t>(accXmm)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(accXmm)&0x00ff));
            data.append(static_cast<char>(static_cast<int16_t>(accYmm)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(accYmm)&0x00ff));
            data.append(static_cast<char>(static_cast<int16_t>(accZmm)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(accZmm)&0x00ff));
            double velXmm=velX*1000,velYmm=velY*1000,velZmm=velZ*1000;
            data.append(static_cast<char>(static_cast<int16_t>(velXmm)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(velXmm)&0x00ff));
            data.append(static_cast<char>(static_cast<int16_t>(velYmm)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(velYmm)&0x00ff));
            data.append(static_cast<char>(static_cast<int16_t>(velZmm)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(velZmm)&0x00ff));
            double disXmm=disX*1000,disYmm=disY*1000,disZmm=disZ*1000;
            data.append(static_cast<char>(static_cast<int16_t>(disXmm)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(disXmm)&0x00ff));
            data.append(static_cast<char>(static_cast<int16_t>(disYmm)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(disYmm)&0x00ff));
            data.append(static_cast<char>(static_cast<int16_t>(disZmm)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(disZmm)&0x00ff));
            double orientAccZmm=orientAccZ*1000;
            data.append(static_cast<char>(static_cast<int16_t>(orientAccZmm)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(orientAccZmm)&0x00ff));
            double orientAccZmm_AfterFilter=orientAccZ_AfterFilter*1000;
            data.append(static_cast<char>(static_cast<int16_t>(orientAccZmm_AfterFilter)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(orientAccZmm_AfterFilter)&0x00ff));
            double var_watch=detector->GetCurrentVar()*1000;
            data.append(static_cast<char>(static_cast<int16_t>(var_watch)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(var_watch)&0x00ff));
            double velZ_AfterMinSquremm=velZ_AfterMinSqure*1000;
            data.append(static_cast<char>(static_cast<int16_t>(velZ_AfterMinSquremm)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(velZ_AfterMinSquremm)&0x00ff));
            double disZ_AfterMinSquremm=disZ_AfterMinSqure*1000;
            data.append(static_cast<char>(static_cast<int16_t>(disZ_AfterMinSquremm)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(disZ_AfterMinSquremm)&0x00ff));
            imu_mutex.lock();
            double velZ_othermm=globalVelZ*1000,disZ_othermm=globalDisZ*1000;
            imu_mutex.unlock();
            data.append(static_cast<char>(static_cast<int16_t>(velZ_othermm)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(velZ_othermm)&0x00ff));
            data.append(static_cast<char>(static_cast<int16_t>(disZ_othermm)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(disZ_othermm)&0x00ff));
            double zTrans=0,xTrans=0,yTrans=0;
            frame_mutex.lock();
            if(!FrameTranslation.empty())
                zTrans=FrameTranslation.at<double>(1,0)*1000;
                xTrans=FrameTranslation.at<double>(0,0)*1000;
                yTrans=FrameTranslation.at<double>(2,0)*1000;
            frame_mutex.unlock();
            data.append(static_cast<char>(static_cast<int16_t>(zTrans)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(zTrans)&0x00ff));
            data.append(static_cast<char>(static_cast<int16_t>(xTrans)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(xTrans)&0x00ff));
            data.append(static_cast<char>(static_cast<int16_t>(yTrans)>>8));
            data.append(static_cast<char>(static_cast<int16_t>(yTrans)&0x00ff));
            union {float f;uint32_t d;};
            f=zTransVision;
            for(int i=0;i<4;i++)
                data.append(static_cast<char>((d>>(8*(3-i)))&0xff));
            imuClient->write(data);
        }
    }
}

int ImuTime::GetDuration(ImuTime new_stamp,ImuTime old_stamp)
{
    if(new_stamp.year>old_stamp.year)
        new_stamp.month+=12*(new_stamp.year-old_stamp.year);
    if(new_stamp.month>old_stamp.month)
        new_stamp.day+=30*(new_stamp.month-old_stamp.month);
    if(new_stamp.day>old_stamp.day)
        new_stamp.hour+=24*(new_stamp.day-old_stamp.day);
    if(new_stamp.hour>old_stamp.hour)
        new_stamp.minite+=60*(new_stamp.hour-old_stamp.hour);
    if(new_stamp.minite>old_stamp.minite)
        new_stamp.seconds+=60*(new_stamp.minite-old_stamp.minite);
    if(new_stamp.seconds>old_stamp.seconds)
        new_stamp.ms+=1000*(new_stamp.seconds-old_stamp.seconds);
    ImuTime::timeStampUntilNow+=new_stamp.ms-old_stamp.ms;
    return new_stamp.ms-old_stamp.ms;
}
void Controller::correctionGra()
{
    double sumAccZ=0;
    int times=120;
    disSolver=new minsqureSolver(times,3);
    velSolver=new minsqureSolver(times,2);
    for(int i=0;i<times;i++)
    {
        analyseData(false);
        sumAccZ+=orientAccZ;
        QThread::msleep(40);
    }
    staticAcc=sumAccZ/times;
    imu_mutex.lock();
    globalStaticGravity=staticAcc;
    imu_mutex.unlock();
#if !ZUPT
    for(int i=0;i<times;i++)
    {
        analyseData(true);
        disSolver->addData({ImuTime::timeStampUntilNow/1000.0*ImuTime::timeStampUntilNow/1000.0,ImuTime::timeStampUntilNow/1000.0,1},disZ,i);
        velSolver->addData({ImuTime::timeStampUntilNow/1000.0,1},velZ,i);
        QThread::msleep(100);
    }
    disSolver->solveParams();
    velSolver->solveParams();
#endif
}
double Controller::simpleKalman(double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
    double R = MeasureNoise_R;
    double Q = ProcessNiose_Q;

    static double x_last;
    double x_mid = x_last;
    double x_now;

    static double p_last;
    double p_mid ;
    double p_now;

    double kg;

    x_mid=x_last;                       //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid=p_last+Q;                     //p_mid=p(k|k-1),p_last=p(k-1|k-1)


    kg=p_mid/(p_mid+R);
    x_now=x_mid+kg*(ResrcData-x_mid);
    p_now=(1-kg)*p_mid;
    p_last = p_now;
    x_last = x_now;

    return x_now;
}
void Controller::StartFrameProcess(int method)
{
    if(processThread)
        return;
    processThread=new ImgProcessThread(method);
    qDebug()<<"start img process";
    processThread->start();
}
