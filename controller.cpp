#include <QDebug>
#include "controller.h"
#include "GAS_N.h"
Controller::Controller(char* com):uart(new SerialPort),timer(new QTimer)
{
    int res=GA_Open(1,com);
    if(res==0)
    {
        qDebug()<<"open card successful";
    }
    else
    {
        qDebug()<<"open card failed";
    }
    double pos=0;
    GA_GetAxisPrfPos(6,&pos);
    qDebug()<<"now at: "<<pos;
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
