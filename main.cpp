#include <QCoreApplication>
#include <QDebug>
#include <iostream>
#include "controller.h"
#include "GAS_N.h"
#include <windows.h>

BOOL HandlerRoutine(DWORD dwCtrlType)
{
    GA_Stop(0x3f,0);
    GA_Close();
    switch (dwCtrlType)
    {
        case CTRL_C_EVENT:
            std::cout<<"ctrl+c\n";
            return TRUE;
        case CTRL_CLOSE_EVENT:
            std::cout<<"ctrl close\n";
            return TRUE;
        case CTRL_BREAK_EVENT:
            std::cout<<"CTRL_BREAK_EVENT\n";
        case CTRL_LOGOFF_EVENT:
            std::cout<<"CTRL_LOGOFF_EVENT\n";
        case CTRL_SHUTDOWN_EVENT:
            std::cout<<"CTRL_SHUTDOWN_EVENT\n";
        default:
            return FALSE;
    }
}
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    SetConsoleCtrlHandler((PHANDLER_ROUTINE)HandlerRoutine,TRUE);
    qDebug() << "Program Start\nusing serial: controlCard:"<<argv[1]<<",modBus:"<<argv[2]<<",imu:"<<argv[3];
    qDebug() << "**********************************************";
    Controller controller(argv[1], argv[2], argv[3]);
    //controller.simpleOperationMode();
    controller.resetAll();
    controller.IMUControlMode();
    //controller.GuiControlMode();
    //controller.simpleTrajectory(0);
    qDebug() << "enter qt event loop";
    return a.exec();
}
