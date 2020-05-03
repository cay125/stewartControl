#include <QCoreApplication>
#include <QDebug>
#include <iostream>
#include "controller.h"
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
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
