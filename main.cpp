#include <QCoreApplication>
#include <QDebug>
#include <iostream>
#include "controller.h"
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    qDebug() << "Program Start\nusing serial: controlCard:"<<argv[1]<<",modBus:"<<argv[2];
    qDebug() << "**********************************************";
    Controller controller(argv[1], argv[2]);
    //controller.simpleOperationMode();
    //controller.resetAll();
    controller.GuiControlMode();
    qDebug()<<"enter qt event loop";
    return a.exec();
}
