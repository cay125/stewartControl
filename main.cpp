#include <QCoreApplication>
#include <QDebug>
#include <iostream>
#include "controller.h"
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    QString com_name(argv[1]);
    qDebug() << "Program Start\nusing Serial:" << com_name;
    Controller controller(argv[1]);
    controller.simpleOperation();
    qDebug()<<"enter qt event loop";
    return a.exec();
}
