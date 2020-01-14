#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "serialport.h"
#include <QString>
#include <QTimer>

class Controller:public QObject
{
    Q_OBJECT
public:
    Controller(char* com);
    void simpleOperation();
private:
    SerialPort *uart;
    QTimer *timer;
private slots:
    void connectedSlot();
    void timerSlot();
signals:
    void portStartSignal(QString,int,int);
    void portCloseSignal();
};

#endif // CONTROLLER_H
