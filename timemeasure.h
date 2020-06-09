#ifndef TIMEMEASURE_H
#define TIMEMEASURE_H
#include <chrono>
#include <QDebug>
#define TIMEBEGIN() {\
                        auto time1=std::chrono::steady_clock::now();
#define TIMEEND(PARA) \
                        auto time2=std::chrono::steady_clock::now();\
                        std::chrono::duration<double, std::milli> dTimeSpan = std::chrono::duration<double,std::milli>(time2-time1);\
                        qDebug()<<PARA<<" "<<dTimeSpan.count()<<"ms";\
                    }
#endif // TIMEMEASURE_H
