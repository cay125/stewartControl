#ifndef ZERODETECTOR_H
#define ZERODETECTOR_H

#include <QVector>

class ZeroDetector
{
public:
    ZeroDetector(double gra);
    ZeroDetector();
    bool DetectZero(double accZ);
    void setStaticGra(double accZ);
    double GetCurrentMean();
    double GetCurrentVar();

private:
    void addData(QVector<double> input);
    void removeData(QVector<double> input);

    QVector<double> historyData;
    double staticGra;
    double currentVar=0;
    double currentMean=0;
    double dataTotalSum=0;
    int    dataCnt=0;

    const double N=10;
    const double varianThresh=0.5;
    const double staticThresh=0.5;
};

#endif // ZERODETECTOR_H
