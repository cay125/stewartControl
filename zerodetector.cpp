#include "zerodetector.h"

ZeroDetector::ZeroDetector(double gra):staticGra(gra)
{

}
ZeroDetector::ZeroDetector():staticGra(0)
{

}
void ZeroDetector::setStaticGra(double accZ)
{
    staticGra=accZ;
}
bool ZeroDetector::DetectZero(double accZ)
{
    addData({accZ});
    if(historyData.size()>N)
        removeData({historyData[0]});
    if(currentVar>varianThresh || fabs(accZ-staticGra)>staticThresh)
        return false;
    else
        return true;
}
double ZeroDetector::GetCurrentVar()
{
    return currentVar;
}
double ZeroDetector::GetCurrentMean()
{
    return currentMean;
}

void ZeroDetector::addData(QVector<double> input)
{
    if(input.size()==0)
            return;
    double sumInput=0;
    for(int i=0;i<input.size();i++)
    {
        historyData.push_back(input[i]);
        sumInput+=input[i];
    }
    double meanInput=sumInput/input.size();
    double varianceInput=0;
    for(int i=0;i<input.size();i++)
        varianceInput+=(meanInput-input[i])*(meanInput-input[i]);
    varianceInput/=input.size();
    if(dataCnt==0)
    {
        currentVar=varianceInput;
    }
    else
    {
        double meanNew=(dataTotalSum+sumInput)/(dataCnt+input.size());
        double t1=dataCnt*(currentVar+(meanNew-currentMean)*(meanNew-currentMean));
        double t2=input.size()*(varianceInput+(meanNew-meanInput)*(meanNew-meanInput));
        currentVar=(t1+t2)/(dataCnt+input.size());
    }
    dataCnt+=input.size();
    dataTotalSum+=sumInput;
    currentMean=dataTotalSum/dataCnt;
}
void ZeroDetector::removeData(QVector<double> input)
{
    if(input.size()==0)
            return;
    double sumInput=0;
    for(int i=0;i<input.size();i++)
        sumInput+=input[i];
    double meanInput=sumInput/input.size();
    double varianceInput=0;
    for(int i=0;i<input.size();i++)
        varianceInput+=(meanInput-input[i])*(meanInput-input[i]);
    varianceInput/=input.size();
    double meanNew=(dataTotalSum-sumInput)/(dataCnt-input.size());
    double t1=dataCnt*(currentVar+(meanNew-currentMean)*(meanNew-currentMean));
    double t2=input.size()*(varianceInput+(meanNew-meanInput)*(meanNew-meanInput));
    currentVar=(t1-t2)/(dataCnt-input.size());
    dataCnt-=input.size();
    dataTotalSum-=sumInput;
    currentMean=dataTotalSum/dataCnt;
    historyData.remove(0,input.size());
}
