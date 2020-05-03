#include "pidcontroller.h"

PIDController::PIDController(double kp,double ki,double kd,double componentKpMax,double componentKiMax,double componentKdMax,double outputMax,double outputMin)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->inte = 0;
    this->componentKpMax = componentKpMax;
    this->componentKiMax = componentKiMax;
    this->componentKdMax = componentKdMax;
    this->outputMax = outputMax;
    this->outputMin = outputMin;
}
void PIDController::setRef(double _ref)
{
    ref=_ref;
}
void PIDController::setFeedBack(double _fdb)
{
    fdb=_fdb;
}
double PIDController::GetOutput()
{
    return output;
}
void PIDController::calculate()
{
    err[0] = err[1];
    err[1] = ref - fdb;
    inte+=err[1];

    componentKp  = kp * err[1];
    componentKi  = ki * inte;
    componentKd  = kd * (err[1] - err[0]);

    if(inte > 200)
        inte = 200;
    else if (inte < -200)
        inte = -200;

    if(componentKp > componentKpMax)
        componentKp = componentKpMax;
    else if (componentKp < -componentKpMax)
        componentKp = -componentKpMax;

    if(componentKi > componentKiMax)
        componentKi = componentKiMax;
    else if (componentKi < -componentKiMax)
        componentKi = -componentKiMax;

    if(componentKd > componentKdMax)
        componentKd = componentKdMax;
    else if (componentKd < -componentKdMax)
        componentKd = -componentKdMax;

    //delta = componentKp + componentKi+ componentKd;
    //output = output + delta;
    output = componentKp + componentKi+ componentKd;

    if(output > outputMax)
        output = outputMax;
    else if (output < outputMin)
        output = outputMin;
}
