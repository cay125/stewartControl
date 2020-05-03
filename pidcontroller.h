#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController
{
public:
    PIDController(double kp,double ki,double kd,double componentKpMax,double componentKiMax,double componentKdMax,double outputMax,double outputMin);
    void calculate();
    void setFeedBack(double _fdb);
    void setRef(double _ref);
    double GetOutput();
private:
    double ref;
    double fdb;
    double inte;
    double err[2];
    double kp;
    double ki;
    double kd;
    double componentKp;
    double componentKi;
    double componentKd;
    double componentKpMax;
    double componentKiMax;
    double componentKdMax;
    double delta;
    double output;
    double outputMax;
    double outputMin;
    //double kp_offset;
    //double ki_offset;
    //double kd_offset;
};

#endif // PIDCONTROLLER_H
