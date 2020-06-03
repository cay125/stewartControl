#ifndef MINSQURESOLVER_H
#define MINSQURESOLVER_H

#include <Eigen/Dense>
#include <QVector>

class minsqureSolver
{
public:
    minsqureSolver(int times,int _order=3);
    void addData(QVector<double> x_vec,double y_vec,int index);
    QVector<double> solveParams();
    double calculate(double input,int now_timestamp);
private:
    bool haveSolvered=false;
    QVector<double> params;
    int order=3;
    Eigen::MatrixXd x;
    Eigen::MatrixXd y;
};

#endif // MINSQURESOLVER_H
