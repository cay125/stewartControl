#include <QDebug>
#include <iostream>
#include "minsquresolver.h"

minsqureSolver::minsqureSolver(int times,int _order):order(_order),x(times,_order),y(times,1)
{

}
void minsqureSolver::addData(QVector<double> x_vec, double y_vec,int index)
{
    if(x_vec.size()!=order)
    {
        qDebug()<<"minsqure solver error!!";
        exit(1);
    }
    for(int i=0;i<order;i++)
        x(index,i)=x_vec[i];
    y(index,0)=y_vec;
}
QVector<double> minsqureSolver::solveParams()
{
    Eigen::MatrixXd ans=(x.transpose()*x).inverse()*x.transpose()*y;
    qDebug()<<"params size: "<<ans.rows()<<"X"<<ans.cols();
    for(int i=0;i<ans.rows();i++)
    {
        std::cout<<ans(i,0)<<" ";
        params.push_back(ans(i,0));
    }
    std::cout<<"\n";
    haveSolvered=true;
    return params;
}
double minsqureSolver::calculate(double input,int now_timestamp)
{
    if(!haveSolvered)
        return input;
    double factor=1;
    for(int i=order-1;i>=0;i--)
    {
        input-=factor*params[i];
        factor*=now_timestamp/1000.0;
    }
    return input;
}
