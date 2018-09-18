#ifndef _TRAJECTORY_POLY_MONO_H_
#define _TRAJECTORY_POLY_MONO_H_

#include <Eigen/Dense>
#include <vector>
#include "trajectory_base.h"

class TrajPolyMono : public Trajectory
{

public:

    TrajPolyMono(Eigen::MatrixXd _coeff, Eigen::VectorXd _time):Trajectory(_coeff, _time) {};

    Eigen::Vector3d getVel(int k, double s) const
    {
        Eigen::Vector3d ret;
        for ( int dim = 0; dim < 3; dim++ )
        {
            Eigen::VectorXd poly1d = (coeff.row(k) ).segment( dim * poly_num1D, poly_num1D );
            Eigen::VectorXd t = VectorXd::Zero( poly_num1D );
            
            for(int j = 0; j < poly_num1D; j ++)
                if(j==0)
                    t(j) = 0.0;
                else
                    t(j) = j * pow(s, j-1);

            ret(dim) = poly1d.dot(t);
        }

        return ret;
    };

    Eigen::Vector3d getAcc(int k, double s) const
    {
        Eigen::Vector3d ret;

        for ( int dim = 0; dim < 3; dim++ )
        {
            Eigen::VectorXd poly1d = (coeff.row(k)).segment( dim * poly_num1D, poly_num1D );
            Eigen::VectorXd t = VectorXd::Zero( poly_num1D );

            for(int j = 0; j < poly_num1D; j ++)
                if( j==0 || j==1 )
                    t(j) = 0.0;
                else
                    t(j) = j * (j - 1) * pow(s, j-2);

            ret(dim) = poly1d.dot(t);
        }

        return ret;
    };

};
#endif