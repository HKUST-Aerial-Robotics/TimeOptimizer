#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

#include <Eigen/Dense>
#include <vector>

class Trajectory
{

protected:

    Eigen::MatrixXd coeff; // coefficients of the piecewise trajectory
    Eigen::VectorXd time;  // time durations of the piecewise trajectory
    int poly_num1D;

public: 

    Trajectory( Eigen::MatrixXd _coeff, Eigen::VectorXd _time)
    {   
        coeff = _coeff;    
        time  = _time;  
        poly_num1D = coeff.cols() / 3;  
    };

    Trajectory(){};
    ~Trajectory(){};

    Eigen::MatrixXd getP() const {return coeff;      };
    Eigen::VectorXd getT() const {return time;       };
    int             getO() const {return poly_num1D; };

    virtual Eigen::Vector3d getVel(int k, double s) const = 0;
    virtual Eigen::Vector3d getAcc(int k, double s) const = 0;

};

#endif