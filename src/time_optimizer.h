#ifndef _TIME_OPTIMIZER_H_
#define _TIME_OPTIMIZER_H_

#include <Eigen/Dense>
#include <vector>
#include "timeAllocator.h"

class MinimumTimeOptimizer 
{
private:
        double _T;    // the reulsted minimum time for a feasible travesal
        Eigen::MatrixXd _P;  // recording the polynomial's coefficients for further evaluation
        int _seg_num, _poly_num1D;

        double _objective;
        Allocator * time_allocator; // for return the final result to high-level planer

        Eigen::Vector3d getVel(int k, double s);
        Eigen::Vector3d getAcc(int k, double s);

        Eigen::Vector3d getVelPoly(int k, double s);
        Eigen::Vector3d getAccPoly(int k, double s);

        Eigen::Vector3d getVelBezier(int k, double s);
        Eigen::Vector3d getAccBezier(int k, double s);

public:
        MinimumTimeOptimizer();
        ~MinimumTimeOptimizer();

        void MinimumTimeGeneration( const Eigen::MatrixXd & polyCoeff, 
                                          Eigen::VectorXd & time,
                                    const double & maxVel, 
                                    const double & maxAcc,
                                    const double & maxdAcc,
                                    const double & d_s); 

        Allocator * GetTimeAllcoation() {return time_allocator;}
};

#endif
