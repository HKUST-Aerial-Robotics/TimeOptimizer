#ifndef _TIME_OPTIMIZER_H_
#define _TIME_OPTIMIZER_H_

#include <Eigen/Dense>
#include <vector>
#include "timeAllocator.h"
#include "trajectory_base.h"

class MinimumTimeOptimizer 
{
private:
        Eigen::MatrixXd _P;  // recording the polynomial's coefficients for further evaluation
        Eigen::VectorXd _T;  // recording the polynomial's time durations
        int _seg_num, _poly_num1D;

        double _objective;
        Allocator * time_allocator; // for return the final result to high-level planer

public:
        MinimumTimeOptimizer();
        ~MinimumTimeOptimizer();

        int MinimumTimeGeneration( const Trajectory & traj,
                                    const double & maxVel, 
                                    const double & maxAcc,
                                    const double & maxdAcc,
                                    const double & d_s,
                                    const double & rho); 

        Allocator * GetTimeAllcoation() {return time_allocator;}
};

#endif
