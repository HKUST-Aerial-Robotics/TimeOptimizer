#ifndef _TIME_OPTIMIZER_H_
#define _TIME_OPTIMIZER_H_

#include <Eigen/Dense>
#include <vector>
#include "timeAllocator.h"

using namespace std;
using namespace Eigen;

class MinimumTimeOptimizer 
{
private:
        double _T;    // the reulsted minimum time for a feasible travesal
        MatrixXd _P;  // recording the polynomial's coefficients for further evaluation
        int _seg_num, _poly_num1D;

        int _poly_order, _ctrl_num1D;
        VectorXd _C, _Cv, _Ca;
        int _type = 0; // 0 for poly traj; 1 for bezier traj

        double _objective;
        Allocator * time_allocator; // for return the final result to high-level planer

        Vector3d getVel(int k, double s);
        Vector3d getAcc(int k, double s);

        Vector3d getVelPoly(int k, double s);
        Vector3d getAccPoly(int k, double s);

        Vector3d getVelBezier(int k, double s);
        Vector3d getAccBezier(int k, double s);

public:
        MinimumTimeOptimizer();
        ~MinimumTimeOptimizer();

        void MinimumTimeGeneration( const MatrixXd & polyCoeff, 
                                          VectorXd & time,
                                    const double & maxVel, 
                                    const double & maxAcc,
                                    const double & maxdAcc,
                                    const double & d_s); 

        Allocator * GetTimeAllcoation() {return time_allocator;}
};

#endif
