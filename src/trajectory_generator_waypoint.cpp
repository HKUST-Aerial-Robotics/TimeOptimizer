#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;    
using namespace Eigen;

#define inf 1>>30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // initial velocity
            const Eigen::MatrixXd &Acc,           // initial acceleration 
            const Eigen::VectorXd &Time)          // time allocation in each segment
{     
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    cout<<"d_order: "<<d_order<<endl;
    int p_order = 2 * d_order - 1; // the order of polynomial
    int p_num1d   = p_order + 1;     // the number of variables in each segment

    int m = Time.size();
    MatrixXd PolyCoeff(m, 3 * p_num1d);
    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

    int num_f, num_p; // number of fixed and free variables
    int num_d;        // number of all segments' derivatives
    const static auto Factorial = [](int x){
        int fac = 1;

        for(int i = x; i > 0; i--)
            fac = fac * i;
          
        return fac;
    };

    /*   Produce Mapping Matrix A to the entire trajectory.   */
    MatrixXd Ab;
    MatrixXd A = MatrixXd::Zero(m * p_num1d, m * p_num1d);

    for(int k = 0; k < m; k++)
    {
        Ab = Eigen::MatrixXd::Zero(p_num1d, p_num1d);
        for(int i = 0; i < d_order; i++)
        {
            Ab(2 * i, i) = Factorial(i);
            for(int j = i; j < p_num1d; j++)
                Ab( 2 * i + 1, j ) = (double)Factorial(j) / (double)Factorial( j - i ) * pow( Time(k), j - i );
        }
        A.block(k * p_num1d, k * p_num1d, p_num1d, p_num1d) = Ab;    
    }

    MatrixXd A_inv   = A.inverse();

    cout<<"A:\n"<<A<<endl;
    cout<<"inverse A:\n"<<A_inv<<endl;

    //ROS_WARN("[Generator] A finished");
    /*   Produce the dereivatives in X, Y and Z axis directly.  */
    VectorXd Dx = VectorXd::Zero(m * p_num1d);
    VectorXd Dy = VectorXd::Zero(m * p_num1d);
    VectorXd Dz = VectorXd::Zero(m * p_num1d);

    for(int k = 1; k < (m + 1); k ++ ){
        Dx((k-1)*p_num1d) = Path(k - 1, 0); Dx((k-1)*p_num1d + 1) = Path(k, 0); 
        Dy((k-1)*p_num1d) = Path(k - 1, 1); Dy((k-1)*p_num1d + 1) = Path(k, 1); 
        Dz((k-1)*p_num1d) = Path(k - 1, 2); Dz((k-1)*p_num1d + 1) = Path(k, 2); 
        
        if( k == 1 )
        {
            Dx((k-1)*p_num1d + 2) = Vel(0, 0);
            Dy((k-1)*p_num1d + 2) = Vel(0, 1); 
            Dz((k-1)*p_num1d + 2) = Vel(0, 2);

            Dx((k-1)*p_num1d + 4) = Acc(0, 0);
            Dy((k-1)*p_num1d + 4) = Acc(0, 1); 
            Dz((k-1)*p_num1d + 4) = Acc(0, 2);
        }
        else if( k == m )
        {
            Dx((k-1)*p_num1d + 3) = Vel(1, 0);
            Dy((k-1)*p_num1d + 3) = Vel(1, 1); 
            Dz((k-1)*p_num1d + 3) = Vel(1, 2);

            Dx((k-1)*p_num1d + 5) = Acc(1, 0);
            Dy((k-1)*p_num1d + 5) = Acc(1, 1); 
            Dz((k-1)*p_num1d + 5) = Acc(1, 2);
        }
    }

    /*   Produce the Minimum Snap cost function, the Hessian Matrix   */
    MatrixXd H = MatrixXd::Zero( m * p_num1d, m * p_num1d );

    for(int k = 0; k < m; k ++){
        for(int i = 3; i < p_num1d; i ++){
            for(int j = 3; j < p_num1d; j ++){
                H( k*p_num1d + i, k*p_num1d + j ) = (double)i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (double)(i + j - 5) * pow( Time(k), (i + j - 5) );
            }
        }
    }

    _Q = H; // Now only minumum snap is used in the cost

    ROS_WARN("[Generator] Q finished");
    if( m > 1)
    {   
        MatrixXd Ct; // The transpose of selection matrix C
        MatrixXd C;  // The selection matrix C

        num_d = p_num1d * m;

        num_f = d_order + d_order + (m - 1) * (d_order - 1); // fixed
        num_p = (m - 1) * (d_order - 1);                     // free

        cout<<"num_p: "<<num_p<<endl;
        cout<<"num_f: "<<num_f<<endl;
        cout<<"num_d: "<<num_d<<endl;

        Ct = MatrixXd::Zero(num_d, num_f + num_p); 
        /*
        num_f = 2 * m + 4; //3 + 3 + (m - 1) * 2 = 2m + 4  // fixed
        num_p = 2 * m - 2; //(m - 1) * 2 = 2m - 2          // free
        
        Ct( 0, 0 ) = 1; Ct( 2, 1 ) = 1;         Ct( 4, 2 ) = 1; // stack the start point
        Ct( 1, 3 ) = 1; Ct( 3, 2 * m + 4 ) = 1; Ct( 5, 2 * m + 5 ) = 1; 
        
        Ct(p_num1d * (m - 1) + 0, 2 * m + 0) = 1; 
        Ct(p_num1d * (m - 1) + 1, 2 * m + 1) = 1; // Stack the end point
        Ct(p_num1d * (m - 1) + 2, 4 * m + 0) = 1;
        Ct(p_num1d * (m - 1) + 3, 2 * m + 2) = 1; // Stack the end point
        Ct(p_num1d * (m - 1) + 4, 4 * m + 1) = 1;
        Ct(p_num1d * (m - 1) + 5, 2 * m + 3) = 1; // Stack the end point
        
        for(int j = 2; j < m; j ++ ){
              Ct( 6 * (j - 1) + 0, 2 + 2 * (j - 1)         + 0 ) = 1;
              Ct( 6 * (j - 1) + 1, 2 + 2 * (j - 1)         + 1 ) = 1;
         
              Ct( 6 * (j - 1) + 2, 2 * m + 4 + 2 * (j - 2) + 0 ) = 1;
              Ct( 6 * (j - 1) + 4, 2 * m + 4 + 2 * (j - 2) + 1 ) = 1;
         
              Ct( 6 * (j - 1) + 3, 2 * m + 4 + 2 * (j - 1) + 0 ) = 1;
              Ct( 6 * (j - 1) + 5, 2 * m + 4 + 2 * (j - 1) + 1 ) = 1;
        }
        */
        
        //ROS_WARN("for the 1st segment");
        // for the 1st segment
        { 
          // enforcing the start states
          for(int i = 0; i < d_order; i++ )
              Ct( i * 2, i ) = 1;  
          
          // enforcing the position of the 2nd segment
          Ct( 1, d_order ) = 1;  
          
          // setting other derivatives as free
          for(int i = 1; i < d_order; i++ )
              Ct( i * 2 + 1, num_f - 1 + i ) = 1;  
        }

        //ROS_WARN("for the last segment");
        // for the last segment
        { 
          // enforcing the final states
          for(int i = 0; i < d_order; i++ )
            Ct(p_num1d * (m - 1) + 2 * i + 1, num_f - 1 - (d_order - 1) + i) = 1; // Stack the end point

          // enforcing the position of the 2nd to last segment
          Ct(p_num1d * (m - 1), num_f - 1 - (d_order - 1) - 1) = 1; // Stack the end point
          
          // setting other derivatives as free
          for(int i = 1; i < d_order; i++ )
          {
            Ct(p_num1d * (m - 1) + 2 * i, num_f + num_p - 1 - (d_order - 1) + i) = 1; // Stack the end point
          }
        }

        //ROS_WARN("for all middle segments");
        // for all meddle segments
        for(int j = 1; j < m - 1; j ++ )
        {   
            // enforcing fixed position at the start and final at this particular segment
            Ct( p_num1d * j + 0, (d_order - 1) + 2 * j + 0 ) = 1;
            Ct( p_num1d * j + 1, (d_order - 1) + 2 * j + 1 ) = 1;
            
            // setting other derivatives as free
            for(int i  = 1; i < d_order; i++)
            {
              Ct( p_num1d * j + 2 * i,     num_f + (d_order - 1) * (j - 1) + i - 1 ) = 1;
              Ct( p_num1d * j + 2 * i + 1, num_f + (d_order - 1) *  j      + i - 1 ) = 1;  
            }
        }

        C = Ct.transpose();
        MatrixXd A_invC  = A_inv * Ct;

        ROS_WARN("[Generator] case m > 1, C finished");
        VectorXd Dx1 = C * Dx;
        VectorXd Dy1 = C * Dy;
        VectorXd Dz1 = C * Dz;

        //ROS_WARN("[Generator] case segment > 1");
        MatrixXd R   = A_invC.transpose() * _Q *  A_invC;
        
        ROS_WARN("[Generator] case m > 1, R finished");
        VectorXd Dxf(num_f), Dyf(num_f), Dzf(num_f);
        
        Dxf = Dx1.segment( 0, num_f );
        Dyf = Dy1.segment( 0, num_f );
        Dzf = Dz1.segment( 0, num_f );

        MatrixXd Rff(num_f, num_f);
        MatrixXd Rfp(num_f, num_p);
        MatrixXd Rpf(num_p, num_f);
        MatrixXd Rpp(num_p, num_p);

        Rff = R.block(0,     0,     num_f, num_f);
        Rfp = R.block(0,     num_f, num_f, num_p);
        Rpf = R.block(num_f, 0,     num_p, num_f);
        Rpp = R.block(num_f, num_f, num_p, num_p);

        MatrixXd Rpp_inv = Rpp.inverse();
        ROS_WARN("[Generator] case m > 1, R blocks finished");
        VectorXd Dxp(num_p), Dyp(num_p), Dzp(num_p);
        Dxp = - (Rpp_inv * Rfp.transpose()) * Dxf;
        Dyp = - (Rpp_inv * Rfp.transpose()) * Dyf;
        Dzp = - (Rpp_inv * Rfp.transpose()) * Dzf;

        ROS_WARN("[Generator] case m > 1, Dp blocks finished");
        Dx1.segment(num_f, num_p) = Dxp;
        Dy1.segment(num_f, num_p) = Dyp;
        Dz1.segment(num_f, num_p) = Dzp;

        ROS_WARN("[Generator] case m > 1, Dx1 resembled finished");
        Px = A_invC * Dx1;
        Py = A_invC * Dy1;
        Pz = A_invC * Dz1;

        //cout<<"J: "<<Dx1.transpose() * R * Dx1 + Dy1.transpose() * R * Dy1 + Dz1.transpose() * R * Dz1<<endl;
    }
    else
    {   
        ROS_WARN("[Generator] case segment = 1");
        Px = A_inv * Dx;
        Py = A_inv * Dy;
        Pz = A_inv * Dz;
        //time_2 = ros::Time::now();
    }

    _Px = Px;
    _Py = Py;
    _Pz = Pz;

    for(int i = 0; i < m; i ++)
    {
        PolyCoeff.block(i, 0 * p_num1d, 1, p_num1d) = Px.segment( i * p_num1d, p_num1d ).transpose();
        PolyCoeff.block(i, 1 * p_num1d, 1, p_num1d) = Py.segment( i * p_num1d, p_num1d ).transpose();
        PolyCoeff.block(i, 2 * p_num1d, 1, p_num1d) = Pz.segment( i * p_num1d, p_num1d ).transpose();
    }

    /*time_3 = ros::Time::now();
    ROS_WARN("[Waypoint-Traj Solver] time in doing calculation is %f", (time_3 - time_2).toSec() );*/
    //ROS_WARN("[Generator] Unconstrained QP solved");

    //cout<<"objective: "<<_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py + _Pz.transpose() * _Q * _Pz<<endl;
    return PolyCoeff;
}  

double TrajectoryGeneratorWaypoint::getObjective()
{ 
      _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py + _Pz.transpose() * _Q * _Pz)(0);
      return _qp_cost; 
}