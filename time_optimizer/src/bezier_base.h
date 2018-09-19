/*
This header file is used to provide some basic mathematic support for the Bernstein-basis trajectory generation optimization problem. Includes:
1-: Mapping matrix maps the coefficients of the Bernstein basis (ie. control points) to Monomial basis. The mapping matrix range from order 3 to order 10
2-: Modulus list of the Bernstein basis to a given order. That is, pre-compute the constant-modulus (the 'n choose k' combinatorial) of the basis vector. 
	To save computation cost of frequently call this value. 

The class should be initialized to a instance before the trajectory generator called. 
Several initializer are provided, and the instance is initialized according to the given order of the control points.
*/

#ifndef _BEZIER_BASE_H_
#define _BEZIER_BASE_H_

#include <stdio.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

class Bernstein
{
	private:
		
		vector<MatrixXd> MQMList_l, MQMList_u, MList, MQMList_arc;
		vector<MatrixXd> FMList;
		vector<VectorXd> CList, CvList, CaList, CjList;

		int _order_min = 3, _order_max = 12;  // The order of the polynomial in each segment, also the number of control points used in each segment
		double _min_order;                    // The order to which we minimize.   1 -- velocity, 2 -- acceleration, 3 -- jerk, 4 -- snap    

	public:
		Bernstein(){} // Empty constructor
		~Bernstein(){}

		void setOrder(double min_order);
		
		MatrixXd CholeskyDecomp(MatrixXd Q); // return square root F of Q; Q = F' * F

		vector<MatrixXd> getM(){ return MList; }
		vector<MatrixXd> getMQM_l(){ return MQMList_l; }
		vector<MatrixXd> getMQM_u(){ return MQMList_u; }
		vector<MatrixXd> getMQM_arc(){ return MQMList_arc; }
		vector<MatrixXd> getFM(){ return FMList; }
		vector<VectorXd> getC(){ return CList; }
		vector<VectorXd> getC_v(){ return CvList; }
		vector<VectorXd> getC_a(){ return CaList; }
		vector<VectorXd> getC_j(){ return CjList; }
};

#endif