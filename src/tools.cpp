#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

#include <iostream>

Tools::Tools() {};

Tools::~Tools() {};

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
	/**
	 * TODO: Calculate the RMSE here.
	 */

	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// Important Note Here that the Estimation Vector Shouldn't be Equal to Zero.
	// And, its size should be equal to ground truth vector size.

	// Sum Squared Residuals
	for(unsigned int i=0; i < estimations.size(); ++i)
	{
		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	// Now, we calculate the Mean value
	rmse = rmse/estimations.size();

	// Now, Sqaured Root
	//calculate the squared root
	rmse = rmse.array().sqrt();

	return rmse;	// Finally, Return RMSE.
};

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
	/**
	 * TODO:
	 * Calculate a Jacobian here.
	 */
	
	MatrixXd Hj = MatrixXd::Zero(3,4);
	// Now, again Here, we have state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// Inorder to avoid repeated calculation, We Pre Compute a set of Terms as
	float c1 = px*px + py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	// We also have to Check Division by 0
	if(std::fabs(c1) < 0.0001)
	{
		std::cout << "Error - Division by Zero" << std::endl;
		return Hj;
	}

	//Now we Finally Compute the Jacobian matrix as:
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
	
	return Hj;	// Return Jacobian Matrix
};