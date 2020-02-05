#include "PID.h"
#include <vector>
#include<numeric>
#include <iostream>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

using std::vector;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	this->Kp = Kp_;
  	this->Ki = Ki_;
	this->Kd = Kd_;

	this->p_error = 0.0;
	this->i_error = 0.0;
	this->d_error = 0.0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
	d_error = cte - p_error;
    p_error = cte;
	i_error = cte + i_error;

	double best_error = TotalError();
	vector<double> dp = {1.0,1.0,1.0};
	vector<double> p = {p_error,i_error,d_error};
	double error = 0.0;
	int j = 0;
	std::cout <<" before total " << accumulate(dp.begin(),dp.end(),0)  << std::endl;
	while(accumulate(dp.begin(),dp.end(),0) > 0.01) {
		for (int i = 0; i < dp.size(); i++) {
			j++;
			std::cout<< j << std::endl;
			p[i] = p[i] + dp[i];
			error = TotalError();
			if (error < best_error) {
				best_error = error;
				dp[i] = dp[i]*1.1;
			} else {
				p[i] = p[i] - 2 * dp[i];
				error = TotalError();
				if (error < best_error) {
					best_error = error;
					dp[i] = dp[i]*1.1;
				} else {
					p[i] = p[i] + dp[i];
					dp[i] = dp[i]*0.90;
				}
			}
		}
		std::cout <<" total " << accumulate(dp.begin(),dp.end(),0)  << std::endl;
	}

	Kp = p[0];
	Ki = p[1];
	Kd = p[2];
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return -(Kp * p_error + Ki * i_error + Kd * d_error);  // TODO: Add your total error calc here!
}
