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
	//std::cout << "init error " << p_error << "  " << i_error<< "  " << d_error << std::endl; 
	double best_error = TotalError();
	vector<double> dp = {1.0,1.0,1.0};
	vector<double> p = {p_error,i_error,d_error};
	double error = 0.0;
	double total = 3.0;
	int j = 0;
//	std::cout <<" before total " << accumulate(dp.begin(),dp.end(),0)  << std::endl;
	//std::cout << " best_error " << best_error << std::endl;
	while(total > 0.001 && j < 500) {
		for (int i = 0; i < dp.size(); i++) {
			j++;
	//		std::cout<< j << std::endl;
			p[i] = p[i] + dp[i];
			error = TotalError();
//			std::cout << " 1 error " << error << std::endl;
			if (error < best_error) {
				best_error = error;
				dp[i] = dp[i]*1.1;
			} else {
				p[i] = p[i] - 2 * dp[i];
				error = TotalError();
		//		std::cout << " 2 error " << error << std::endl;
				if (error < best_error) {
					best_error = error;
					dp[i] = dp[i]*1.1;
				} else {
					p[i] = p[i] + dp[i];
					dp[i] = dp[i]*0.90;
				}
			}
		}
		total = dp[0] + dp[1] + dp[2];
//		std::cout << " dp 0 is "<< dp[0] << std::endl;
//		std::cout << " dp 1 is "<< dp[1] << std::endl;
//		std::cout << " dp 2 is "<< dp[2] << std::endl;
		//std::cout <<" total after" << total  << std::endl;
//		j = 0;
	}

	Kp = dp[0];
	Ki = dp[1];
	Kd = dp[2];
//   std::cout << " update error "<< Kp << "  " << Ki << "  " << Kd << std::endl; 
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  //std::cout << " K vaues "<< Kp << "  " << Ki << "  " << Kd << std::endl; 
  //std::cout << " total error "<< p_error << "  " << i_error << "  " << d_error << std::endl; 
  return -(Kp * p_error + Ki * i_error + Kd * d_error);  // TODO: Add your total error calc here!
}
