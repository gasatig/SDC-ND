#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
     VectorXd rmse(4);
     rmse << 0,0,0,0;
     for (int i=0; i < estimations.size(); ++i) {
         VectorXd residual = (estimations[i] - ground_truth[i]);
         residual = residual.array() * residual.array();
         rmse += residual;
      }
    rmse =  rmse/ estimations.size();
    // TODO: calculate the squared root
    rmse = rmse.array().sqrt();
    // return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
    MatrixXd Hj(3,4);

    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float sqr_px = px * px;
    float sqr_py = py * py;
    float sqrt_xy =  sqrt(sqr_px + sqr_py);
    float thrd_xy = pow((sqr_px + sqr_py), 1.5);

    Hj(0,2) = Hj(0,3) = Hj(1,2) = Hj(1,3) = 0;
    if (sqr_px < 0.0001)
        sqr_px = 0.0001;
    if (sqr_py < 0.0001)
        sqr_py = 0.0001;
    if (sqrt_xy < 0.0001)
        sqrt_xy = 0.0001;
    if (thrd_xy < 0.0001)
        thrd_xy = 0.0001;

    Hj(0,0) = px / sqrt_xy;
    Hj(0,1) = py / sqrt_xy;
    Hj(1,0) = -py / (sqr_px + sqr_py);
    Hj(1,1) = px / (sqr_px + sqr_py);
    Hj(2,0) = py * (vx * py -  vy * px) / thrd_xy;
    Hj(2,1) = px * (vy * px - vx * py) / thrd_xy;
    Hj(2,2) = px /sqrt_xy;
    Hj(2,3) = py / sqrt_xy;

    return Hj;
}
