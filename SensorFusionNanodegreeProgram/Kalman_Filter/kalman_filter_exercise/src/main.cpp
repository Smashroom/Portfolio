#include "kalman_filter.h"

int main() {
  /**
   * Code used as example to work with Eigen matrices
   */
  // design the KF with 1D motion
  KF kf;
  
  kf.x = Eigen::VectorXd(2);
  kf.x << 0, 0;

  kf.P = Eigen::MatrixXd(2, 2);
  kf.P << 1000, 0, 0, 1000;

  kf.u = Eigen::VectorXd(2);
  kf.u << 0, 0;

  kf.F = Eigen::MatrixXd(2, 2);
  kf.F << 1, 1, 0, 1;

  kf.H = Eigen::MatrixXd(1, 2);
  kf.H << 1, 0;

  kf.R = Eigen::MatrixXd(1, 1);
  kf.R << 1;

  kf.Q = Eigen::MatrixXd(2, 2);
  kf.Q << 0, 0, 0, 0;

  // create a list of measurements
  Eigen::VectorXd single_meas(1);
  single_meas << 1;
  kf.measurements.push_back(single_meas);
  single_meas << 2;
  kf.measurements.push_back(single_meas);
  single_meas << 3;
  kf.measurements.push_back(single_meas);

  // call Kalman filter algorithm
  kf.filter();

  return 0;
}

