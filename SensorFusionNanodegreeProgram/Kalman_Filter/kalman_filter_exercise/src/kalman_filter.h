#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <iostream>
#include <vector>
#include "Eigen/Dense"

class KF{
  public:
    KF();
    // Kalman Filter variables
    Eigen::VectorXd x;	// object state
    Eigen::MatrixXd P;	// object covariance matrix
    Eigen::VectorXd u;	// external motion
    Eigen::MatrixXd F; // state transition matrix
    Eigen::MatrixXd H;	// measurement matrix
    Eigen::MatrixXd R;	// measurement covariance matrix
    Eigen::MatrixXd I; // Identity matrix
    Eigen::MatrixXd Q;	// process covariance matrix
    // In the future can be converted to a queue    
    std::vector<Eigen::VectorXd> measurements;
    
    // functions
    void filter();
};

#endif /* kalman_filter_h */