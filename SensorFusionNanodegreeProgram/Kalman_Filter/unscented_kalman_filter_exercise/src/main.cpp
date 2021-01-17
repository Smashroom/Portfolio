#include <iostream>
#include "Eigen/Dense"
#include "ukf.h"

using Eigen::MatrixXd;

int main() {

  // Create a UKF instance
  UKF ukf;

  /**
   * Programming assignment calls
   */
  Eigen::VectorXd x_out = Eigen::VectorXd(5);
  MatrixXd P_out = MatrixXd(5,5);
  
  ukf.UpdateState(&x_out, &P_out);

  // print result
  //std::cout << "Xsig = " << std::endl << Xsig << std::endl;

  return 0;
}