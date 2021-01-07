/** 
 * Write a function 'filter()' that implements a multi-
 *   dimensional Kalman Filter for the example given
 */

#include "kalman_filter.h"

KF::KF() { 
  I = Eigen::MatrixXd::Identity(2, 2);
}

void KF::filter(){
  Eigen::MatrixXd K,S;
  Eigen::VectorXd y,z;  
  for (unsigned int n = 0; n < measurements.size(); ++n) {

    z = measurements[n];
    // KF Measurement update step
		// initialise the error matrix 
    y = z - H*x;
    // Project prediction 
    S = H*P*(H.transpose()) + R;
    // Kalman gain 
    K = P*(H.transpose())*(S.inverse());
    // new state
		x = x + K*y;
    P = (I - K*H)*P;
    // KF Prediction step
		x = F*x + u;
    P = F*P*(F.transpose());
    std::cout << "x=" << std::endl <<  x << std::endl;
    std::cout << "P=" << std::endl <<  P << std::endl;
  }
}

