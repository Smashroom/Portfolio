#include "ukf.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF() {
  Init();
}

UKF::~UKF() {

}

void UKF::Init() {

}

void UKF::PredictRadarMeasurement(Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // define spreading parameter
  double lambda = 3 - n_aug;

  // set vector for weights
  Eigen::VectorXd weights = Eigen::VectorXd(2*n_aug+1);
  double weight_0 = lambda/(lambda+n_aug);
  double weight = 0.5/(lambda+n_aug);
  weights(0) = weight_0;

  for (int i=1; i<2*n_aug+1; ++i) {  
    weights(i) = weight;
  }

  // radar measurement noise standard deviation radius in m
  double std_radr = 0.3;

  // radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;

  // radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;

  Eigen::MatrixXd R = Eigen::MatrixXd(n_z, n_z);
  R << pow(std_radr,2), 0, 0,
       0, pow(std_radphi,2), 0,
       0, 0, pow(std_radrd, 2);
       
  // create example matrix with predicted sigma points
  Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  // create matrix for sigma points in measurement space
  Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2 * n_aug + 1);

  // mean predicted measurement
  Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);
  z_pred.fill(0.0);
  
  // measurement covariance matrix S
  Eigen::MatrixXd S = Eigen::MatrixXd(n_z,n_z);
  S.fill(0.0);

       
  /**
   * Student part begin
   */
  double px, py, v, yaw, yaw_rate, px2, py2;
  // transform sigma points into measurement space
  for(int i = 0; i < Xsig_pred.cols(); i++){
      px = Xsig_pred(0,i);
      py = Xsig_pred(1,i);
      v = Xsig_pred(2,i);
      yaw = Xsig_pred(3,i);
      yaw_rate = Xsig_pred(4,i);
      
      px2 = pow(px,2);
      py2 = pow(py,2);
      
      Zsig.col(i) << sqrt(px2+py2),
                     atan2(py,px),
                     v*(px*cos(yaw)+py*sin(yaw))/float(sqrt(px2+py2));
  }

  // calculate mean predicted measurement
  for(int i = 0; i < Zsig.cols(); i++){
    // predict state mean
    z_pred += weights(i)*Zsig.col(i);
  }
  
  // calculate innovation covariance matrix S
  Eigen::VectorXd z_diff; /* To check angle normalisation*/
  // predict state covariance matrix
  for(int i = 0; i < Zsig.cols(); i++){
      z_diff = Zsig.col(i) - z_pred;
      S += weights(i)*(z_diff)*(z_diff.transpose());
  }
  S += R;
  /**
   * Student part end
   */

  // print result
  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  std::cout << "S: " << std::endl << S << std::endl;

  // write result
  *z_out = z_pred;
  *S_out = S;
}

void UKF::PredictMeanAndCovariance(Eigen::VectorXd* x_out, Eigen::MatrixXd* P_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // define spreading parameter
  double lambda = 3 - n_aug;

  // create example matrix with predicted sigma points
  Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  // create vector for weights
  Eigen::VectorXd weights = Eigen::VectorXd(2*n_aug+1);
  
  // create vector for predicted state
  Eigen::VectorXd x = Eigen::VectorXd(n_x);
  x << 0, 0, 0, 0, 0;
  // create covariance matrix for prediction
  Eigen::MatrixXd P = Eigen::MatrixXd(n_x, n_x);
  P <<  0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0;

  /**
   * Student part begin
   */
  // Calculate mean
  for(int i = 0; i < Xsig_pred.cols(); i++){
    // set weights
    if ( i == 0){
    weights(i) = lambda/(lambda+n_aug);
    } else {
      weights(i) = 1/(2*(lambda+n_aug));
    }
    // predict state mean
    x += weights(i)*Xsig_pred.col(i);
  }
  
  Eigen::VectorXd x_diff; /* To check angle normalisation*/
  // predict state covariance matrix
  for(int i = 0; i < Xsig_pred.cols(); i++){
      x_diff = Xsig_pred.col(i) - x;
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
      P += weights(i)*(x_diff)*(x_diff.transpose());
  }
  /**
   * Student part end
   */

  // print result
  std::cout << "Predicted state" << std::endl;
  std::cout << x << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P << std::endl;

  // write result
  *x_out = x;
  *P_out = P;
}

void UKF::SigmaPointPrediction(Eigen::MatrixXd* Xsig_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // create example sigma point matrix
  Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  // create matrix with predicted sigma points as columns
  Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd(n_x, 2 * n_aug + 1);

  double delta_t = 0.1; // time diff in sec

  /**
   * Student part begin
   */

  Eigen::VectorXd deterministic_part = Eigen::VectorXd(n_x);
  Eigen::VectorXd stochastic_part = Eigen::VectorXd(n_x);
  Eigen::VectorXd base_part = Eigen::VectorXd(n_x);
  // predict sigma points
  double yaw, yaw_rate, px, py, v, vLong, vLang;
  
  for(int i=0; i < Xsig_aug.cols() ; i++){
    px = Xsig_aug(0,i);
    py = Xsig_aug(1,i);
    v = Xsig_aug(2,i);
    yaw = Xsig_aug(3,i);
    yaw_rate = Xsig_aug(4,i);
    
    vLong = Xsig_aug(5,i);
    vLang = Xsig_aug(6,i);
    
    base_part << px, 
                 py, 
                 v, 
                 yaw, 
                 yaw_rate;
     std::cout << "base part: " << base_part << "\n";
    
    stochastic_part << (0.5)*pow(delta_t,2)*cos(yaw)*vLong,
                       (0.5)*pow(delta_t,2)*sin(yaw)*vLong,
                       delta_t*vLong,
                       (0.5)*pow(delta_t,2)*vLang,
                       delta_t*vLang;
    std::cout << "stochastic part: " << stochastic_part << "\n";
    
    // avoid division by zero
    // Cutting corner (yawd != 0)
    if(abs(yaw_rate) > std::numeric_limits<double>::epsilon()){
        deterministic_part << (v/yaw_rate)*(sin(yaw + yaw_rate*delta_t)-sin(yaw)),
                              (v/yaw_rate)*(-cos(yaw + yaw_rate*delta_t)+cos(yaw)),
                              0,
                              yaw_rate*delta_t,
                              0;
    }    
    else{ /* Going straight (yawd == 0) */
        deterministic_part << v*cos(yaw)*delta_t,
                              v*sin(yaw)*delta_t,
                              0,
                              yaw_rate*delta_t,
                              0;
    }
    std::cout << "deterministic part: " << deterministic_part << "\n";
    
  // write predicted sigma points into right column
    Xsig_pred.col(i) = base_part + deterministic_part + stochastic_part;
  }
  

  /**
   * Student part end
   */

  // print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  // write result
  *Xsig_out = Xsig_pred;
}

void UKF::AugmentedSigmaPoints(Eigen::MatrixXd* Xsig_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 0.2;

  // define spreading parameter
  double lambda = 3 - n_aug;

  // set example state
  Eigen::VectorXd x = Eigen::VectorXd(n_x);
  x <<   5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;
  
  Eigen::MatrixXd Q = Eigen::MatrixXd(2,2);
  Q << pow(std_a,2), 0,
        0, pow(std_yawdd,2);
         
  // create example covariance matrix
  Eigen::MatrixXd P = Eigen::MatrixXd(n_x, n_x);
  P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  // create augmented mean vector
  Eigen::VectorXd x_aug = Eigen::VectorXd(7);

  // create augmented state covariance
  Eigen::MatrixXd P_aug = Eigen::MatrixXd(7, 7);

  // create sigma point matrix
  Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug, 2 * n_aug + 1);

  Eigen::MatrixXd zero_vector_1 = Eigen::MatrixXd(5,2);
  zero_vector_1 << 0, 0,
                 0, 0,
                 0, 0,
                 0, 0,
                 0, 0;
  
  Eigen::MatrixXd zero_vector_2 = Eigen::MatrixXd(2,5);
  zero_vector_2 = zero_vector_1.transpose();
  
  /**
   * Student part begin
   */

  // create augmented mean state
  x_aug << x, 
           0, 
           0 ;
  // create augmented covariance matrix
  P_aug << P , zero_vector_1,
           zero_vector_2, Q; 
  std::cout << "Augmented P: " << P_aug << "\n";
           
  // create square root matrix
  // Square root of P_aug
  Eigen::MatrixXd A = P_aug.llt().matrixL();
  std::cout << "Root of A:" << A << "\n";
  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for(int i = 1; i<=n_aug ; i++){
       Xsig_aug.col(i) = x_aug + sqrt(n_aug+lambda)*A.col(i-1);
       Xsig_aug.col(i+n_aug) = x_aug - sqrt(n_aug+lambda)*A.col(i-1);
   }
  /**
   * Student part end
   */

  // print result
  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  // write result
  *Xsig_out = Xsig_aug;
}

/**
 * Programming assignment functions: 
 */
void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {

  // set state dimension
  int n_x = 5;

  // define spreading parameter
  double lambda = 3 - n_x;

  // set example state
  VectorXd x = VectorXd(n_x);
  x <<   5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;

  // set example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  // create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

  // calculate square root of P
  MatrixXd A = P.llt().matrixL();

  /**
   * Student part begin
   */

  // your code goes here 
  // calculate sigma points ...
   Xsig.col(0) = x;
  // set sigma points as columns of matrix Xsig
   for(int i = 1; i<=n_x ; i++){
       Xsig.col(i) = x + sqrt(n_x+lambda)*A.col(i-1);
       Xsig.col(i+n_x) = x - sqrt(n_x+lambda)*A.col(i-1);
   }
  /**
   * Student part end
   */

  // print result
  std::cout << "Xsig = " << std::endl << Xsig << std::endl;

  // write result
  *Xsig_out = Xsig;
}

/**
 * expected result:
 * Xsig =
 *  5.7441  5.85768   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441
 *    1.38  1.34566  1.52806     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38
 *  2.2049  2.28414  2.24557  2.29582   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049
 *  0.5015  0.44339 0.631886 0.516923 0.595227   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015
 *  0.3528 0.299973 0.462123 0.376339  0.48417 0.418721 0.405627 0.243477 0.329261  0.22143 0.286879
 */