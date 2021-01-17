#include "ukf.h"
#include "Eigen/Dense"
#include <chrono>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  
  // Debug 
  enable_debug_ = false;
  // State space dimension
  // x = [px py v yaw yaw_rate]
  n_x_ = 5;

  // Augmented state space dimension
  // z = [x var_acc var_yaw_acc]
  n_aug_ = 7;

  // spreading parameter to generate sigma points
  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = Eigen::VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Initialise covariance to identity
//  P_ = Eigen::MatrixXd::Identity(n_x_,n_x_);
  P_ = MatrixXd::Identity(n_x_,n_x_);
  
  // Initialise sigma points prediction matrix
  Xsig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);
  
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.0 ;

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // set weights
  weights_ = Eigen::VectorXd(2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    if (i == 0)
    {
      weights_(i) = lambda_ / (lambda_ + n_aug_);
    }
    else
    {
      weights_(i) = (0.5) / (lambda_ + n_aug_);
    }
  }

  is_initialized_ = false;
  are_sigma_points_generated_ = false;

  // Initialise nis stats
  nis_lidar_ = Eigen::VectorXd(1);
  nis_lidar_ << 0;

  nis_radar_ = Eigen::VectorXd(1);
  nis_radar_ << 0;
  
  time_us_ = 0;
  /**
 * End DO NOT MODIFY section for measurement noise values 
 */

  /**
 * TODO: Complete the initialization. See ukf.h for other member properties.
 * Hint: one or more values initialized above might be wildly off...
 */
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if(enable_debug_)
    std::cout << "Is it initialised ?? : " << is_initialized_ << "\n";

  if (!is_initialized_)
  {
    if(enable_debug_)
      std::cout << meas_package.sensor_type_ << "\n";
    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) /* Lidar Measurement*/
    {
      if(enable_debug_)
        std::cout << "State vector initialised by lidar ... \n";

      // Map [px, py] to [px, py, v, yaw, yaw_rate]
      // set all of the velocity related states to zero
      x_ << meas_package.raw_measurements_[0],
          meas_package.raw_measurements_[1],
          0,
          0, //atan2(meas_package.raw_measurements_[1],meas_package.raw_measurements_[0]),
          0;
      if(enable_debug_){
        std::cout << "State vector is \n"
                << x_ << "\n";
      }
      is_initialized_ = true;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
    {
      if(enable_debug_)
        std::cout << "State vector initialised by radar ... \n";
      // Map [p, phi, p_dot] to [px, py, v, yaw, yaw_rate]
      // NOTE: p_dot to v mapping is a bit dodgy
      // v = p_dot*cos(yaw), it is still not a perfect way of initialising CTRV velocity
      // but it should be good enough to start the UKF steps
      if(enable_debug_)
        std::cout << "meas_package: " << meas_package.raw_measurements_ << "\n";
      double vx = meas_package.raw_measurements_[2] * cos(meas_package.raw_measurements_[1]);
      double vy = meas_package.raw_measurements_[2] * sin(meas_package.raw_measurements_[1]);
      double v = sqrt(vx * vx + vy * vy);

      x_ << meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]),
          meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]),
          0,
          0,
          0;

      if(enable_debug_){
        std::cout << "State vector is \n"
                << x_ << "\n";
      }
      is_initialized_ = true;
    }

    time_us_ = meas_package.timestamp_;
    return;
  }
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  
  std::cout << "## Prediction ## \n";
  Prediction(delta_t);
  
  std::cout << " delta_t : " << delta_t << "\n";
  time_us_ = meas_package.timestamp_;
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  {
    std::cout << "## RADAR Update ## \n";

    auto startTime = std::chrono::steady_clock::now();
    
    UpdateRadar(meas_package);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "The radar update took " << elapsedTime.count()  << " milliseconds. Mean and covariance are: \n " 
              << x_ << "\n" << P_ << std::endl;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
  {
    std::cout << "## LIDAR Update ## \n";
    
    auto startTime = std::chrono::steady_clock::now();

    UpdateLidar(meas_package);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << " The lidar update took " << elapsedTime.count() << " milliseconds. Mean and covariance are: \n "
              << x_ << "\n" << P_ << std::endl;
  }
}

void UKF::Prediction(double delta_t)
{
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  /* std::cout << "#### Start to Prediction step ####\n";  
  // ########
  auto startTime = std::chrono::steady_clock::now();

  Eigen::MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);
  Xsig.fill(0.0);
  // Generate the sigma points
  GenerateSigmaPoints(&Xsig);
  
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "Generating sigma points took " << elapsedTime.count() << " milliseconds. Found sigma points\n " << Xsig << std::endl;
  // ########
*/

  // ########
  auto startTime = std::chrono::steady_clock::now();

  // AugmentedSigmaPoints
  Eigen::MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.fill(0.0);
  AugmentedSigmaPoints(&Xsig_aug);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  if(enable_debug_)
    std::cout << "Augmenting sigma points took " << elapsedTime.count() << " milliseconds. Augmented sigma points\n " << Xsig_aug << std::endl;
  // ########

  // ########
  startTime = std::chrono::steady_clock::now();

  // Predict sigma points
  SigmaPointPrediction(delta_t, Xsig_aug);

  endTime = std::chrono::steady_clock::now();
  elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  if(enable_debug_)
    std::cout << "Predicting sigma points took " << elapsedTime.count() << " milliseconds. Predicted sigma points\n " << Xsig_pred_ << std::endl;
  // ########

  // ########
  startTime = std::chrono::steady_clock::now();

  // Calculate sigma points mean&covariance
  PredictMeanAndCovariance();
  
  endTime = std::chrono::steady_clock::now();
  elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

  std::cout << "Predicting mean and covariance of sigma points took " << elapsedTime.count() << " milliseconds. Mean and covariance are: \n " << x_ << "\n"
            << P_ << std::endl;

  // ########
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  /* 
  * The measurement of the lidar does have the structure:
  *  z = [px py] which is first two elements of state space of CTRV
  * So that the calculated mean, covariance from transformed sigma 
  * points can be used directly
  */

  // Predicted mean&covariance of generated sigma points in lidar measurement space which is subspace of state vector
  // Lidar does measure two component
  // z = [px py]
  if(enable_debug_){
    std::cout << "Lidar update \n";
    std::cout << "State vector before lidar correction \n"
              << x_ << "\n";
    std::cout << "State covariance before lidar correction \n"
              << P_ << "\n";
  }
   int n_z = 2;

/*
  // Initialise measurement matrix
  Eigen::MatrixXd H = Eigen::MatrixXd(n_z, n_x_);
  H << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0;
*/
  Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2*n_aug_ +1);
  // Subset of state space is the lidar space 
  if(enable_debug_)
    std::cout << "Xsig is : \n" << Xsig_pred_ << "\n";
  
  Zsig = Xsig_pred_.block(0,0,n_z,2*n_aug_ +1);
  if(enable_debug_)
    std::cout << "Zsig is : \n" << Zsig << "\n";
  
  Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);
  z_pred.fill(0.0);
  
  for(int i = 0; i < Zsig.cols() ; i++ ){
    z_pred += weights_(i)* Zsig.col(i);
  }
  
  Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  
  // calculate cross correlation matrix
  Eigen::VectorXd x_diff = Eigen::VectorXd(n_x_);
  Eigen::VectorXd z_diff = Eigen::VectorXd(n_z);
  
  // Measurement covariance matrix
  Eigen::MatrixXd S = Eigen::MatrixXd(n_z, n_z);
  S.fill(0.0);
  
  // Measurement matrix
  Eigen::MatrixXd R = Eigen::MatrixXd(n_z, n_z);
  R << pow(std_laspx_, 2), 0,
      0, pow(std_laspy_, 2);
      
  for(int i = 0; i < Zsig.cols() ; i++){
      x_diff = Xsig_pred_.col(i) - x_;
      z_diff = Zsig.col(i) - z_pred;
      
      // angle normalization
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
      
      Tc += weights_(i)*x_diff*(z_diff.transpose());
      S += weights_(i) *z_diff*(z_diff.transpose());
  }
  S += R;

  

  /*
  // Measurement covariance matrix
  Eigen::MatrixXd S = Eigen::MatrixXd(n_z, n_z);
  S.fill(0.0);
  for(int i = 0; i < Zsig.cols() ; i++){
    z_diff  = Zsig.col(i) - z_pred;
    
    S += weights_(i) *z_diff*(z_diff.transpose());
  }
  std::cout << "lidar S ... \n" << S << "\n";
  S += R;
    std::cout << "lidar S ... \n" << S << "\n";
  */

  // z measurement matrix
  Eigen::VectorXd z = Eigen::VectorXd(n_z);
  z << meas_package.raw_measurements_[0],
      meas_package.raw_measurements_[1];
  if(enable_debug_){
    std::cout << "Lidar measurement is: \n"
            << z << "\n";
  }
  
  // Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);
  // z_pred = H * x_;
  
  // Calculate the residual
  z_diff = z - z_pred;
  
  // calculate Kalman gain K
  
  Eigen::MatrixXd K = Eigen::MatrixXd(n_x_, n_z);
 
  K.fill(0.0);
  K = Tc*S.inverse();
 /*
  Eigen::VectorXd y = z - z_pred;
  Eigen::MatrixXd Ht = H.transpose();
  Eigen::MatrixXd S = H * P_ * Ht + R;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd PHt = P_ * Ht;
  Eigen::MatrixXd K = PHt * Si;

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_x_, n_x_);
*/
  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;

  // angle normalization
  while (x_(3) > M_PI)
    x_(3) -= 2. * M_PI;
  while (x_(3) < -M_PI)
    x_(3) += 2. * M_PI;

  //P_ = (I - K * H) * P_;
  P_ -= K*S*(K.transpose());

  // Initialise the vector
  nis_lidar_ = z_diff.transpose() * (S.inverse()) * z_diff;
  std::cout << "NIS of lidar: " << nis_lidar_ << "\n";
  
  // Calculate nis for lidar
  if(enable_debug_){
    std::cout << "State vector after lidar correction \n"
              << x_ << "\n";
    std::cout << "State covariance after lidar correction \n"
              << P_ << "\n";
  }
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  if(enable_debug_){
    std::cout << "radar update \n";
    std::cout << "State vector before radar correction \n"
              << x_ << "\n";
    std::cout << "State covariance before radar correction \n"
              << P_ << "\n";
  }
  // Predicted mean&covariance of generated sigma points in radar measurement space
  // Radar does measure three component
  // z = [ro phi phi_dot]
  int n_z = 3;

  // Mean
  Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);
  // Covariance
  Eigen::MatrixXd S = Eigen::MatrixXd(n_z, n_z);
  
  // Sigma points in Radar space
  Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2 * n_aug_ + 1);
  
  if(enable_debug_)
    std::cout << "transform radar measurements in \n";
  
  // TransformToRadarMeasurement(&z_pred, &S, &Zsig);
  
  //************************
  Eigen::MatrixXd R = Eigen::MatrixXd(n_z, n_z);
  R << pow(std_radr_, 2), 0, 0,
      0, pow(std_radphi_, 2), 0,
      0, 0, pow(std_radrd_, 2);

  // Reset z_pred and S
  z_pred.fill(0.0);
  S.fill(0.0);

  // Reset the sigma points in radar space matrix
  Zsig.fill(0.0);

  double px, py, v, yaw, yaw_rate, px2, py2, yaw_estimate, theta;
  
  // transform sigma points into measurement space
  for (int i = 0; i < Xsig_pred_.cols(); i++)
  {
    px = Xsig_pred_(0, i);
    py = Xsig_pred_(1, i);
    v = Xsig_pred_(2, i);
    yaw = Xsig_pred_(3, i);

    px2 = pow(px, 2);
    py2 = pow(py, 2);

    yaw_estimate = atan2(py, px);
    
    // angle normalization
    while (yaw_estimate > M_PI)
      yaw_estimate -= 2. * M_PI;
    while (yaw_estimate < -M_PI)
      yaw_estimate += 2. * M_PI;

    Zsig.col(i) << sqrt(px2 + py2),
                   yaw_estimate,
                   v*cos(yaw)/cos(yaw_estimate);
                   //v*(px*cos(yaw)+py*sin(yaw))/sqrt(px2+py2);
                   
  }
  
  // calculate mean predicted measurement
  for (int i = 0; i < Zsig.cols(); i++)
  {
    // predict state mean
    z_pred += weights_(i) * Zsig.col(i);
  }
  if(enable_debug_){
  std::cout << "radar_sig pred : \n"
            << (z_pred) << "\n";
  }
  
  // calculate innovation covariance matrix S
  Eigen::VectorXd z_diff; /* To check angle normalisation*/

  // predict state covariance matrix
  for (int i = 0; i < Zsig.cols(); i++)
  {
    z_diff = Zsig.col(i) - (z_pred);

    // angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;

    (S) += weights_(i) * (z_diff) * (z_diff.transpose());
  }
  (S) += R;
  
  //************************
  
  if(enable_debug_){
    std::cout << "z_pred: \n"
              << z_pred << "\n";
    std::cout << "radar covariance \n"
              << S << "\n";
    std::cout << "z sigma points \n"
              << Zsig << "\n";

    std::cout << "transform radar measurements out \n";
  }
  
  // ####################
  // create matrix for cross correlation Tc
  Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z);
  Tc.fill(0.0);

  // calculate cross correlation matrix
  Eigen::VectorXd x_diff = Eigen::VectorXd(n_x_);
  z_diff.fill(0.0);
  for (int i = 0; i < Zsig.cols(); i++)
  {
    x_diff = Xsig_pred_.col(i) - x_;
    z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;

    // angle normalization
    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;

    Tc += weights_(i) * x_diff * (z_diff.transpose());
  }

  // initialise current measurement
  Eigen::VectorXd z = Eigen::VectorXd(n_z);
  z << meas_package.raw_measurements_[0],
       meas_package.raw_measurements_[1],
       meas_package.raw_measurements_[2];
  if(enable_debug_){
    std::cout << "Radar measurement is: \n"
              << z << "\n";
  }
  while (z(1) > M_PI)
    z(1) -= 2. * M_PI;
  while (z(1) < -M_PI)
    z(1) += 2. * M_PI;

  // calculate Kalman gain K;
  Eigen::MatrixXd K = Eigen::MatrixXd(n_x_, n_z);

  K = Tc * S.inverse();
  // update state mean and covariance matrix
  x_ = x_ + K * (z - z_pred);
  // angle normalization
  while (x_(3) > M_PI)
    x_(3) -= 2. * M_PI;
  while (x_(3) < -M_PI)
    x_(3) += 2. * M_PI;
    
  P_ = P_ - K * S * (K.transpose());

  // Initialise the vector
  nis_radar_ = ((z_pred - z).transpose()) * (S.inverse()) * (z_pred - z);
  std::cout << "NIS of radar: " << nis_radar_ << "\n";
  
  if(enable_debug_){
    std::cout << "update radar measurements out \n";
    std::cout << "State vector after radar correction \n"
              << x_ << "\n";
    std::cout << "State covariance after radar correction \n"
              << P_ << "\n";
  }
}

void UKF::TransformToRadarMeasurement(Eigen::VectorXd *z_pred, Eigen::MatrixXd *S, Eigen::MatrixXd *Zsig)
{
  Eigen::MatrixXd R = Eigen::MatrixXd(z_pred->rows(), z_pred->rows());
  R << pow(std_radr_, 2), 0, 0,
      0, pow(std_radphi_, 2), 0,
      0, 0, pow(std_radrd_, 2);

  // Reset z_pred and S
  z_pred->fill(0.0);

  // Reset the sigma points in radar space matrix
  Zsig->fill(0.0);

  double px, py, v, yaw, yaw_rate, px2, py2;
  // transform sigma points into measurement space
  for (int i = 0; i < Xsig_pred_.cols(); i++)
  {
    px = Xsig_pred_(0, i);
    py = Xsig_pred_(1, i);
    v = Xsig_pred_(2, i);
    yaw = Xsig_pred_(3, i);
    yaw_rate = Xsig_pred_(4, i);

    px2 = pow(px, 2);
    py2 = pow(py, 2);

    Zsig->col(i) << sqrt(px2 + py2),
        atan2(py, px),
        v * (px * cos(yaw) + py * sin(yaw)) / float(sqrt(px2 + py2));
  }

  // calculate mean predicted measurement
  for (int i = 0; i < Zsig->cols(); i++)
  {
    // predict state mean
    (*z_pred) += weights_(i) * Zsig->col(i);
  }
  std::cout << "radar_sig pred : \n"
            << (*z_pred) << "\n";

  // calculate innovation covariance matrix S
  Eigen::VectorXd z_diff; /* To check angle normalisation*/
  // predict state covariance matrix
  for (int i = 0; i < Zsig->cols(); i++)
  {
    z_diff = Zsig->col(i) - (*z_pred);

    // angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;

    (*S) = (*S) + weights_(i) * (z_diff) * (z_diff.transpose());
  }
  (*S) = (*S) + R;
}
void UKF::GenerateSigmaPoints(Eigen::MatrixXd *Xsig)
{
  // Square root of P
  Eigen::MatrixXd A = P_.llt().matrixL();

  // Generate sigma points
  Xsig->col(0) = x_;

  for (int i = 1; i <= n_x_; i++)
  {
    Xsig->col(i) = x_ + sqrt(n_x_ + lambda_) * A.col(i - 1);
    Xsig->col(i + n_x_) = x_ - sqrt(n_x_ + lambda_) * A.col(i - 1);
  }
}

void UKF::AugmentedSigmaPoints(Eigen::MatrixXd *Xsig_aug)
{

  // Augmented state vector
  Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_);
  x_aug << x_,
      0,
      0;

  // Augmented Process covariance matrix
  Eigen::MatrixXd P_aug = Eigen::MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = pow(std_a_, 2);
  P_aug(6, 6) = pow(std_yawdd_, 2);

  // Square root of P_aug
  Eigen::MatrixXd A = P_aug.llt().matrixL();

  // Generate augmented sigma points
  Xsig_aug->col(0) = x_aug;

  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug->col(i+1) = x_aug + sqrt(n_aug_ + lambda_) * A.col(i);
    Xsig_aug->col(i + 1 + n_aug_) = x_aug - sqrt(n_aug_ + lambda_) * A.col(i);
  }
}

void UKF::SigmaPointPrediction(double delta_t, Eigen::MatrixXd Xsig_aug)
{
  // Initialise part of the equations
  Eigen::VectorXd deterministic_part = Eigen::VectorXd(n_x_);
  Eigen::VectorXd stochastic_part = Eigen::VectorXd(n_x_);
  Eigen::VectorXd base_part = Eigen::VectorXd(n_x_);

  // Define the state elements
  double px, py, v, yaw, yaw_rate, var_acc, var_yaw_acc;

  // predict sigma points
  for (int i = 0; i < Xsig_pred_.cols(); i++)
  {
    px = Xsig_aug(0, i);
    py = Xsig_aug(1, i);
    v = Xsig_aug(2, i);
    yaw = Xsig_aug(3, i);
    yaw_rate = Xsig_aug(4, i);

    var_acc = Xsig_aug(5, i);
    var_yaw_acc = Xsig_aug(6, i);

    base_part << px,
        py,
        v,
        yaw,
        yaw_rate;

    stochastic_part << (0.5) * pow(delta_t, 2) * cos(yaw) * var_acc,
        (0.5) * pow(delta_t, 2) * sin(yaw) * var_acc,
        delta_t * var_acc,
        (0.5) * pow(delta_t, 2) * var_yaw_acc,
        delta_t * var_yaw_acc;

    // avoid division by zero
    // Cutting corner (yawd != 0)
    if (fabs(yaw_rate) > 0.001) 
    //(abs(yaw_rate) > std::numeric_limits<double>::epsilon())
    { //pow(std_yawdd_, 2)){
      deterministic_part << (v / yaw_rate) * (sin(yaw + yaw_rate * delta_t) - sin(yaw)),
                            (v / yaw_rate) * (-cos(yaw + yaw_rate * delta_t) + cos(yaw)),
                            0,
                            yaw_rate * delta_t,
                            0;
    }
    else
    { /* Going straight (yawd == 0) */
      deterministic_part << v * cos(yaw) * delta_t,
          v * sin(yaw) * delta_t,
          0,
          yaw_rate * delta_t,
          0;
    }

    // write predicted sigma points into right column
    Xsig_pred_.col(i) = base_part + deterministic_part + stochastic_part;
  }
}

void UKF::PredictMeanAndCovariance()
{
  // Reset prior to calculate predicted state
  Eigen::VectorXd x_pred = Eigen::VectorXd(n_x_);
  Eigen::MatrixXd P_pred = Eigen::MatrixXd(n_x_, n_x_);
  x_pred.fill(0.0);
  P_pred.fill(0.0);
  for (int i = 0; i < (2*n_aug_ + 1); i++)
  {
    // predict state mean
    x_pred += weights_(i) * Xsig_pred_.col(i);
  }
  // Reset prior covariance to calculate predicted covariance matrix
  Eigen::VectorXd x_diff; /* To check angle normalisation*/
  // predict state covariance matrix
  for (int i = 0; i < Xsig_pred_.cols(); i++)
  {
    x_diff = Xsig_pred_.col(i) - x_pred;
    
    while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;
    
    P_pred += weights_(i) * (x_diff) * (x_diff.transpose());
  }
  x_ = x_pred;
  while (x_(3) > M_PI) x_(3) -= 2. * M_PI;
  while (x_(3) < -M_PI) x_(3) += 2. * M_PI;
  P_ = P_pred;
}
