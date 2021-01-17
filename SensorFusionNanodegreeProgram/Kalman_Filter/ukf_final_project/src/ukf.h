#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"
#include <iostream>

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /**
   * Generates the sigma points to do unscented transformations
   * @param Xsig Generated sigma points
   */
  void GenerateSigmaPoints(Eigen::MatrixXd *Xsig);
  
  /**
   * Augmentes the sigma points be able to combine process noise with state space
   * @param Xsig Generated sigma points
   * @param Xsig_aug Augmented sigma points
   */
  void AugmentedSigmaPoints(Eigen::MatrixXd *Xsig_aug);
  
  /**
   * Predicts the new sigma points based on CTRV model
   * @param Xsig_aug Augmented sigma points
   * @param delta_t prediction time window (time ahead in the future to predict)
   */
  void SigmaPointPrediction(double delta_t, Eigen::MatrixXd Xsig_aug );
  
  /**
   * Predicts mean&covariance of latest predicted sigma points
   */
  void PredictMeanAndCovariance();
  
  /**
   * Applies unscented transform to predicted covariance&mean of sigma points
   * to radar measurement space to be able to do update with the sensor
   * @param radar_sig_pred mean of sigma points in radar measurement space 
   * @param S covariance of sigma points in radar measurement space
   * @param Zsig sigma points in radar measurement space
   */
  void TransformToRadarMeasurement(Eigen::VectorXd *radar_sig_pred, Eigen::MatrixXd *S, Eigen::MatrixXd *Zsig);
  
  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // Enable debugging 
  bool enable_debug_;
  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;
  
  // Initially set to false, it will be set to true once the sigma points generation has been done
  bool are_sigma_points_generated_;
  
  public:
  // NIS (Normalised innovation Squared) stat of radar 
  Eigen::VectorXd nis_radar_;
  
  // NIS (Normalised innovation Squared) stat of radar 
  Eigen::VectorXd nis_lidar_;
};

#endif  // UKF_H