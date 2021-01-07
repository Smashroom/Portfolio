#include "tracking.h"
#include <iostream>
#include "Eigen/Dense"


Tracking::Tracking() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // create a 4D state vector, we don't know yet the values of the x state
  kf_.x_ = Eigen::VectorXd(4);

  // state covariance matrix P
  kf_.P_ = Eigen::MatrixXd(4, 4);
  kf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;


  // measurement covariance
  kf_.R_ = Eigen::MatrixXd(2, 2);
  kf_.R_ << 0.0225, 0,
            0, 0.0225;

  // measurement matrix
  kf_.H_ = Eigen::MatrixXd(2, 4);
  kf_.H_ << 1, 0, 0, 0,
            0, 1, 0, 0;

  // the initial transition matrix F_
  kf_.F_ = Eigen::MatrixXd(4, 4);
  kf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

  // set the acceleration noise components
  noise_ax = 5;
  noise_ay = 5;
  kf_.Qnu_ = Eigen::MatrixXd(2,2);
  kf_.Qnu_ << noise_ax, 0,
              0, noise_ay;
              
  kf_.G_ = Eigen::MatrixXd(4,2);
}

Tracking::~Tracking() {

}

// Process a single measurement
void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  if (!is_initialized_) {

    // set the state with the initial location and zero velocity
    kf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;

    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }
  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // TODO: YOUR CODE HERE
  // 1. Modify the F matrix so that the time is integrated
  kf_.F_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
  
   
  // 2. Set the process covariance matrix Q
  // Set the G matrix;
  
  kf_.G_ << pow(dt,2)/2, 0,
            0, pow(dt,2)/2,
            dt, 0,
            0, dt;

  kf_.Q_ = kf_.G_*kf_.Qnu_*(kf_.G_.transpose());

  // 3. Call the Kalman Filter predict() function
  kf_.Predict();
  // 4. Call the Kalman Filter update() function
  //      with the most recent raw measurements_
  kf_.Update(measurement_pack.raw_measurements_);
  
  std::cout << "x_= " << kf_.x_ << std::endl;
  std::cout << "P_= " << kf_.P_ << std::endl;
}