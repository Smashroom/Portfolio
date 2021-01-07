#include "kalman_filter.h"

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  Eigen::MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const Eigen::VectorXd &z) {
  Eigen::VectorXd z_pred = H_ * x_;
  Eigen::VectorXd y = z - z_pred;
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd S = H_ * P_ * Ht + R_;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd PHt = P_ * Ht;
  Eigen::MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
Eigen::VectorXd KalmanFilter::CalculateRMSE(const vector<VectorXd> &estimations,
    const vector<VectorXd> &ground_truth) {

  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  VectorXd tempRmse(4);
  tempRmse << 0,0,0,0;
  
  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() < 1){
      std::cout << "Error - No estimations\n";
      return rmse;
  }
  
  if(estimations.size() != ground_truth.size()){
      std::cout << "Error - Ground truth and estimations are not same size \n";
      return rmse;
  }
  
  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    tempRmse = (ground_truth[i] - estimations[i]);
    rmse = rmse + tempRmse.cwiseProduct(tempRmse);
    
  }
    
  // TODO: calculate the mean
  rmse = rmse.array()/estimations.size();
  // TODO: calculate the squared root
  rmse = rmse.array().sqrt();
  // return the result
  return rmse;
}


Eigen::MatrixXd KalmanFilter::CalculateJacobian(const Eigen::VectorXd x){
    MatrixXd Hj(3,4);
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    if ((px == 0) || (py == 0)){
        std::cout << "Error: Px or Py is zero \n";
        return MatrixXd;
    }
    
    float px2 = pow(px,2);
    float py2 = pow(py,2);
    
    auto h11 = px/sqrt(px2+py2);
    auto h12 = py/sqrt(px2+py2);
    
    auto h21 = -py/(px2+py2);
    auto h22 = px/(px2+py2); 
    
    auto h31 = py*(vx*py-vy*px)/sqrt(pow(px2+py2,3));
    auto h32 = px*(vy*px-vx*py)/sqrt(pow(px2+py2,3));
    auto h33 = px/sqrt(px2+py2);
    auto h34 = py/sqrt(px2+py2);
    
    Hj << h11, h12, 0,   0,
          h21, h22, 0,   0,
          h31, h32, h33, h34;

    return Hj;
}