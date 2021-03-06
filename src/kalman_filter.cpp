#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define MIN_SENSOR_VALUE    0.000001

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    // processing nonsense data (both data are too small)
    if (x_(0)<=min_sensor_value_ && x_(1)<=min_sensor_value_)
    {
        return;
    }

    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    // processing nonsense data (both data are too small)
    if (x_(0)<=min_sensor_value_ && x_(1)<=min_sensor_value_)
    {
        return;
    }
    x_(0) = (x_(0)<=MIN_SENSOR_VALUE) ? MIN_SENSOR_VALUE : x_(0);

    VectorXd z_pred = VectorXd(3);
    float px=x_(0);
    float py=x_(1);
    float vx=x_(2);
    float vy=x_(3);
    z_pred(0) = sqrt(px*px+py*py); // This operation is safe since px and py have been protected by min_sensor_value_
    z_pred(1) = atan2(py,px); // This operation is safe since px is set to be >=min_sensor_value_
    z_pred(2) = (px*vx+py*vy)/z_pred(0); // This operation is safe since px and py have been protected by min_sensor_value_
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
