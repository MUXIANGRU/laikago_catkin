#include "gmbd_observer/kalman_filter.h"
KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}
void KalmanFilter::Init(){
    A_.setZero();
    B_.setZero();
    H_.setIdentity();

}
void KalmanFilter::Predict(MatrixXd U) {
    U_ = U;
    x_ = A_ * x_ + B_ * U_;
    MatrixXd Ft = A_.transpose();
    P_ = A_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
