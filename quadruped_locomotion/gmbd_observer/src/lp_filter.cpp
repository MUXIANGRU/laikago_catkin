#include "gmbd_observer/lp_filter.h"
lpFilter::lpFilter() {
}

lpFilter::~lpFilter() {
}
//void KalmanFilter::Init(){
//    A_.setZero();
//    B_.setZero();
//    H_.setIdentity();

//}
//void KalmanFilter::Predict(MatrixXd U) {
//    U_ = U;
//    x_ = A_ * x_ + B_ * U_;
//    MatrixXd Ft = A_.transpose();
//    P_ = A_ * P_ * Ft + Q_;
//}

void lpFilter::Update(VectorXd &current_state, const VectorXd &current_input,
                      const double alpha,const double frequency){

    //MXR::NOTE: how to compute alpha
    float dt = 0.02;
    float b = 2 * float(M_PI) * frequency*dt;
    float a = b / (1 + b);

    current_state = alpha*current_state+(1-alpha)*current_input;
}
