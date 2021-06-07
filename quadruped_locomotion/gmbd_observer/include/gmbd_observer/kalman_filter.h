#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:

    ///* state vector
    VectorXd x_;

    ///* state covariance matrix
    MatrixXd P_;

    ///* state transistion matrix
    MatrixXd A_;

    ///* process covariance matrix
    MatrixXd Q_;

    ///* input control matrix
    MatrixXd B_;

    ///* input control vector
    MatrixXd U_;

    ///* measurement matrix
    MatrixXd H_;

    ///* measurement covariance matrix
    MatrixXd R_;

    /**
     * Constructor
     */
    KalmanFilter();

    /**
     * Destructor
     */
    virtual ~KalmanFilter();

    /**
     * Predict Predicts the state and the state covariance
     * using the process model
     */
    void Predict(MatrixXd U);

    /**
     * Updates the state and
     * @param z The measurement at k+1
     */
    void Init();
    void Update(const VectorXd &z);

};
#endif // KALMAN_FILTER_H
