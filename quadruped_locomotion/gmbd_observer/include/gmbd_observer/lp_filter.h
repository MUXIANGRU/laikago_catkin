#ifndef LP_FILTER_H
#define LP_FILTER_H
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class lpFilter {
public:
//
//     ///* state vector
//     VectorXd x_;
//
//     ///* state covariance matrix
//     MatrixXd P_;
//
//     ///* state transistion matrix
//     MatrixXd A_;
//
//     ///* process covariance matrix
//     MatrixXd Q_;
//
//     ///* input control matrix
//     MatrixXd B_;
//
//     ///* input control vector
//     MatrixXd U_;
//
//     ///* measurement matrix
//     MatrixXd H_;
//
//     ///* measurement covariance matrix
//     MatrixXd R_;
//
     /**
      * Constructor
      */
     lpFilter();

     /**
      * Destructor
      */
     virtual ~lpFilter();
//
//     /**
//      * Predict Predicts the state and the state covariance
//      * using the process model
//      */
//     void Predict(MatrixXd U);
//
//     /**
//      * Updates the state and
//      * @param z The measurement at k+1
//      */
//     void Init();
//     bool getState(const VectorXd &current_state){
//         if(current_state.x()!=0){
//             return true;
//         }
//         return false;
//     }
     void Update(VectorXd &current_state,const VectorXd &current_input,
                 const double alpha,const double frequency);
//
};
#endif // KALMAN_FILTER_H
