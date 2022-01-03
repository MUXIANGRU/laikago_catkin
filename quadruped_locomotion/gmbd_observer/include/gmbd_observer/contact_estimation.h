#ifndef CONTACT_ESTIMATION_H
#define CONTACT_ESTIMATION_H
#include<iostream>
#include "free_gait_msgs/RobotState.h"
#include "free_gait_core/executor/State.hpp"
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/free_gait_core.hpp"
#include "boost/thread.hpp"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/WrenchStamped.h"
#include <Eigen/Dense>
#include <Eigen/Eigen>
//#include "gmbd_observer.h"

using namespace std;
using namespace free_gait;
class ContactEstimation{
struct phase{
  double swing_phase;
  double stance_phase;
  bool stance_status;
  bool swing_status;
  bool ready_to_swing;
  bool real_contact;
};
typedef std::unordered_map<free_gait::LimbEnum, phase, EnumClassHash> LimbPhase;
public:
ContactEstimation(const ros::NodeHandle& node_handle);
~ContactEstimation();
double computeContactPro(double phase);
double computeHeightPro(double h);
double computeForcePro(double f);
bool updateEstimationState(int b);
void  init();
void  Predict(Eigen::Matrix<double,4,4>& A,Eigen::Matrix<double,4,4>& B,Eigen::Matrix<double,4,1>& x,Eigen::Matrix<double,4,1>& u);
void  Update(Eigen::Matrix<double,8,1> &z);
void  ContactPublisherThread();
void footposCB(const std_msgs::Float64MultiArray::ConstPtr& msg);
void lf_forceCB(const geometry_msgs::WrenchStamped::ConstPtr& msg);
void rf_forceCB(const geometry_msgs::WrenchStamped::ConstPtr& msg);
void lh_forceCB(const geometry_msgs::WrenchStamped::ConstPtr& msg);
void rh_forceCB(const geometry_msgs::WrenchStamped::ConstPtr& msg);
void robot_phaseCB(const free_gait_msgs::RobotState::ConstPtr& msg);
double p_c_lf,p_c_rf,p_c_lh,p_c_rh;
double p_h_lf,p_h_rf,p_h_lh,p_h_rh;
double p_f_lf,p_f_rf,p_f_lh,p_f_rh;
double lf_height,rf_height,lh_height,rh_height;
double lf_phase,rf_phase,lh_phase,rh_phase;
Eigen::Matrix<double,4,1> x_state_pre;
std_msgs::Float64MultiArray contact_prob;
private:
    ros::NodeHandle nodeHandle_;
    std::shared_ptr<free_gait::State> robot_state_;
    boost::recursive_mutex r_mutex;
    ros::Subscriber foot_pos_sub,lf_force_est_sub,rf_force_est_sub,lh_force_est_sub,rh_force_est_sub,
    robot_phase_sub;
    ros::Publisher contact_pro_pub;
    int s_phi;
    int s_phi_;
    double phase;
    double phase_;
    double mu_0;
    double mu_0_;
    double mu_1;
    double mu_1_;
    double mu_z;
    double mu_f;
    double sigma_0;
    double sigma_1;
    double sigma_0_;
    double sigma_1_;
    double sigma_z;
    double sigma_f;
    double h;
    double t_swing_,t_stance_;
    Eigen::Matrix<double,4,4> P_pre,Q_pre;
    Eigen::Matrix<double,8,4> H_mea;
    Eigen::Matrix<double,1,2> H_t;
    Eigen::Matrix<double,8,8> R_;
    LimbPhase limb_phase;
    boost::thread Contactpublisherthread;
    int log_length_, log_index_;
    std::vector<free_gait::LimbEnum> limbs_;
    std::vector<free_gait::BranchEnum> branches_;


};
#endif // CONTACT_ESTIMATION_H
