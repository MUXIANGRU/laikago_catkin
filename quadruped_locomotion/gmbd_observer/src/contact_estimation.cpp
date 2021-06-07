#include "gmbd_observer/contact_estimation.h"
#include "math.h"
#include "iostream"
#include "ros/ros.h"


ContactEstimation::ContactEstimation(const ros::NodeHandle& node_handle)
    : nodeHandle_(node_handle)
    {
//    log_length_ = 10000;
//    log_index_ = log_length_;
//    limbs_.push_back(free_gait::LimbEnum::LF_LEG);
//    limbs_.push_back(free_gait::LimbEnum::RF_LEG);
//    limbs_.push_back(free_gait::LimbEnum::LH_LEG);
//    limbs_.push_back(free_gait::LimbEnum::RH_LEG);
//    branches_.push_back(free_gait::BranchEnum::BASE);
//    branches_.push_back(free_gait::BranchEnum::LF_LEG);
//    branches_.push_back(free_gait::BranchEnum::RF_LEG);
//    branches_.push_back(free_gait::BranchEnum::LH_LEG);
//    branches_.push_back(free_gait::BranchEnum::RH_LEG);
//    //! WSHY: initialize STAte
//    robot_state_.reset(new free_gait::State);
//    robot_state_->initialize(limbs_, branches_);
        init();
        contact_prob.data.resize(4);
        x_state_pre.resize(4);
        foot_pos_sub = nodeHandle_.subscribe("/laikago_pronto/foot_position_in_world",1,&ContactEstimation::footposCB,this);
        lf_force_est_sub = nodeHandle_.subscribe("/estimate_torque_lf",1,&ContactEstimation::lf_forceCB,this);
        rf_force_est_sub = nodeHandle_.subscribe("/estimate_torque_rf",1,&ContactEstimation::rf_forceCB,this);
        lh_force_est_sub = nodeHandle_.subscribe("/estimate_torque_lh",1,&ContactEstimation::lh_forceCB,this);
        rh_force_est_sub = nodeHandle_.subscribe("/estimate_torque_rh",1,&ContactEstimation::rh_forceCB,this);
        //free_gait_msgs::RobotState
        robot_phase_sub = nodeHandle_.subscribe("/desired_robot_state",1,&ContactEstimation::robot_phaseCB,this);
        contact_pro_pub = nodeHandle_.advertise<std_msgs::Float64MultiArray>("/contact_pro",1);
        Contactpublisherthread = boost::thread(boost::bind(&ContactEstimation::ContactPublisherThread, this));
    }
ContactEstimation::~ContactEstimation(){

}
//robot_state->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LF_LEG)
void ContactEstimation::init(){
    sigma_0 = 0.05;
    mu_z = 0.0;
    mu_f = 50.0;
    sigma_z = 0.05;
    sigma_f = 0.05;
    P_pre.setIdentity();
    Q_pre.setZero();
    Q_pre(0,0) = sigma_0;Q_pre(1,1) = sigma_0;Q_pre(2,2) = sigma_0;Q_pre(3,3) = sigma_0;
    H_mea.setIdentity();
    H_mea(4,0) = 1;H_mea(5,1) = 1;H_mea(6,2) = 1;H_mea(7,3) = 1;
    //H_mea<<1.0,1.0;
    R_.setZero();
    R_(0.0) = sigma_f;//force
    R_(1,1) = sigma_z;//height
    R_(2.2) = sigma_f;//force
    R_(3,3) = sigma_z;//height
    R_(4,4) = sigma_f;//force
    R_(5,5) = sigma_z;//height
    R_(6,6) = sigma_f;//force
    R_(7,7) = sigma_z;//height


}
//MXR::NOTE: input:
//s_phi: desired contact state
//phase: current phase
//mu_1,sigma_1:  desired contact phase?
//mu_1_,sigma_1_: desired swing phase?
double ContactEstimation::computeContactPro(double phase){

//    p_c = 0.5 * (s_phi * (erf((phase-mu_0) / (sigma_0 * sqrt(2)))+erf((mu_1-phase) / (sigma_1 * sqrt(2))))+
//               (1-s_phi)*(2+erf((mu_0_-phase) / (sigma_0_ * sqrt(2)))+erf((phase-mu_1_) / (sigma_1 * sqrt(2)))));
    double p_c;
    p_c = 0.5*(1+erf((2*phase-1)/(sigma_0*sqrt(2))));
    return 1 - p_c;
}
//MXR::NOTE: input h(height of the foot)
double ContactEstimation::computeHeightPro(double h){
    double p_height;
    p_height = 0.5 *(1+erf((mu_z - h)/(sigma_z * sqrt(2))));
    return p_height;
}
//MXR::NOTE: input f(force of foot along z-axis)
double ContactEstimation::computeForcePro(double f){
    double p_force;
    p_force = 0.5 *(1+erf((f - mu_f)/(sigma_f * sqrt(2))));
    return p_force;
}
bool ContactEstimation::updateEstimationState(int b)
{
     for(int i = 0; i<4 ;i++){
         LimbEnum limb = static_cast<LimbEnum>(i);
         double phase_ = 0;
         if(limb_phase.at(limb).stance_status)
             phase_ = limb_phase.at(limb).stance_phase/(t_stance_);
         if(limb_phase.at(limb).swing_status)
             phase_ = limb_phase.at(limb).swing_phase/(t_swing_);
     }
     return true;
 }

void ContactEstimation::Predict(Eigen::Matrix<double,4,4>& A,Eigen::Matrix<double,4,4>& B,Eigen::Matrix<double,4,1>& x,Eigen::Matrix<double,4,1>& u){
    //double x_state_pre;
    Eigen::Matrix<double,4,4> A_T;
    x_state_pre = A*x+B*u;
    A_T = A.transpose();
    P_pre = A*P_pre*A_T+Q_pre;
//    std::cout<<"++++++++++++++++++"<<std::endl;
//    std::cout<<"u     "<<u<<std::endl;
//    std::cout<<"x_state_pre     "<<x_state_pre<<std::endl;
//    std::cout<<"++++++++++++++++++"<<std::endl;
}

void ContactEstimation::Update(Eigen::Matrix<double,8,1> &z){
    Eigen::Matrix<double,8,1> z_pred = H_mea * x_state_pre;
    Eigen::Matrix<double,8,1> y = z-z_pred;
//    std::cout<<"============================"<<std::endl;
//    std::cout<<"H_mea  "<<H_mea<<std::endl;
//    std::cout<<"y  "<<y<<std::endl;
//    std::cout<<"============================"<<std::endl;
    Eigen::Matrix<double,4,8> H_T = H_mea.transpose();
//    H_t = H_mea.transpose();
////    std::cout<<"============================"<<std::endl;
////    std::cout<<"H_mea   "<<H_mea<<std::endl;
////    std::cout<<"H_t     "<<H_mea<<std::endl;
////    std::cout<<"H_mea * P_pre * H_t   "<<H_mea * P_pre * H_t<<std::endl;
////    std::cout<<"============================"<<std::endl;
    Eigen::Matrix<double,8,8> S = H_mea * P_pre * H_T + R_;
    Eigen::Matrix<double,8,8> Si = S.inverse();
    Eigen::Matrix<double,4,8> PHt = P_pre* H_T;
    Eigen::Matrix<double,4,8> K = PHt * S;
    std::cout<<"============================"<<std::endl;
//    std::cout<<"P_pre     "<<P_pre<<std::endl;
//    std::cout<<"H_mea     "<<H_mea<<std::endl;
//    std::cout<<"H_T     "<<H_T<<std::endl;
//    std::cout<<"H_mea * P_pre * H_T     "<<H_mea * P_pre * H_T<<std::endl;
//    std::cout<<"S     "<<S<<std::endl;
//    std::cout<<"PHt     "<<PHt<<std::endl;
//    std::cout<<"y     "<<y<<std::endl;
//    std::cout<<"K     "<<K<<std::endl;
//    std::cout<<"K*y   "<<K*y<<std::endl;
//    std::cout<<"============================"<<std::endl;
    x_state_pre = x_state_pre + (K*y);
    Eigen::Matrix<double,4,4> I;
    I.setIdentity();
    P_pre = (I-K*H_mea)*P_pre;
//    std::cout<<"============================"<<std::endl;
//    std::cout<<"PHt     "<<PHt<<std::endl;
//    std::cout<<"K     "<<K<<std::endl;
//    std::cout<<"x_state_pre   "<<x_state_pre<<std::endl;
////    std::cout<<"K*y     "<<K*y<<std::endl;
//    std::cout<<"============================"<<std::endl;


}


void ContactEstimation::footposCB(const std_msgs::Float64MultiArray::ConstPtr &msg){
    p_h_lf = computeHeightPro(msg->data[2]);
    p_h_rf = computeHeightPro(msg->data[5]);
    p_h_lh = computeHeightPro(msg->data[11]);
    p_h_rh = computeHeightPro(msg->data[8]);
//    std::cout<<"============================"<<std::endl;
    //MXR::NOTE: when height is zero , p_h = 0.5
//    std::cout<<"p_h_lf     "<<p_h_lf<<std::endl;
//    std::cout<<"p_h_lh     "<<p_h_lh<<std::endl;
//    std::cout<<"p_h_rh     "<<p_h_rh<<std::endl;
//    std::cout<<"p_h_rf     "<<p_h_rf<<std::endl;
//    std::cout<<"============================"<<std::endl;
}
void ContactEstimation::lf_forceCB(const geometry_msgs::WrenchStamped::ConstPtr &msg){
    p_f_lf = computeForcePro(msg->wrench.force.z);
//    std::cout<<"============================"<<std::endl;
//    std::cout<<"p_f_lf   "<<p_f_lf<<std::endl;
//    std::cout<<"============================"<<std::endl;
}
void ContactEstimation::rf_forceCB(const geometry_msgs::WrenchStamped::ConstPtr &msg){
    p_f_rf = computeForcePro(msg->wrench.force.z);
//    std::cout<<"============================"<<std::endl;
//    std::cout<<"p_f_rf   "<<p_f_rf<<std::endl;
//    std::cout<<"============================"<<std::endl;
}
void ContactEstimation::lh_forceCB(const geometry_msgs::WrenchStamped::ConstPtr &msg){
    p_f_lh = computeForcePro(msg->wrench.force.z);
//    std::cout<<"============================"<<std::endl;
//    std::cout<<"p_f_lh   "<<p_f_lh<<std::endl;
//    std::cout<<"============================"<<std::endl;
}
void ContactEstimation::rh_forceCB(const geometry_msgs::WrenchStamped::ConstPtr &msg){
    p_f_rh = computeForcePro(msg->wrench.force.z);
//    std::cout<<"============================"<<std::endl;
//    std::cout<<"p_f_rh   "<<p_f_rh<<std::endl;
//    std::cout<<"============================"<<std::endl;
}
void ContactEstimation::robot_phaseCB(const free_gait_msgs::RobotState::ConstPtr &msg){
    p_c_lf = computeContactPro(msg->lf_leg_mode.phase);
    p_c_rf = computeContactPro(msg->rf_leg_mode.phase);
    p_c_lh = computeContactPro(msg->lh_leg_mode.phase);
    p_c_rh = computeContactPro(msg->rh_leg_mode.phase);
//    std::cout<<"============================"<<std::endl;
//    std::cout<<"p_c_lf     "<<p_c_lf<<std::endl;
//    std::cout<<"p_c_rf     "<<p_c_rf<<std::endl;
//    std::cout<<"p_c_lh     "<<p_c_lh<<std::endl;
//    std::cout<<"p_c_rh     "<<p_c_rh<<std::endl;
//    std::cout<<"============================"<<std::endl;
}
void ContactEstimation::ContactPublisherThread()
{
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        boost::recursive_mutex::scoped_lock lock(r_mutex);
        //============predict process============================
        Eigen::Matrix<double,4,4> A,B;//MXR::NOTE A(ZERO) b(IDENTITY)
        A.setZero();
        B.setIdentity();
        Eigen::Matrix<double,4,1> U;
        U(0,0)=p_c_lf;U(1,0)=p_c_rf;U(2,0)=p_c_lh;U(3,0)=p_c_rh;
        Predict(A,B,x_state_pre,U);
        //=======================================================
        //===========update process==============================
        Eigen::Matrix<double,8,1> Z;
        Z(0,0)=p_f_lf;Z(1,0)=p_h_lf;Z(2,0)=p_f_rf;Z(3,0)=p_h_rf;
        Z(4,0)=p_f_lh;Z(5,0)=p_h_lh;Z(6,0)=p_f_rh;Z(7,0)=p_h_rh;
        Update(Z);

        contact_prob.data[0] = x_state_pre(0,0);
        contact_prob.data[1] = x_state_pre(1,0);
        contact_prob.data[2] = x_state_pre(2,0);
        contact_prob.data[3] = x_state_pre(3,0);
//        std::cout<<"============================"<<std::endl;
//        std::cout<<"U    "<<U<<std::endl;
//        std::cout<<"Z    "<<Z<<std::endl;
//        std::cout<<"p_pre    "<<P_pre<<std::endl;
//        std::cout<<"============================"<<std::endl;
        contact_pro_pub.publish(contact_prob);
       //std::cout<<robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LF_LEG)<<std::endl;
       //std::cout<<robot_state_->getJointPositionsForLimb(free_gait::LimbEnum::LF_LEG)<<std::endl;
//       std::cout<<robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LF_LEG).y()<<std::endl;
//       std::cout<<robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LF_LEG).z()<<std::endl;
//        estimate_torque_lf_pub.publish(estimate_torque_lf);
//        estimate_torque_rf_pub.publish(estimate_torque_rf);
//        estimate_torque_rh_pub.publish(estimate_torque_rh);
//        estimate_torque_lh_pub.publish(estimate_torque_lh);
        lock.unlock();
        loop_rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "contact_pro_node");
    ros::NodeHandle nodehandle;
    ContactEstimation contactestimation(nodehandle);
//    Model model;
    while (ros::ok())
    {
        ros::spin();
    }
    return 0;
}

