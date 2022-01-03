#include <gmbd_observer/gmbd_observer.h>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
 using namespace gmbd_obs;
 Gmbd_obs::Gmbd_obs(){}
 Gmbd_obs::Gmbd_obs(LimbEnum leg){

         switch (leg)
         {
         case LimbEnum::LF_LEG:{
             init();
             pinocchio::Model lf_leg_model_;

//             string urdf_file_leg_;
             string urdf_file =  "/home/mxr/catkin_laikago/src/quadruped_locomotion/quadruped_model/urdf/laikago.urdf";
             urdf_file_leg = "/home/mxr/catkin_laikago/src/quadruped_locomotion/quadruped_model/urdf/quadruped_model_lf_leg.urdf";
             kdl_parser::treeFromFile(urdf_file,tree_);
             tree_.getChain("base", "LF_FOOT", Leg_Chain);
//             cout<< "here is ok 2.5?"<<endl;
             pinocchio::urdf::buildModel(urdf_file_leg,lf_leg_model_);
//             cout<< "here is ok 3cd?"<<endl;
             leg_model=lf_leg_model_;
             cout<<"lf_leg_model constructed"<<endl;
             cout<<"lf_leg_model is:\n"<<leg_model<<endl;
            break;
        }
         case LimbEnum::RF_LEG:{
             init();
             pinocchio::Model rf_leg_model_;
             string urdf_file =  "/home/mxr/catkin_laikago/src/quadruped_locomotion/quadruped_model/urdf/laikago.urdf";
             urdf_file_leg = "/home/mxr/catkin_laikago/src/quadruped_locomotion/quadruped_model/urdf/quadruped_model_rf_leg.urdf";
             kdl_parser::treeFromFile(urdf_file,tree_);
             tree_.getChain("base", "RF_FOOT", Leg_Chain);
             pinocchio::urdf::buildModel(urdf_file_leg,rf_leg_model_);
             leg_model=rf_leg_model_;
            cout<<"rf_leg_model constructed"<<endl;
            cout<<"rf_leg_model is:\n"<<leg_model<<endl;
             break;
        }

         case LimbEnum::RH_LEG:{
             init();
             pinocchio::Model rh_leg_model_;
             string urdf_file =  "/home/mxr/catkin_laikago/src/quadruped_locomotion/quadruped_model/urdf/laikago.urdf";
             urdf_file_leg = "/home/mxr/catkin_laikago/src/quadruped_locomotion/quadruped_model/urdf/quadruped_model_rh_leg.urdf";
             kdl_parser::treeFromFile(urdf_file,tree_);
             tree_.getChain("base", "RH_FOOT", Leg_Chain);
             pinocchio::urdf::buildModel(urdf_file_leg,rh_leg_model_);
             leg_model=rh_leg_model_;
             cout<<"rh_leg_model constructed"<<endl;
             cout<<"rh_leg_model is:\n"<<leg_model<<endl;
             break;
         }
         case LimbEnum::LH_LEG:{
             init();
             pinocchio::Model lh_leg_model_;
             string urdf_file =  "/home/mxr/catkin_laikago/src/quadruped_locomotion/quadruped_model/urdf/laikago.urdf";
             urdf_file_leg = "/home/mxr/catkin_laikago/src/quadruped_locomotion/quadruped_model/urdf/quadruped_model_lh_leg.urdf";
             kdl_parser::treeFromFile(urdf_file,tree_);
             tree_.getChain("base", "LH_FOOT", Leg_Chain);
             pinocchio::urdf::buildModel(urdf_file_leg,lh_leg_model_);
             leg_model=lh_leg_model_;
             cout<<"lh_leg_model constructed"<<endl;
             cout<<"lh_leg_model is:\n"<<leg_model<<endl;
             break;
         }
 //        default:{
 //            cout<<"error"<<endl;
 //            break;}
        }

 }
 Gmbd_obs::~Gmbd_obs(){}

 bool Gmbd_obs::init(){
     start = 0;
     S = S.setIdentity();
     jacobian.setZero();
     M = M.setZero();
     C = C.setZero();
     G = G.setZero();
     torque_j.setZero();
     torque_d.setZero();
     torque_d_prev.setZero();
     p_k.setZero();
     p_k_prev.setZero();
     Q.setZero();
     QDot.setZero();
     disturbance_force.setZero();
 }

 bool Gmbd_obs::compute(Eigen::VectorXd &q,Eigen::VectorXd &qdot,Eigen::VectorXd &tau){
     if(start== 0){
         compute_M(q);
         p_k_prev = M*qdot;
         torque_d_prev = torque_d_prev.setZero();
         start = 1;
         cout<<"=======initialized======"<<endl;
     }else{
         compute_M(q);
         compute_C(q,qdot);
         compute_G(q);
//         compute_J(q);
         computeJacobian(q,jacobian);
         compute_tau_j(tau);
         gamma = exp(-freq*delta_t);
         beta = (1-gamma)/(gamma*delta_t);
         p_k = M*QDot;
         alpha_k = beta*p_k+S*torque_j+C.transpose()*QDot-G;
         torque_d = (gamma-1)*alpha_k+beta*(p_k-gamma*p_k_prev)+gamma*torque_d_prev;
         //cout<<"=======estimation torque========"<<endl<<torque_d<<endl;
         //cout<<"jacobian::"<<endl<<jacobian<<endl;
         //cout<<"jacobian.transpose().inverse()"<<endl<<jacobian.transpose().inverse()<<endl;
         disturbance_force = jacobian.transpose().inverse()*torque_d;
         applied_torque_approximation = jacobian.transpose()*disturbance_force+G;
         torque_d_prev = torque_d;
         p_k_prev = p_k;
         //cout<<"disturbance_force :\n"<< disturbance_force <<endl;
         //cout<<"applied_torque_approximation :\n"<<applied_torque_approximation <<endl;
         //cout<<"torque_j is:\n"<<torque_j<<endl;
         return true;
     }
 }
 bool Gmbd_obs::compute_M(Eigen::VectorXd &q){
 //    Data data_m;
     Data data_m(leg_model);
//     data_m= data;
     Q = q;
     pinocchio::crba(leg_model,data_m,Q);
     data_m.M.triangularView<Eigen::StrictlyLower>() = data_m.M.transpose().triangularView<Eigen::StrictlyLower>();
     M = data_m.M;
    // cout<<"M is:\n"<<M<<endl;
     return true;
 }

 bool Gmbd_obs::compute_C(Eigen::VectorXd &q,Eigen::VectorXd &qdot){
     Data data_c(leg_model);
//     data = data_c;
     Q = q;
     QDot = qdot;
     pinocchio::computeCoriolisMatrix(leg_model,data_c,Q,QDot);
     C = data_c.C;
     //cout<<"C is:\n"<<C<<endl;
     return true;
 }

 bool Gmbd_obs::compute_G(Eigen::VectorXd &q){
     Data data_g(leg_model);
//     data = data_g;
     Q = q;
     pinocchio::computeGeneralizedGravity(leg_model,data_g,Q);
     G = data_g.g;
     //cout<<"G is:\n"<<G<<endl;
     return true;
 }
 bool Gmbd_obs::compute_tau_j(Eigen::VectorXd &tau){
     torque_j = tau;     return true;
 }

 bool Gmbd_obs::compute_J(Eigen::VectorXd &q){
     Data data_j(leg_model);
     Q=q;
//     const int JOINT_ID = 3;
//     Data::Matrix3x J_(3,3);
     pinocchio::computeJointJacobians(leg_model,data_j,Q);
     J = data_j.J.topRows(3);
//     cout<<"J====0000000========="<<endl<<J<<endl;
//     cout<<"===============J_"<<J_<<endl;
//     cout<<"ok2"<<endl;
     return true;
 }
 bool Gmbd_obs::computeJacobian(Eigen::VectorXd &joint_positions, Eigen::MatrixXd &jacobian){
     int number_of_joints = joint_positions.size();
     KDL::JntArray joints = KDL::JntArray(number_of_joints);
     KDL::Jacobian J;
     J.resize(number_of_joints);
     for(int i = 0; i<number_of_joints; i++)
     {
       joints(i) = joint_positions(i);
       //cout<<"joint_lf is:"<<endl<<joints(i)<<endl;
   //    cout<<"the "<<i<<"th joint: "<<joints(i)<<endl;
     }
   //  cout<<LF_Chain.getNrOfJoints()<<" joint rows"<<joints.rows()<<" joint columns"<<joints.columns()<<endl;
   //  cout<<joints.data<<endl;
     int error_code = 0;
     KDL::ChainJntToJacSolver jacobian_solver(Leg_Chain);
     error_code = jacobian_solver.JntToJac(joints, J);
     if(error_code != 0)
     {
       cout<<"Failed to solve Jacobian problem"<<" error code :"<<error_code<<endl;
       return false;
     }
     jacobian = J.data.block(0,0,3,3);
//     jacobian = jacobian.block(0,0,3,3);
//     cout<<"J.data is==========================="<<endl<<jacobian<<endl;
     return true;
 }
