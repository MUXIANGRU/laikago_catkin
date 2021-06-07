#ifndef GMBD_OBSERVER_H
#define GMBD_OBSERVER_H
#include <pinocchio/algorithm/crba.hpp>
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "quadruped_model/quadruped_state.h"
//#include "pinocchio/parsers/urdf/model.hxx"
using namespace std;
using namespace Eigen;
using namespace quadruped_model;
using namespace pinocchio;
namespace gmbd_obs{
class Gmbd_obs
{
public:
    Gmbd_obs();
    Gmbd_obs(LimbEnum leg);
    ~Gmbd_obs();
    bool init();
    bool compute_M(Eigen::VectorXd &q);
    bool compute_C(Eigen::VectorXd &q,Eigen::VectorXd &qdot);
    bool compute_G(Eigen::VectorXd &q);
    bool compute_tau_j(Eigen::VectorXd &tau);
    bool compute_J(Eigen::VectorXd &q);
    bool computeJacobian(Eigen::VectorXd &joint_positions,Eigen::MatrixXd& jacobian);
    bool compute(Eigen::VectorXd &q,Eigen::VectorXd &qdot,Eigen::VectorXd &tau);

    Model getleg_model()
    {
        return leg_model;
    }
    string filepath()
    {
        return urdf_file_leg;
    }
    Matrix3d get_M(){
        return M;
    }
    Matrix3d get_C(){
        return C;
    }
    Vector3d get_G(){
        return G;
    }
    Vector3d get_torque_d(){
        return torque_d;
    }
    Vector3d get_disturbance_force(){
        return disturbance_force;
    }
    Vector3d get_ata(){
        return applied_torque_approximation;
    }

private:
    string urdf_file_leg;
    Model leg_model;
//    Model leg_model_rf;
//    Model leg_model_rh;
//    Model leg_model_lh;
    double t_prev=0.0;
    double delta_t=0.001;
    double freq=15;
    double g=9.81;
    int start=0;
    double gamma;
    double beta;
    Matrix3d M;
    Matrix3d C;
    Vector3d G;
    Vector3d Q;
    Vector3d QDot;
    Vector3d tau;
    int n_joints;
    Vector3d torque_d_prev;
    Vector3d torque_d;
    Vector3d torque_j;
    Vector3d p_k_prev;
    Vector3d p_k;
    Vector3d alpha_k;
    Vector3d disturbance_force;
    Vector3d applied_torque_approximation;
    Matrix3d S;
    Matrix3d J;
    MatrixXd jacobian;
    Data data;
    KDL::Tree tree_;
//    KDL::Chain LF_Chain, RF_Chain,RH_Chain,LH_Chain;
    KDL::Chain Leg_Chain;
};
}
#endif // GMBD_OBSERVER_H
