#ifndef LEGODOM_H
#define LEGODOM_H

/*
 *
 * 2019.7.15
TODO 1.data log
    2.
*/
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
#include "pinocchio/fwd.hpp"
#include <gmbd_observer/gmbd_observer.h>
#include <gmbd_observer/contact_estimation.h>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "kindr/Core"
#include <queue>
#include <boost/thread.hpp>
#include <tic_toc.h>
//#include <pinocchio/fwd.hpp>

//sensor
//#include "gmbd_observer/contact_estimation.h"
//#include "pinocchio/fwd.hpp"
//#include "pinocchio/parsers/urdf.hpp"
//#include "pinocchio/algorithm/jacobian.hpp"
//#include "pinocchio/algorithm/joint-configuration.hpp"
//#include "pinocchio/algorithm/kinematics.hpp"
//#include "pinocchio/algorithm/crba.hpp"
//#include "pinocchio/algorithm/rnea.hpp"
//#include <gmbd_observer/gmbd_observer.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ContactsState.h>//    Position ;
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <kindr_ros/kindr_ros.hpp>
//include shunyao
//#include <quadruped_model/QuadrupedModel.hpp>
//#include <quadruped_model/quadruped_state.h>
//#include <quadruped_model/quadrupedkinematics.h>
#include <sim_assiants/FootContacts.h>
#include <free_gait_msgs/RobotState.h>
#include "free_gait_core/free_gait_core.hpp"


//#include <message_filters/subscriber.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/time_synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>

//*****************************************************************//
using namespace std;

namespace quadruped_odom
{
class QuadrupedEstimation
{
public:
    typedef std::unordered_map<free_gait::LimbEnum, free_gait::JointPositionsLeg, EnumClassHash> Joints_Init;
    //
    QuadrupedEstimation(const ros::NodeHandle& _nodehandle,
                        std::shared_ptr<free_gait::State> robot_state);
    QuadrupedEstimation(const ros::NodeHandle& _nodehandle);

    ~QuadrupedEstimation();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool ProcessSensorData();
    bool ProcessSensorData(ros::NodeHandle& _nodehandle,
                           sensor_msgs::Imu& imu_in,
                           std_msgs::Float64MultiArray& foot_in,
                           free_gait::JointPositions& jointsin,
                           nav_msgs::Odometry& gazeboin);
    //init
    void InitParam();
    void InitParam(const ros::NodeHandle& nh);
    void RobotStateLoad();
    //*********** position ***********//
    //pose -kin
    void getKINposition();
    void QuadrupedPosition();//base_to_world
    void GetPositionAddFromInit();
    void GetPositionAddEveryStep();
    void GetPositionAddEveryTime();
    void GetPositionAddEveryTimeInAllFoot();
    void GetPositionAddFootStep();
    void GetPositionParms();
    void GetFootInWorld();
    void GetFootPoseInBase();
    void GetFootPoseInBase(free_gait::JointPositions& jointsall);
    void FootstateJudge(const std_msgs::Float64MultiArray foot_out_);
    void ThetaInPI(free_gait::JointPositions& jointsall);
    //*********** vel ***********//
    void QuadrupedVel();//base_to_world
    //vel-kin
    void GetLinearVelFromKin();
    void GetLinearVelFromKinforTwoQueue();
    void GetLinearVelFromIMUAcc(ros::Time current_time);
    LinearVelocity GetLinearVelFromIMUAng();
    LinearVelocity GetLinearVelFilter(Eigen::Vector3d& v_filter);
    //vel-j_choose
    void GetLinearVelFromJointvel();
    void GetVelInWorld(LinearVelocity odomvel_odom);
    Eigen::Vector3d GetVelInOdom(LinearVelocity odomvel_world);

    //*********** Orientation ***********//
    void QuadrupedOrientation();//base_to_world
    void QuadrupedOrientation(sensor_msgs::Imu& imu_in_);//base_to_world
    //imu-q
    void GetQFromImu(const sensor_msgs::Imu& imu_output);
    //kin-q
    void GetQFromKin();
    void GetQPlus(const Eigen::VectorXcd& delta);
    //*********** output ***********//
    void LegOdomOut();
    void LegTFOut();
    void TFINIT();
    void TFINIT(const nav_msgs::Odometry& gazebo_msg,sensor_msgs::Imu& imu_0, free_gait::JointPositions& jointsall);
    Eigen::Vector3d QuaterniondToRPY(Eigen::Quaterniond& imu_000);
    Eigen::Vector3d QuaterniondToRPY(RotationQuaternion imu_rq);
    RotationQuaternion RPYTOQuaterniond(Eigen::Vector3d q_rpy);

    void setFootState(const std_msgs::Float64MultiArray& foot_msg);
//    Eigen::Quaterniond RPYTOQuaterniond(Eigen::Vector3d q_rpy_);
    ////////////////////
    //output

    void ResetParms();
//    void ResetParms();
    void LegPubTopic(ros::NodeHandle& _nn);
    Eigen::VectorXd GetStopStateOdom();
    ////////////////////

    bool imu_cb_flag, joints_cb_flag, foot_cb_flag, tfcal_flag,topic_flag,no_data_input_flag,P_init_flag,first_check_p,velget_flag,position_joints_flag,gazebo_flag;
    bool use_kin_state,use_pronto_pose,use_pronto_twist,use_imu_orientation;
    int foot_flag;
    double z_init,init_x,init_y ,init_z,init_wx,init_wy,init_wz,init_ww;
    unsigned int cycle_T,cycle_step,P_init_time;
    Eigen::Vector3d P_feet_1,P_feet_2,P_feet_3,P_feet_4, odom_YPR,odom_vel_inodom_vec;
    Eigen::Vector3d gazebo_queue,gazebo_queue_tmp,gazebo_change;
    Eigen::Quaterniond odom2init_orientation,RPY_bw_q;

    vector<double> tmp_init_data;
    vector<Pose> T_DH;
    vector<Position> P_DH, P_BW;
    vector<RotationMatrix> rotation_matrix_DH;
    vector<RotationQuaternion> R_Q;
    vector<Eigen::Vector3d> YPRfromQ;
    vector<RotationQuaternion> Q_kindr;
    vector<free_gait::JointPositionsLeg> jointsleg_;
    vector<LinearVelocity> V_Jacob;
    vector<free_gait::JointVelocitiesLeg> joint_vel_;

    LinearVelocity odom_vel,odom_vel_inodom,odom_vel_inodom_1,odom_vel_inodom_2;
    RotationQuaternion odom_orientation;
    Position odom_position, odom_positioninit,odom_position_tmp,tmp_legposition,first_tmp_position,last_odom_position,odom_position_error;
    Position P_leg1,P_leg2,P_leg3,P_leg4,leg1_first, leg2_first,leg3_first,leg4_first,P_leg1_BW,P_leg2_BW,P_leg3_BW,P_leg4_BW,cycle_long;
    Position P_leg1_,P_leg2_,P_leg3_,P_leg4_,P_leg1_tmp,P_leg2_tmp,P_leg3_tmp,P_leg4_tmp;

    free_gait::JointPositions joints_all;
    free_gait::JointVelocities jointsvel_all;

    queue<Position> vel_tmp;
    queue<pair<double, Position> > vel_info;
    deque<pair<ros::Time, Eigen::Vector3d> > vel_info_all;
    deque<Eigen::Vector3d> vel_filter_buf,vel_filter_vec;
    queue<size_t> flag_tmp;
    queue<Position> leg1w_tmp,leg2w_tmp,leg3w_tmp,leg4w_tmp, odom_p_tmp;
    queue<Eigen::Vector3d> gazebo_tmp;

    sensor_msgs::Imu imu_output;
    sensor_msgs::JointState joints_output;
    nav_msgs::Odometry gazebo_output;
    std_msgs::Float64MultiArray foot_output;
    nav_msgs::Odometry legodom_map_,leg_odom,legodom_init,legodom_error;
    tf::Transform legodom_tf;

    std_msgs::Float64MultiArray imu_out;
    std_msgs::Float64MultiArray joint_out;
    std_msgs::Float64MultiArray foot_out;
    std_msgs::Float64MultiArray gazebo_out;
    std_msgs::Float64MultiArray legodom_error_cal;
    std_msgs::Float64MultiArray Contact_state;
    std::string _cal_vel_way, _orientation_way, _cal_position_way, imu_topic_name_;

    ros::Publisher legodom_odom_pub,legodom_error_pub,legodom_map_pub, legodom_init_pub,legPose_pub,gazebo_pub,imuvel_pub;

    ros::Time init_time;

    RotationQuaternion base_orientation;
    geometry_msgs::PoseWithCovarianceStamped foot_odom;
    Eigen::Vector3d P_LF,P_RF,P_RH,P_LH,pre_P_LF,pre_p_RF,pre_P_RH,pre_P_LH,
    P_LF_w,P_RF_w,P_RH_w,P_LH_w,P_LF_r,P_RF_r,P_RH_r,P_LH_r;
    std_msgs::Float64MultiArray footpos_delta,footpos_world,gazebo_contact;
    Eigen::Vector3d LF_foot_Pos_delta,RF_foot_Pos_delta,RH_foot_Pos_delta,LH_foot_Pos_delta;
    double N_contact,z_delta,x_delta,y_delta,pre_pos;
    Eigen::Vector3d POSE_IN_WORLD;
    sim_assiants::FootContact lf_foot_contact_, rf_foot_contact_, rh_foot_contact_, lh_foot_contact_;
    sim_assiants::FootContacts foot_contacts_;

    geometry_msgs::WrenchStamped estimate_torque_lf, estimate_torque_rf, estimate_torque_rh, estimate_torque_lh;
private:
    ros::NodeHandle nodeHandle_, nodeh_;
    ros::Subscriber imu_sub,joints_sub,foot_state_sub, gazebo_sub,robot_state_sub,modelStatesSub_;
    ros::Subscriber pronto_pose_sub,pronto_twist_sub,init_pose_sub,LegOdom_sub_,Contact_sub,joint_state_sub,gazeboC_sub,robot_phase_sub;
    ros::Publisher footpos_pub,contactpro_pub,footContact_pub;
    ros::Publisher estimate_torque_lf_pub,estimate_torque_rf_pub,estimate_torque_lh_pub,estimate_torque_rh_pub;
    //main function
    std::shared_ptr<free_gait::State> robot_state_;
    std::unique_ptr<gmbd_obs::Gmbd_obs> gmbd_obs_lf_;
    std::unique_ptr<gmbd_obs::Gmbd_obs> gmbd_obs_rf_;
    std::unique_ptr<gmbd_obs::Gmbd_obs> gmbd_obs_lh_;
    std::unique_ptr<gmbd_obs::Gmbd_obs> gmbd_obs_rh_;
    std::shared_ptr<ContactEstimation> contact_ptr_;
    double _cal_fator_x, _cal_fator_y, _cal_fator_z, real_time_factor;
    //tf
    geometry_msgs::TransformStamped odom2map_tf,base2odom_tf,base2map_tf;
    geometry_msgs::PoseWithCovarianceStamped Pose_stamped_;
    tf::TransformBroadcaster odom2map_broadcaster,base2odom_broadcaster,base2map_broadcaster,tfBoardcaster_;
    tf::Transform odom_to_footprint, footprint_to_base,base_to_odom;
    //    callback
    void imuCb(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void jointsCb(const sensor_msgs::JointState::ConstPtr& joint_msg);
    void footstateCb(const std_msgs::Float64MultiArray::ConstPtr& foot_msg);
    void gazeboCb(const nav_msgs::Odometry::ConstPtr& gazebo_msg);
    void robotstateCb(const free_gait_msgs::RobotState::ConstPtr& robotstate_msg);
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStatesMsg);
    void Gmbd_obs_Callback(const sensor_msgs::JointState::ConstPtr &robot_state_);
    void robot_phaseCB(const free_gait_msgs::RobotState::ConstPtr &msg);
    void computeContactPro();

    void InitCB(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void prontoPoseCB(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void prontoTwistCB(const geometry_msgs::TwistWithCovarianceStamped& twist);
    void KinCB(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void ConCB(const std_msgs::Float64MultiArray& msg);
    void gazeboConCB(const std_msgs::Float64MultiArray& msg);

    //just for time at sama time
    //    void sensor_callback(const sensor_msgs::ImuConstPtr &imu_msg,
    //                         const sensor_msgs::JointState::ConstPtr& joint_msg,
    //                         const gazebo_msgs::ContactsStateConstPtr &feet1_msg,
    //                         const gazebo_msgs::ContactsStateConstPtr &feet2_msg,
    //                         const gazebo_msgs::ContactsStateConstPtr &feet3_msg,
    //                         const gazebo_msgs::ContactsStateConstPtr &feet4_msg,
    //                         const nav_msgs::Odometry::ConstPtr& gazebo_msg);
    Eigen::Vector3d X_everystep ;

    int  odom_position_T, foot_state_T;
    unsigned int vel_kin_T_,velfromkin_,leg1_T,leg2_T,leg3_T,leg4_T,gazebo_T,gazebo_pose_stop_time;
    Joints_Init joints_position_init;
    double  _vel_set_T,_odom_dt;
    size_t first_foot_flag;

    //thread
    //    boost::thread LoopThread;
    //    boost::recursive_mutex r_mutex;


};
}
#endif
