/*
  *  gmbd_obs_node.cpp
  *  Descriotion:
  *
  *  Created on: May,20, 2019
  *  Author: Guo Yang
  *  Institute: Harbin Institute of Technology, Shenzhen
  */
#include <gmbd_observer/gmbd_observer.h>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/WrenchStamped.h"
#include "free_gait_msgs/RobotState.h"
#include "std_msgs/String.h"
#include "boost/thread.hpp"
#include <ros/package.h>
using namespace std;
class ExternalForceEstimation
{
public:
    ExternalForceEstimation(ros::NodeHandle &nodehandle)
        : nodehandle_(nodehandle)

    {
        gmbd_obs_lf_.reset(new gmbd_obs::Gmbd_obs(LimbEnum:: LF_LEG));
        gmbd_obs_rf_.reset(new gmbd_obs::Gmbd_obs(LimbEnum:: RF_LEG));
        gmbd_obs_rh_.reset(new gmbd_obs::Gmbd_obs(LimbEnum:: RH_LEG));
        gmbd_obs_lh_.reset(new gmbd_obs::Gmbd_obs(LimbEnum:: LH_LEG));
        joint_state_sub = nodehandle_.subscribe("/gazebo/robot_states", 1, &ExternalForceEstimation::Gmbd_obs_Callback, this);
        estimate_torque_lf_pub = nodehandle_.advertise<geometry_msgs::WrenchStamped>("/estimate_torque_lf", 1);
        estimate_torque_rf_pub = nodehandle_.advertise<geometry_msgs::WrenchStamped>("/estimate_torque_rf", 1);
        estimate_torque_rh_pub = nodehandle_.advertise<geometry_msgs::WrenchStamped>("/estimate_torque_rh", 1);
        estimate_torque_lh_pub = nodehandle_.advertise<geometry_msgs::WrenchStamped>("/estimate_torque_lh", 1);
        Wrenchpublisherthread = boost::thread(boost::bind(&ExternalForceEstimation::WrenchPublisherThreadFun, this));
    }

private:
    ros::NodeHandle nodehandle_;
    ros::Subscriber joint_state_sub;
    ros::Publisher estimate_torque_lf_pub, estimate_torque_rf_pub, estimate_torque_rh_pub, estimate_torque_lh_pub;
    geometry_msgs::WrenchStamped estimate_torque_lf, estimate_torque_rf, estimate_torque_rh, estimate_torque_lh;
    boost::thread Wrenchpublisherthread;
    boost::recursive_mutex r_mutex;
    std::unique_ptr<gmbd_obs::Gmbd_obs> gmbd_obs_lf_,gmbd_obs_rf_,gmbd_obs_rh_,gmbd_obs_lh_;
    void Gmbd_obs_Callback(const free_gait_msgs::RobotState::ConstPtr &robot_state_)
    {
        VectorXd q_lf_, q_rf_, q_rh_, q_lh_;
        VectorXd v_lf_, v_rf_, v_rh_, v_lh_;
        VectorXd e_lf_, e_rf_, e_rh_, e_lh_;
        for (int i = 0; i < 3; i++)
        {
            q_lf_.resize(3);
            q_rf_.resize(3);
            q_rh_.resize(3);
            q_lh_.resize(3);
            v_lf_.resize(3);
            v_rf_.resize(3);
            v_rh_.resize(3);
            v_lh_.resize(3);
            e_lf_.resize(3);
            e_rf_.resize(3);
            e_rh_.resize(3);
            e_lh_.resize(3);
            q_lf_[i] = robot_state_->lf_leg_joints.position[i];
            q_rf_[i] = robot_state_->rf_leg_joints.position[i];
            q_rh_[i] = robot_state_->rh_leg_joints.position[i];
            q_lh_[i] = robot_state_->lh_leg_joints.position[i];
            v_lf_[i] = robot_state_->lf_leg_joints.velocity[i];
            v_rf_[i] = robot_state_->rf_leg_joints.velocity[i];
            v_rh_[i] = robot_state_->rh_leg_joints.velocity[i];
            v_lh_[i] = robot_state_->lh_leg_joints.velocity[i];
            e_lf_[i] = robot_state_->lf_leg_joints.effort[i];
            e_rf_[i] = robot_state_->rf_leg_joints.effort[i];
            e_rh_[i] = robot_state_->rh_leg_joints.effort[i];
            e_lh_[i] = robot_state_->lh_leg_joints.effort[i];
        }
//        q_lf_(0)=0;
//        q_lf_(1)=0.78;
//        q_lf_(2)=-1.57;
        cout << "q_lf is:\n" << q_lf_ << endl;
        cout << "v_lf is:\n" << v_lf_ << endl;
        cout << "e_lf is:\n" << e_lf_ << endl;

        cout << "q_rf is:\n" << q_rf_ << endl;
        cout << "v_rf is:\n" << v_rf_ << endl;
        cout << "e_rf is:\n" << e_rf_ << endl;

        cout << "q_rh is:\n" << q_rh_ << endl;
        cout << "v_rh is:\n" << v_rh_ << endl;
        cout << "e_rh is:\n" << e_rh_ << endl;

        cout << "q_lh is:\n" << q_lh_ << endl;
        cout << "v_lh is:\n" << v_lh_ << endl;
        cout << "e_lh is:\n" << e_lh_ << endl;

        gmbd_obs_lf_->compute(q_lf_,v_lf_,e_lf_);
        gmbd_obs_rf_->compute(q_rf_,v_rf_,e_rf_);
        gmbd_obs_rh_->compute(q_rh_,v_rh_,e_rh_);
        gmbd_obs_lh_->compute(q_lh_,v_lh_,e_lh_);
//        q_joint.setZero(3);
//        gmbd_obs_lf_->compute_J(q_joint);
//        gmbd_obs_lf_->computeJacobian(q_joint,jacabian);
//        cout<<"==jacabian===="<<jacabian.block(0,0,3,3)<<endl;
        cout<<"force================="<<gmbd_obs_lf_->get_disturbance_force()<<endl;
        cout<<"torque================="<<gmbd_obs_lf_->get_torque_d()<<endl;
        estimate_torque_lf.wrench.force.x = gmbd_obs_lf_->get_disturbance_force()[0];
        estimate_torque_lf.wrench.torque.x = gmbd_obs_lf_->get_ata()[0];
//        cout << "estimat_torque_lf_x is:\n"
//             << estimate_torque_lf.force.x << endl;
        estimate_torque_lf.wrench.force.y = gmbd_obs_lf_->get_disturbance_force()[1];
        estimate_torque_lf.wrench.torque.y = gmbd_obs_lf_->get_ata()[1];
//        cout << "estimat_torque_lf_y is:\n"
//             << estimate_torque_lf.force.y << endl;
        estimate_torque_lf.wrench.force.z = gmbd_obs_lf_->get_disturbance_force()[2];
        estimate_torque_lf.wrench.torque.z = gmbd_obs_lf_->get_ata()[2];

        cout << "estimat_torque_lf_z is:\n"
             << estimate_torque_lf.wrench.force.z << endl;
//        cout<<"lf_leg_force is:\n"<<estimate_torque_lf.force<<endl;

        estimate_torque_rf.wrench.force.x = gmbd_obs_rf_->get_disturbance_force()[0];
        estimate_torque_rf.wrench.torque.x= gmbd_obs_rf_->get_ata()[0];
        estimate_torque_rf.wrench.force.y = gmbd_obs_rf_->get_disturbance_force()[1];
        estimate_torque_rf.wrench.torque.y= gmbd_obs_rf_->get_ata()[1];
        estimate_torque_rf.wrench.force.z = gmbd_obs_rf_->get_disturbance_force()[2];
        estimate_torque_rf.wrench.torque.z= gmbd_obs_rf_->get_ata()[2];

        estimate_torque_rh.wrench.force.x = gmbd_obs_rh_->get_disturbance_force()[0];
        estimate_torque_rh.wrench.torque.x= gmbd_obs_rh_->get_ata()[0];
        estimate_torque_rh.wrench.force.y = gmbd_obs_rh_->get_disturbance_force()[1];
        estimate_torque_rh.wrench.torque.y= gmbd_obs_rh_->get_ata()[1];
        estimate_torque_rh.wrench.force.z = gmbd_obs_rh_->get_disturbance_force()[2];
        estimate_torque_rh.wrench.torque.z= gmbd_obs_rh_->get_ata()[2];

        estimate_torque_lh.wrench.force.x = gmbd_obs_lh_->get_disturbance_force()[0];
        estimate_torque_lh.wrench.torque.x= gmbd_obs_lh_->get_ata()[0];
        estimate_torque_lh.wrench.force.y = gmbd_obs_lh_->get_disturbance_force()[1];
        estimate_torque_lh.wrench.torque.y= gmbd_obs_lh_->get_ata()[1];
        estimate_torque_lh.wrench.force.z = gmbd_obs_lh_->get_disturbance_force()[2];
        estimate_torque_lh.wrench.torque.z= gmbd_obs_lh_->get_ata()[2];
    }
    void WrenchPublisherThreadFun()
    {
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            boost::recursive_mutex::scoped_lock lock(r_mutex);
            estimate_torque_lf_pub.publish(estimate_torque_lf);
            estimate_torque_rf_pub.publish(estimate_torque_rf);
            estimate_torque_rh_pub.publish(estimate_torque_rh);
            estimate_torque_lh_pub.publish(estimate_torque_lh);
//            cout<<"message is published"<<endl;
            lock.unlock();
            loop_rate.sleep();
        }
    }
};



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gmm_observer_node");
    ros::NodeHandle nodehandle;
    ExternalForceEstimation forceestimation(nodehandle);
//    Model model;
    while (ros::ok())
    {
        ros::spin();
    }
    return 0;
}
