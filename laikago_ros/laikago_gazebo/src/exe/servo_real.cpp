/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "laikago_msgs/LowCmd.h"
#include "laikago_msgs/LowState.h"
#include "laikago_msgs/MotorCmd.h"
#include "laikago_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "../body.h"
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace laikago_model;

bool start_up = true;

class multiThread
{
public:
    multiThread(){
        imu_sub = nm.subscribe("/imu/data", 1, &multiThread::imuCallback, this);
//        footForce_sub[0] = nm.subscribe("/visual/FR_foot_contact/the_force", 1, &multiThread::FRfootCallback, this);
//        footForce_sub[1] = nm.subscribe("/visual/FL_foot_contact/the_force", 1, &multiThread::FLfootCallback, this);
//        footForce_sub[2] = nm.subscribe("/visual/RR_foot_contact/the_force", 1, &multiThread::RRfootCallback, this);
//        footForce_sub[3] = nm.subscribe("/visual/RL_foot_contact/the_force", 1, &multiThread::RLfootCallback, this);
        servo_sub[0] = nm.subscribe("/joint_states", 1, &multiThread::FRhipCallback, this);
        servo_sub[1] = nm.subscribe("/joint_states", 1, &multiThread::FRthighCallback, this);
        servo_sub[2] = nm.subscribe("/joint_states", 1, &multiThread::FRcalfCallback, this);
        servo_sub[3] = nm.subscribe("/joint_states", 1, &multiThread::FLhipCallback, this);
        servo_sub[4] = nm.subscribe("/joint_states", 1, &multiThread::FLthighCallback, this);
        servo_sub[5] = nm.subscribe("/joint_states", 1, &multiThread::FLcalfCallback, this);
        servo_sub[6] = nm.subscribe("/joint_states", 1, &multiThread::RRhipCallback, this);
        servo_sub[7] = nm.subscribe("/joint_states", 1, &multiThread::RRthighCallback, this);
        servo_sub[8] = nm.subscribe("/joint_states", 1, &multiThread::RRcalfCallback, this);
        servo_sub[9] = nm.subscribe("/joint_states", 1, &multiThread::RLhipCallback, this);
        servo_sub[10] = nm.subscribe("/joint_states", 1, &multiThread::RLthighCallback, this);
        servo_sub[11] = nm.subscribe("/joint_states", 1, &multiThread::RLcalfCallback, this);
    }

    void imuCallback(const sensor_msgs::Imu & msg)
    { 
        lowState.imu.quaternion[0] = msg.orientation.w;
        lowState.imu.quaternion[1] = msg.orientation.x;
        lowState.imu.quaternion[2] = msg.orientation.y;
        lowState.imu.quaternion[3] = msg.orientation.z;
    }

    void FRhipCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        start_up = false;
        lowState.motorState[0].mode = 0x0A;
        lowState.motorState[0].position = msg->position[6];
        lowState.motorState[0].velocity = msg->velocity[6];
        lowState.motorState[0].torque = msg->effort[6];
    }

    void FRthighCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        lowState.motorState[1].mode = 0x0A;
        lowState.motorState[1].position = msg->position[7];
        lowState.motorState[1].velocity = msg->velocity[7];
        lowState.motorState[1].torque = msg->effort[7];
    }

    void FRcalfCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        lowState.motorState[2].mode = 0x0A;
        lowState.motorState[2].position = msg->position[8];
        lowState.motorState[2].velocity = msg->velocity[8];
        lowState.motorState[2].torque = msg->effort[8];
    }

    void FLhipCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        start_up = false;
        lowState.motorState[3].mode = 0x0A;
        lowState.motorState[3].position = msg->position[0];
        lowState.motorState[3].velocity = msg->velocity[0];
        lowState.motorState[3].torque = msg->effort[0];
    }

    void FLthighCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        lowState.motorState[4].mode = 0x0A;
        lowState.motorState[4].position = msg->position[1];
        lowState.motorState[4].velocity = msg->velocity[1];
        lowState.motorState[4].torque = msg->effort[1];
    }

    void FLcalfCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        lowState.motorState[5].mode = 0x0A;
        lowState.motorState[5].position = msg->position[2];
        lowState.motorState[5].velocity = msg->velocity[2];
        lowState.motorState[5].torque = msg->effort[2];
    }

    void RRhipCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        start_up = false;
        lowState.motorState[6].mode = 0x0A;
        lowState.motorState[6].position = msg->position[9];
        lowState.motorState[6].velocity = msg->velocity[9];
        lowState.motorState[6].torque = msg->effort[9];
    }

    void RRthighCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        lowState.motorState[7].mode = 0x0A;
        lowState.motorState[7].position = msg->position[10];
        lowState.motorState[7].velocity = msg->velocity[10];
        lowState.motorState[7].torque = msg->effort[10];
    }

    void RRcalfCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        lowState.motorState[8].mode = 0x0A;
        lowState.motorState[8].position = msg->position[11];
        lowState.motorState[8].velocity = msg->velocity[11];
        lowState.motorState[8].torque = msg->effort[11];
    }

    void RLhipCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        start_up = false;
        lowState.motorState[9].mode = 0x0A;
        lowState.motorState[9].position = msg->position[3];
        lowState.motorState[9].velocity = msg->velocity[3];
        lowState.motorState[9].torque = msg->effort[3];
    }

    void RLthighCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        lowState.motorState[10].mode = 0x0A;
        lowState.motorState[10].position = msg->position[4];
        lowState.motorState[10].velocity = msg->velocity[4];
        lowState.motorState[10].torque = msg->effort[4];
    }

    void RLcalfCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        lowState.motorState[11].mode = 0x0A;
        lowState.motorState[11].position = msg->position[5];
        lowState.motorState[11].velocity = msg->velocity[5];
        lowState.motorState[11].torque = msg->effort[5];
    }
private:
    ros::NodeHandle nm;
    ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laikago_servo_real");

    multiThread listen_publish_obj;
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(250000); // must wait 300ms, to get first state  (250000)

    ros::NodeHandle n;
    ros::Publisher lowState_pub; //for rviz visualization
    // ros::Rate loop_rate(1000);
    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_real/lowState/state", 1);
    servo_pub[0] = n.advertise<laikago_msgs::MotorCmd>("/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<laikago_msgs::MotorCmd>("/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<laikago_msgs::MotorCmd>("/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<laikago_msgs::MotorCmd>("/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<laikago_msgs::MotorCmd>("/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<laikago_msgs::MotorCmd>("/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<laikago_msgs::MotorCmd>("/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<laikago_msgs::MotorCmd>("/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<laikago_msgs::MotorCmd>("/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<laikago_msgs::MotorCmd>("/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<laikago_msgs::MotorCmd>("/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<laikago_msgs::MotorCmd>("/RL_calf_controller/command", 1);

    motion_init_real();

    while (ros::ok()){
        /*
        control logic
        */
        lowState_pub.publish(lowState);
        sendServoCmd();  //MXR::NOTE publish the joint command from body.cpp
    }
    return 0;
}
