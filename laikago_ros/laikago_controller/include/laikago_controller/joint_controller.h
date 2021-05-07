/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _JOINT_CONTROLLER_H_
#define _JOINT_CONTROLLER_H_

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include "laikago_msgs/MotorCmd.h"
#include "laikago_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <balance_controller/ros_controler/robot_state_interface.hpp>
#include "sensor_msgs/JointState.h"

#define PMSM      (0x0A)
#define PosStopF  (2.146E+9f)
#define VelStopF  (16000.0f)

namespace laikago_controller{
    class LaikagoJointController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        private:
            hardware_interface::JointHandle joint;
            hardware_interface::RobotStateHandle::Data robot_state_data_;
            hardware_interface::RobotStateHandle robot_state_handle;
            hardware_interface::RobotStateInterface robot_state_interface_;
            ros::Subscriber sub_cmd,js_sub_;
            control_toolbox::Pid pid_controller_;
            boost::scoped_ptr<realtime_tools::RealtimePublisher<laikago_msgs::MotorState> > controller_state_publisher_ ;
            double pos_read[12], pos_write[12], vel_read[12], vel_write[12], eff_read[12],eff_write[12];
            double position[3], orinetation[4], linear_vel[3], angular_vel[3],contact_pressure[4];
            int foot_contact[4],motor_status_word[12],mode_of_joint[12];
            sensor_msgs::JointState js_msg;

        public:
            // bool start_up;
            std::string name_space;
            std::string joint_name;
            urdf::JointConstSharedPtr joint_urdf;
            realtime_tools::RealtimeBuffer<laikago_msgs::MotorCmd> command;
            laikago_msgs::MotorCmd lastCmd;
            laikago_msgs::MotorState lastState;

            LaikagoJointController();
            ~LaikagoJointController();
            virtual bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
            virtual void starting(const ros::Time& time);
            virtual void update(const ros::Time& time, const ros::Duration& period);
            virtual void stopping();
            void setCommandCB(const laikago_msgs::MotorCmdConstPtr& msg);
            void jsCallback(const sensor_msgs::JointState::ConstPtr& msg);
            void positionLimits(double &position);
            void velocityLimits(double &velocity);
            void effortLimits(double &effort);
            void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);
            void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);
            void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
    };
}

#endif
