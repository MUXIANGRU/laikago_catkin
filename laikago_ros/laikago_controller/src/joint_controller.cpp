/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "laikago_controller/joint_controller.h"
#include "laikago_controller/laikago_control_tool.h"
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>


namespace laikago_controller {

    ServoCmd servoCmd;

    LaikagoJointController::LaikagoJointController(){}

    LaikagoJointController::~LaikagoJointController(){
        sub_cmd.shutdown();
    }

    void LaikagoJointController::setCommandCB(const laikago_msgs::MotorCmdConstPtr& msg)
    {
      //  ROS_ERROR("noting!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        lastCmd.mode = msg->mode;
        lastCmd.position = msg->position;
        lastCmd.positionStiffness = msg->positionStiffness;
        lastCmd.velocity = msg->velocity;
        lastCmd.velocityStiffness = msg->velocityStiffness;
        lastCmd.torque = msg->torque;
        // the writeFromNonRT can be used in RT, if you have the guarantee that
        //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
        //  * there is only one single rt thread
        command.writeFromNonRT(lastCmd);
    }
    void LaikagoJointController::jsCallback(const sensor_msgs::JointState::ConstPtr &msg){
        js_msg.name = msg->name;
        js_msg.effort = msg->effort;
        js_msg.header = msg->header;
        js_msg.position = msg->position;
        js_msg.velocity = msg->velocity;
    }

    // Controller initialization in non-realtime
    bool LaikagoJointController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
//    bool LaikagoJointController::init(hardware_interface::RobotStateInterface* hardware,
//                                      ros::NodeHandle& n)
    {
//        robot_state_data_.name = "base_controller";
//        robot_state_data_.position = position;
//        robot_state_data_.orientation = orinetation;
//        robot_state_data_.linear_velocity = linear_vel;
//        robot_state_data_.angular_velocity = angular_vel;
//        robot_state_data_.joint_position_read = pos_read;
//        robot_state_data_.joint_position_write = pos_write;
//        robot_state_data_.joint_velocity_read = vel_read;
//        robot_state_data_.joint_velocity_write = vel_write;
//        robot_state_data_.joint_effort_read = eff_read;
//        robot_state_data_.joint_effort_write = eff_write;
//        robot_state_data_.foot_contact = foot_contact;
//        robot_state_data_.contact_pressure = contact_pressure;
//        robot_state_data_.motor_status_word = motor_status_word;
//        robot_state_data_.mode_of_joint = mode_of_joint;
//        //! WSHY: registerhandle pass the data point to the hardwareResourseManager and then
//        //! the read() method update data which the pointer points to or write() the
//        //! updated commmand
//        robot_state_interface_.registerHandle(hardware_interface::RobotStateHandle(robot_state_data_));
        //robot_state_handle = hardware->getHandle("base_controller");
        name_space = n.getNamespace();
        if (!n.getParam("joint", joint_name)){
            ROS_ERROR("No joint given in namespace: '%s')", n.getNamespace().c_str());
            return false;
        }
        urdf::Model urdf; // Get URDF info about joint
        if (!urdf.initParamWithNodeHandle("robot_description", n)){
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }
        joint_urdf = urdf.getJoint(joint_name);
        if (!joint_urdf){
            ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
            return false;
        }
        joint = robot->getHandle(joint_name);
        // Start command subscriber
        sub_cmd = n.subscribe("command", 20, &LaikagoJointController::setCommandCB, this);
        js_sub_ = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, &LaikagoJointController::jsCallback, this);
 
        // Start realtime state publisher
        controller_state_publisher_.reset(
            new realtime_tools::RealtimePublisher<laikago_msgs::MotorState>(n, name_space + "/state", 1));        
        return true;
    }

    void LaikagoJointController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
    {
        pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
    }

    void LaikagoJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
    {
        pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
    }

    void LaikagoJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
    {
        bool dummy;
        pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
    }

    // Controller startup in realtime
    void LaikagoJointController::starting(const ros::Time& time)
    {
        ROS_INFO("STARTING!!!!!!");
        //double init_pos = joint.getPosition();//how to get joint_state
        double init_pos;
        if(joint.getName()=="LF_HAA"){
            init_pos = js_msg.position[0];
        }
        else if(joint.getName()=="LF_HFE"){
            init_pos = js_msg.position[1];
        }
        else if(joint.getName()=="LF_KFE"){
            init_pos = js_msg.position[2];
        }
        else if(joint.getName()=="RF_HAA"){
            init_pos = js_msg.position[6];
        }
        else if(joint.getName()=="RF_HFE"){
            init_pos = js_msg.position[7];
        }
        else if(joint.getName()=="RF_KFE"){
            init_pos = js_msg.position[8];
        }
        else if(joint.getName()=="LH_HAA"){
            init_pos = js_msg.position[3];
        }
        else if(joint.getName()=="LH_HFE"){
            init_pos = js_msg.position[4];
        }
        else if(joint.getName()=="LH_KFE"){
            init_pos = js_msg.position[5];
        }
        else if(joint.getName()=="RH_HAA"){
            init_pos = js_msg.position[9];
        }
        else if(joint.getName()=="RH_HFE"){
            init_pos = js_msg.position[10];
        }
        else if(joint.getName()=="RH_KFE"){
            init_pos = js_msg.position[11];
        }
        std::cout<<"init_pos   "<<init_pos<<std::endl;
        lastCmd.position = init_pos;
        lastState.position = init_pos;
        lastCmd.velocity = 0;
        lastState.velocity = 0;
        lastCmd.torque = 0;
        lastState.torque = 0;
        command.initRT(lastCmd);

        pid_controller_.reset();
    }

    // Controller update loop in realtime
    void LaikagoJointController::update(const ros::Time& time, const ros::Duration& period)
    {
        double currentPos, currentVel, calcTorque;
        lastCmd = *(command.readFromRT());


        servoCmd.pos = lastCmd.position;
        positionLimits(servoCmd.pos);
        servoCmd.posStiffness = lastCmd.positionStiffness;
        if(fabs(lastCmd.position - PosStopF) < 0.00001){
            servoCmd.posStiffness = 0;
        }
        servoCmd.vel = lastCmd.velocity;
        velocityLimits(servoCmd.vel);
        servoCmd.velStiffness = lastCmd.velocityStiffness;
        if(fabs(lastCmd.velocity - VelStopF) < 0.00001){
            servoCmd.velStiffness = 0;
        }
        servoCmd.torque = lastCmd.torque;
//        std::cout<<"))))))))))))))"<<std::endl;
//        std::cout<<servoCmd.posStiffness<<"  "<<servoCmd.velStiffness<<std::endl;


        if(joint.getName()=="LF_HAA"){
            currentPos = js_msg.position[0];
        }
        else if(joint.getName()=="LF_HFE"){
            currentPos = js_msg.position[1];
        }
        else if(joint.getName()=="LF_KFE"){
            currentPos = js_msg.position[2];
        }
        else if(joint.getName()=="RF_HAA"){
            currentPos = js_msg.position[6];
        }
        else if(joint.getName()=="RF_HFE"){
            currentPos = js_msg.position[7];
        }
        else if(joint.getName()=="RF_KFE"){
            currentPos = js_msg.position[8];
        }
        else if(joint.getName()=="LH_HAA"){
            currentPos = js_msg.position[3];
        }
        else if(joint.getName()=="LH_HFE"){
            currentPos = js_msg.position[4];
        }
        else if(joint.getName()=="LH_KFE"){
            currentPos = js_msg.position[5];
        }
        else if(joint.getName()=="RH_HAA"){
            currentPos = js_msg.position[9];
        }
        else if(joint.getName()=="RH_HFE"){
            currentPos = js_msg.position[10];
        }
        else if(joint.getName()=="RH_KFE"){
            currentPos = js_msg.position[11];
        }else{
            currentPos = joint.getPosition();
        }
        currentVel = computeVel(currentPos, (double)lastState.position, (double)lastState.velocity, period.toSec());
        calcTorque = computeTorque(currentPos, currentVel, servoCmd);
        //std::cout<<currentPos<<" "<<servoCmd.posStiffness<<" "<<currentVel<<" "<<servoCmd.velStiffness<<" "<<calcTorque<<std::endl;
        effortLimits(servoCmd.torque);
//                std::cout<<"+++++++++++++++++++++++++++++++++"<<std::endl;
//                std::cout<<calcTorque<<std::endl;
        effortLimits(calcTorque);

        joint.setCommand(calcTorque);
//        std::cout<<"+++++++++++++++++++++++++++++++++"<<std::endl;
//        //std::cout<<robot_state_handle.getPosition()[0]<<std::endl;
//        //std::cout<<robot_state_handle.getPosition()[1]<<std::endl;
//        //std::cout<<robot_state_handle.getPosition()[2]<<std::endl;
////        std::cout<<joint.getName()<<std::endl;
//        std::cout<<joint.getName()<<std::endl;
//        std::cout<<currentPos<<std::endl;

//////        std::cout<<joint.getEffort()<<std::endl;
//////        std::cout<<joint.getCommand()<<std::endl;
////        std::cout<<calcTorque<<std::endl;
//////        std::cout<<robot_state_data_.position[0]<<std::endl;
//////        std::cout<<robot_state_data_.position[1]<<std::endl;
//////        std::cout<<robot_state_data_.position[2]<<std::endl;
//        std::cout<<"+++++++++++++++++++++++++++++++++"<<std::endl;

        lastState.position = currentPos;
        lastState.velocity = currentVel;
        lastState.torque = calcTorque;
        if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
            controller_state_publisher_->msg_.position = lastState.position;
            controller_state_publisher_->msg_.velocity = lastState.velocity;
            controller_state_publisher_->msg_.torque = lastState.torque;
            controller_state_publisher_->unlockAndPublish();
        }
    }

    // Controller stopping in realtime
    void LaikagoJointController::stopping(){}

    void LaikagoJointController::positionLimits(double &position)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(position, joint_urdf->limits->lower, joint_urdf->limits->upper);
    }

    void LaikagoJointController::velocityLimits(double &velocity)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(velocity, -joint_urdf->limits->velocity, joint_urdf->limits->velocity);
    }

    void LaikagoJointController::effortLimits(double &effort)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(effort, -joint_urdf->limits->effort, joint_urdf->limits->effort);
    }

} // namespace

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(laikago_controller::LaikagoJointController, controller_interface::ControllerBase);
