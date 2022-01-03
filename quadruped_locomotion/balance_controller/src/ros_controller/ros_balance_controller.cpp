/*
 *  ros_balance_controller.cpp
 *  Descriotion:
 *
 *  Created on: Mar 18, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#include "balance_controller/ros_controler/ros_balance_controller.hpp"
#include "controller_manager/controller_manager.h"
namespace balance_controller{

  RosBalanceController::RosBalanceController()
  {
//    state_.reset();
//    robot_state_-
    log_length_ = 10000;
    log_index_ = log_length_;
    limbs_.push_back(free_gait::LimbEnum::LF_LEG);
    limbs_.push_back(free_gait::LimbEnum::RF_LEG);
    limbs_.push_back(free_gait::LimbEnum::LH_LEG);
    limbs_.push_back(free_gait::LimbEnum::RH_LEG);

    branches_.push_back(free_gait::BranchEnum::BASE);
    branches_.push_back(free_gait::BranchEnum::LF_LEG);
    branches_.push_back(free_gait::BranchEnum::RF_LEG);
    branches_.push_back(free_gait::BranchEnum::LH_LEG);
    branches_.push_back(free_gait::BranchEnum::RH_LEG);
    //! WSHY: initialize STAte
    robot_state_.reset(new free_gait::State);
    robot_state_->initialize(limbs_, branches_);
    robot_state.reset(new free_gait::State);
    robot_state->initialize(limbs_, branches_);


    for(auto limb : limbs_)
      {
        limbs_state[limb].reset(new StateSwitcher);
        limbs_state.at(limb)->initialize(0);
        limbs_last_state[limb].reset(new StateSwitcher);
        limbs_last_state.at(limb)->initialize(0);
        limbs_desired_state[limb].reset(new StateSwitcher);
        limbs_desired_state.at(limb)->initialize(0);
        t_sw0[limb] = ros::Time::now();
        t_st0[limb] = ros::Time::now();
        sw_phase[limb] = 0.0;
        st_phase[limb] = 0.0;
        sw_flag[limb] = false;
        st_flag[limb] = false;
        surface_normals[limb] = Vector(0,0,1);
        foot_positions[limb] = Vector(0,0,0);
        foot_velocities[limb] = Vector(0,0,0);
        foot_accelerations[limb] = Vector(0,0,0);
        stored_foot_positions[limb] = Vector(0,0,0);
        stored_current_foot_position_flag[limb] = false;
        store_current_joint_state_flag_[limb] = false;
        update_surface_normal_flag[limb] = false;
        real_contact_[limb] = false;
        real_contact_force_[limb] = Vector(0,0,0);
        is_cartisian_motion_[limb] = false;
        is_footstep_[limb] = false;
        is_legmode_[limb] = false;
      }
    for(int i=0;i<4;i++)
      {
        initial_pressure[i] = 0;
      }
//    store_current_joint_state_flag_ = false;
//    stored_limb_joint_position_.resize(3);

  };
  RosBalanceController::~RosBalanceController()
  {
    base_command_sub_.shutdown();
    contact_sub_.shutdown();
  };

  bool RosBalanceController::init(hardware_interface::RobotStateInterface* hardware,
                             ros::NodeHandle& node_handle)
  {
    ROS_INFO("Initializing RosBalanceController");
    //! WSHY: base balance QP controller
   // odom_ptr_.reset(new quadruped_odom::QuadrupedEstimation(node_handle,robot_state));
    contact_distribution_.reset(new ContactForceDistribution(node_handle, robot_state));
    virtual_model_controller_.reset(new VirtualModelController(node_handle, robot_state, contact_distribution_));
    node_handle.param("/my_log", my_log, false);
    //! WSHY: single leg controller
    single_leg_solver_.reset(new MyRobotSolver(node_handle, robot_state));
    single_leg_solver_->model_initialization();
    if(!single_leg_solver_->loadLimbModelFromURDF())
      {
        ROS_ERROR("Failed to load model from URDF file");
      }

    //! WSHY: Load parameters for VMC controller
    if(!contact_distribution_->loadParameters())
      {
        ROS_INFO("CFD load parameters failed");
      }
    if(!virtual_model_controller_->loadParameters())
      {
        ROS_INFO("VMC load parameters failed");
      }
    if(!single_leg_solver_->loadParameters())
      {
        ROS_INFO("SWC load parameters failed");
      }

    if(!node_handle.getParam("real_robot", real_robot))
      {
        ROS_ERROR("Can't find parameter of 'real_robot'");
        return false;
      }

    if(!node_handle.getParam("contact_pressure_bias", contact_pressure_bias))
      {
        ROS_ERROR("Can't find parameter of 'contact_pressure_bias'");
        return false;
      }

    if(!node_handle.getParam("ignore_contact_sensor", ignore_contact_sensor))
      {
        ROS_ERROR("Can't find parameter of 'ignore_contact_sensor'");
        return false;
      }
    if(!node_handle.getParam("log_data", log_data))
      {
        ROS_ERROR("Can't find parameter of 'log_data'");
        return false;
      }

    urdf::Model urdf;
    if (!urdf.initParam("/robot_description"))
    {
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }

    //! WSHY: get joint handle from robot state handle
    std::string param_name = "joints";
    if(!node_handle.getParam(param_name, joint_names))
      {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << node_handle.getNamespace() << ").");
        return false;
      }
    n_joints = joint_names.size();
    if(n_joints == 0){
          ROS_ERROR_STREAM("List of joint names is empty.");
          return false;
        }
    pid_controllers_.resize(n_joints);
    ROS_WARN("COMTING THERE!!!!!!!");
    for(unsigned int i = 0; i < n_joints; i++)
      {
        try {
//          joints.push_back(hardware->getHandle(joint_names[i]));
          joints.push_back(hardware->joint_effort_interfaces_.getHandle(joint_names[i]));
          //MXR::NOTE: In gazebo.urdf we have the jointstateinterface
          //position_joints.push_back(hardware->joint_position_interfaces_.getHandle(joint_names[i]));
          ROS_INFO("Get '%s' Handle", joint_names[i].c_str());
//          hardware->g
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
          ROS_ERROR_STREAM("Exception thrown : "<< ex.what());
          return false;
        }
        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names[i]);
        if (!joint_urdf)
        {
          ROS_ERROR("Could not find joint '%s' in urdf", joint_names[i].c_str());
          return false;
        }
        joint_urdfs_.push_back(joint_urdf);

        // Load PID Controller using gains set on parameter server
        if (!pid_controllers_[i].init(ros::NodeHandle(node_handle, joint_names[i] + "/pid")))
        {
          ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names[i] + "/pid");
          return false;
        }


      }


    ROS_INFO("Balance Controller to Get robot state handle");
    //! WSHY: get robot state handle
    robot_state_handle = hardware->getHandle("base_controller");

    ROS_INFO("Balance Controller initialized");
    for(unsigned int i=0;i<12;i++)
      {
        //ROS_INFO("Balance Controller coming there too");
        robot_state_handle.getJointEffortWrite()[i] = 0;
        robot_state_handle.motor_status_word_[i] = 0;

      }

    for(int i = 0;i<4;i++)
      {
        //ROS_INFO("Balance Controller coming there !!!!!!");
        robot_state_handle.foot_contact_[i] = 1;
        delay_counts[i] = 0;
      }

//    if(ignore_contact_sensor)
//      {
        for(int i =0;i<4;i++)
          {
            limbs_state.at(static_cast<free_gait::LimbEnum>(i))->setState(StateSwitcher::States::StanceNormal);
            robot_state->setSupportLeg(static_cast<free_gait::LimbEnum>(i), true);
          }
//      }
        int i = 0;
        ros::Duration delay(0.1);
        while(i<10)
          {
            for(int j =0;j<4;j++)
              {
                initial_pressure[j] += robot_state_handle.contact_pressure_[j];
              }
            delay.sleep();
            i++;
          }
        for(int i = 0;i<4;i++)
          {
            initial_pressure[i] = initial_pressure[i]/10;
            std::cout<<"Initial contact pressure for leg "<<i<<" is : "<<initial_pressure[i]<<std::endl;
          }

    ROS_INFO("Balance Controller coming there");
    commands_buffer.writeFromNonRT(std::vector<double>(n_joints, 0.0));

//    command_foot_buffer.writeFromNonRT(LimbVector({{free_gait::LimbEnum::LF_LEG, Vector(0,0,0)},
//                                                   {free_gait::LimbEnum::RF_LEG, Vector(0,0,0)},
//                                                   {free_gait::LimbEnum::RH_LEG, Vector(0,0,0)},
//                                                   {free_gait::LimbEnum::LH_LEG, Vector(0,0,0)}}));
//    command_foot_vel_buffer.writeFromNonRT(LimbVector({{free_gait::LimbEnum::LF_LEG, Vector(0,0,0)},
//                                                   {free_gait::LimbEnum::RF_LEG, Vector(0,0,0)},
//                                                   {free_gait::LimbEnum::RH_LEG, Vector(0,0,0)},
//                                                   {free_gait::LimbEnum::LH_LEG, Vector(0,0,0)}}));

//    Eigen::VectorXd joints_positions;
//    for(unsigned int i = 0; i<12; i++)
//      joints_positions =
//    single_leg_solver_->setvecQAct(Eigen::Vector3d(0,0,0), free_gait::LimbEnum::LF_LEG);
//    single_leg_solver_->setvecQAct(Eigen::Vector3d(0,0,0), free_gait::LimbEnum::RF_LEG);
//    single_leg_solver_->setvecQAct(Eigen::Vector3d(0,0,0), free_gait::LimbEnum::RH_LEG);
//    single_leg_solver_->setvecQAct(Eigen::Vector3d(0,0,0), free_gait::LimbEnum::LH_LEG);

    foot_vel.data.resize(16);
    foot_contact.data.resize(4);
    footVelPub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/foot_vel_norm",1);
    //velPub_ = node_handle.advertise<geometry_msgs::TwistStamped>("/estimated_vel",1);


    lf_contact_forceSub_ = node_handle.subscribe("/estimate_torque_lf",1,&RosBalanceController::lf_forceCB,this);
    rf_contact_forceSub_ = node_handle.subscribe("/estimate_torque_rf",1,&RosBalanceController::rf_forceCB,this);
    rh_contact_forceSub_ = node_handle.subscribe("/estimate_torque_rh",1,&RosBalanceController::rh_forceCB,this);
    lh_contact_forceSub_ = node_handle.subscribe("/estimate_torque_lh",1,&RosBalanceController::lh_forceCB,this);
    if(real_robot){
        contactPro_Sub_ = node_handle.subscribe("/legodom/contact_pro",1,&RosBalanceController::contactProCB,this);
    }else{
        contactPro_Sub_ = node_handle.subscribe("/contact_pro",1,&RosBalanceController::contactProCB,this);
    }


    log_data_srv_ = node_handle.advertiseService("/capture_log_data", &RosBalanceController::logDataCapture, this);

    base_command_sub_ = node_handle.subscribe<free_gait_msgs::RobotState>("/desired_robot_state", 1, &RosBalanceController::baseCommandCallback, this);
    //! WSHY: having problem with update foot contact in gazebo_state_hardware_interface, so
    //! use subscribe, for real hardware interface, there should no problem fro this.
    string contact_topic;
    if(real_robot)
      contact_topic = "/foot_contacts";
    else
      contact_topic = "/bumper_sensor_filter_node/foot_contacts";
    contact_sub_ = node_handle.subscribe<sim_assiants::FootContacts>(contact_topic, 1, &RosBalanceController::footContactsCallback, this);

    //! WSHY: Logged data publisher
    joint_command_pub_ = node_handle.advertise<sensor_msgs::JointState>("/balance_controller/joint_command", 1);
    base_command_pub_ = node_handle.advertise<nav_msgs::Odometry>("/log/base_command", log_length_);
    base_actual_pub_ = node_handle.advertise<nav_msgs::Odometry>("/log/base_actual", log_length_);
    leg_state_pub_ = node_handle.advertise<std_msgs::Int8MultiArray>("/log/leg_state", log_length_);
    joint_command_pub_ = node_handle.advertise<sensor_msgs::JointState>("/log/joint_command", log_length_);
    joint_actual_pub_ = node_handle.advertise<sensor_msgs::JointState>("/log/joint_state", log_length_);
    contact_desired_pub_ = node_handle.advertise<sim_assiants::FootContacts>("/log/desired_foot_contact", log_length_);
    leg_phase_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/log/leg_phase", log_length_);
    desired_robot_state_pub_ = node_handle.advertise<free_gait_msgs::RobotState>("/log/desired_robot_state", log_length_);
    actual_robot_state_pub_ = node_handle.advertise<free_gait_msgs::RobotState>("/log/actual_robot_state", log_length_);
    vmc_info_pub_ = node_handle.advertise<geometry_msgs::WrenchStamped>("/log/vmc_force_torque", log_length_);
    desired_vmc_info_pub_ = node_handle.advertise<geometry_msgs::WrenchStamped>("/log/desired_vmc_force_torque", log_length_);
    motor_status_word_pub_ = node_handle.advertise<std_msgs::Int8MultiArray>("/log/status_word", log_length_);

//    base_command_pub_ = node_handle.advertise<nav_msgs::Odometry>("/log/base_command", log_length_);
//    base_actual_pub_ = node_handle.advertise<nav_msgs::Odometry>("/log/base_actual", log_length_);
    pos_error_pub = node_handle.advertise<geometry_msgs::Pose>("/log/pos_error",log_length_);
    vel_error_pub = node_handle.advertise<geometry_msgs::Twist>("/log/twist_error",log_length_);
    ROS_INFO("Balance Controller initialized");
    foot_odom.pose.pose.position.x=0.0;
    foot_odom.pose.pose.position.y=0.0;
    foot_odom.pose.pose.position.z=0.084;
    footpos_delta.data.resize(12);
    return true;
  };
  /**
   * @brief RosBalanceController::update, controller update loop
   * @param time
   * @param period
   */
  void RosBalanceController::update(const ros::Time& time, const ros::Duration& period)
  {
//    double real_time_factor = 0.001/period.toSec();
//    ROS_WARN_STREAM("Real Time Factor :"<<real_time_factor<<std::endl);
    ROS_INFO_ONCE("Balance Controller Update Once");

    //! WSHY: for logging data
    sensor_msgs::JointState joint_command, joint_actual;
    joint_command.effort.resize(12);
    joint_command.position.resize(12);
    joint_command.name.resize(12);
    joint_actual.name.resize(12);
    joint_actual.position.resize(12);
    joint_actual.velocity.resize(12);
    joint_actual.effort.resize(12);

    //! WSHY: update joint state
    free_gait::JointPositions all_joint_positions;
    free_gait::JointVelocities all_joint_velocities;
    free_gait::JointEfforts all_joint_efforts;
    //! WSHY: get joint postions from robot state handle
    std_msgs::Int8MultiArray status_word;
    status_word.data.resize(12);
    //std::cout<<"I want to get all joint_positions!"<<std::endl;
    for(unsigned int i=0; i<12; i++)
      {
        all_joint_positions(i) = robot_state_handle.getJointPositionRead()[i];
        all_joint_velocities(i) = robot_state_handle.getJointVelocityRead()[i];
        all_joint_efforts(i) = robot_state_handle.getJointEffortRead()[i];
        joint_actual.position[i] = all_joint_positions(i);
        joint_actual.velocity[i] = all_joint_velocities(i);
        joint_actual.effort[i] = all_joint_efforts(i);
        status_word.data[i] = robot_state_handle.motor_status_word_[i];
        //std::cout<<all_joint_positions(i)<<" ";

      }
    //std::cout<<"++++++++++++++++++"<<std::endl;

    boost::recursive_mutex::scoped_lock lock(r_mutex_);
    //! WSHY: read joint position command
    std::vector<double> & commands = *commands_buffer.readFromRT();
     //std::cout<<"++++++++++++++++++"<<std::endl;
//     for(int i=0;i<commands.size();i++){
//         if(commands[i]!=0){
//             std::cout<<commands[i]<<std::endl;
//         }

//     }
     //std::cout<<"++++++++++++++++++"<<std::endl;
//    LimbVector & foot_posiion_commands = *command_foot_buffer.readFromRT();
//    LimbVector & foot_velocity_commands = *command_foot_vel_buffer.readFromRT();

    std_msgs::Int8MultiArray leg_state;
    std_msgs::Float64MultiArray leg_phase;
    leg_phase.data.resize(8);
    leg_state.data.resize(4);
    RotationQuaternion base_orinetation = RotationQuaternion(robot_state_handle.getOrientation()[0],
         robot_state_handle.getOrientation()[1],
         robot_state_handle.getOrientation()[2],
         robot_state_handle.getOrientation()[3]);
//    Vector surface_normal = base_orinetation.rotate(Vector(0,0,1));
    /****************
* TODO(Shunyao) : Rotate surface normals with base orinetation
****************/
    if(!ignore_contact_sensor)
      contactStateMachine();
    int num_of_stance_legs = 0;
    for(unsigned int i=0;i<4;i++)
      {
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
        //! WSHY: set foot cartesian motion for single leg controller
        robot_state->setTargetFootPositionInBaseForLimb(Position(foot_positions.at(limb).vector()), limb);
        robot_state->setTargetFootVelocityInBaseForLimb(LinearVelocity(foot_velocities.at(limb).vector()), limb);
        single_leg_solver_->setvecQAct(all_joint_positions.vector().segment(3*i, 3), limb);
        single_leg_solver_->setvecQDotAct(all_joint_velocities.vector().segment(3*i, 3), limb);
        //! WSHY: Without foot contact sensor node
        if(ignore_contact_sensor)
          {/*
            limbs_state.at(limb)->setState(StateSwitcher::States::SwingNormal);
            if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::StanceNormal && st_phase.at(limb)>0.1)*/
              limbs_state.at(limb)->setState(limbs_desired_state.at(limb)->getState());
          }

        switch (limbs_state.at(limb)->getState()) {
          case StateSwitcher::States::SwingNormal:
            {
              delay_counts[i] = 0;
              robot_state_handle.foot_contact_[i] = 0;
              robot_state->setSupportLeg(limb, false);
              //            surface_normals.at(limb) = base_orinetation.rotate(Vector(0,0,1));
              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
              //            robot_state->setSurfaceNormal(limb, robot_state_->getSurfaceNormal(limb));
              leg_state.data[i] = 0;
              store_current_joint_state_flag_.at(limb) = false;
              update_surface_normal_flag.at(limb) = false;
              stored_current_foot_position_flag.at(limb) = false;
//              ROS_INFO("Leg '%d' is in SwingNormal mode", i);
              break;
            }
          case StateSwitcher::States::StanceNormal:
            {
              num_of_stance_legs++;
              delay_counts[i]++;
//              robot_state->setSupportLeg(limb, true);
              //! WSHY: delay for base velocity estimate
              leg_state.data[i] = 1;
//              robot_state_handle.foot_contact_[i] = 0;
              if(ignore_contact_sensor)
                {

                  if(st_phase.at(limb)>0.2)
                    {

                      robot_state_handle.foot_contact_[i] = 1;
                      leg_state.data[i] = 2;
                    }
                } else {
                  robot_state->setSupportLeg(limb, true);
                  leg_state.data[i] = 2;
//                  if(delay_counts[i]>30) //for trot
                  if(delay_counts[i]>5)
                    {

                      robot_state_handle.foot_contact_[i] = 1;

                    }
//                  if(st_phase.at(limb)>0.8)
//                    {

//                      robot_state_handle.foot_contact_[i] = 0;
//                    }

                }


              //            if(!update_surface_normal_flag.at(limb))
              //              {
              //                update_surface_normal_flag.at(limb) = true;
              //                Eigen::Matrix3d jacobian = robot_state_->getTranslationJacobianFromBaseToFootInBaseFrame(limb);
              //                free_gait::JointEffortsLeg joint_effort = free_gait::JointEffortsLeg(all_joint_efforts.vector().segment(i*3, 3));
              //                free_gait::Force contact_force = free_gait::Force(jacobian * joint_effort.toImplementation());
              //                surface_normals.at(limb) = Vector(contact_force.normalize());
              //                if(limb == free_gait::LimbEnum::LF_LEG || limb == free_gait::LimbEnum::RH_LEG)
              //                  surface_normals.at(limb) = Vector(-contact_force.normalize());
              //              }
              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
              //            robot_state->setSurfaceNormal(limb, robot_state_->getSurfaceNormal(limb));
//              leg_state.data[i] = 1;
              store_current_joint_state_flag_.at(limb) = false;
//              ROS_INFO("Leg '%d' is in StanceNormal mode", i);
              break;
            }
          case StateSwitcher::States::SwingEarlyTouchDown:
            {
              robot_state_handle.foot_contact_[i] = 2;
//              delay_counts[i]++;
//              if(delay_counts[i]==10)
//                {
//                  delay_counts[i] = 0;
//                  limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
//                }
//              robot_state->setSupportLeg(limb, true);
              if(!stored_current_foot_position_flag.at(limb))
                {
                  stored_current_foot_position_flag.at(limb) = true;
                  stored_foot_positions.at(limb) = foot_positions.at(limb);
                }else{
                  robot_state_->setTargetFootPositionInBaseForLimb(Position(stored_foot_positions.at(limb)),limb);
                  robot_state->setSupportLeg(limb, false);
                }
//              robot_state->setSupportLeg(limb, true);
              //              if(!update_surface_normal_flag.at(limb))
              //                {
              //                  update_surface_normal_flag.at(limb) = true;
              //                  Eigen::Matrix3d jacobian = robot_state_->getTranslationJacobianFromBaseToFootInBaseFrame(limb);
              //                  free_gait::JointEffortsLeg joint_effort = free_gait::JointEffortsLeg(all_joint_efforts.vector().segment(i*3, 3));
              //                  free_gait::Force contact_force = free_gait::Force(jacobian * joint_effort.toImplementation());
              //                  surface_normals.at(limb) = Vector(contact_force.normalize());
              //                  if(limb == free_gait::LimbEnum::LF_LEG || limb == free_gait::LimbEnum::RH_LEG)
              //                    surface_normals.at(limb) = Vector(-contact_force.normalize());
              //                }
              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
              //              robot_state->setSurfaceNormal(limb, robot_state_->getSurfaceNormal(limb));
              //! WSHY: keep end effort position when early touch down
//              ROS_WARN("Leg '%d' is in SwingEarlyTouchDown mode", i);
              leg_state.data[i] = 1;
              break;
            }
          case StateSwitcher::States::SwingBumpedIntoObstacle:
            {
              leg_state.data[i] = 4;
              robot_state_handle.foot_contact_[i] = 4;
              robot_state->setSupportLeg(limb, false);
              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
              LinearVelocity desired_velocity_in_base = base_orinetation.inverseRotate(base_desired_linear_velocity);

//              LinearVelocity current_velocity_in_base = base_orinetation.inverseRotate(LinearVelocity(robot_state_handle.getLinearVelocity()[0],
//                                                                    robot_state_handle.getLinearVelocity()[1],
//                                                                    robot_state_handle.getLinearVelocity()[2]));
              //! WSHY: move back(-x) and upward(+z)
              foot_positions.at(limb)(0) -= 0.005;
              foot_positions.at(limb)(2) += 0.02;
//              foot_positions.at(limb).x() -= desired_velocity_in_base.x()*period.toSec();
//              foot_positions.at(limb).y() -= desired_velocity_in_base.y()*period.toSec();
              std::cout<<"desired velocity in base is : "<<desired_velocity_in_base<<std::endl;
              robot_state->setTargetFootPositionInBaseForLimb(Position(foot_positions.at(limb).vector()), limb);

              break;
            }
          case StateSwitcher::States::SwingLatelyTouchDown:
            {
              LinearVelocity current_velocity_in_base = base_orinetation.inverseRotate(LinearVelocity(robot_state_handle.getLinearVelocity()[0],
                                                                    robot_state_handle.getLinearVelocity()[1],
                                                                    robot_state_handle.getLinearVelocity()[2]));
              LinearVelocity desired_velocity_in_base = base_orinetation.inverseRotate(base_desired_linear_velocity);
              robot_state_handle.foot_contact_[i] = 3;
              /****************
            * TODO(Shunyao) : Directly move down?
            ****************/

              if(!stored_current_foot_position_flag.at(limb))
                {
                  stored_current_foot_position_flag.at(limb) = true;
                  stored_foot_positions.at(limb) = foot_positions.at(limb);
                }else{
                  stored_foot_positions.at(limb).z() -= 0.005;
                  stored_foot_positions.at(limb).x() += desired_velocity_in_base.x()*period.toSec();
                  stored_foot_positions.at(limb).y() += desired_velocity_in_base.y()*period.toSec();
                  if(stored_foot_positions.at(limb).z()<-0.6)
                    stored_foot_positions.at(limb).z() = -0.6;
                  robot_state_->setTargetFootPositionInBaseForLimb(Position(stored_foot_positions.at(limb)),limb);
                  robot_state->setSupportLeg(limb, false);
                  foot_positions.at(limb) = stored_foot_positions.at(limb);
                }

              robot_state->setSupportLeg(limb, false);
              //            surface_normals.at(limb) = base_orinetation.rotate(Vector(0,0,1));
              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
              //! WSHY: directly move down to ground
//              foot_positions.at(limb).z() -= 0.005;
//              foot_positions.at(limb).x() += desired_velocity_in_base.x()*period.toSec();
//              foot_positions.at(limb).y() += desired_velocity_in_base.y()*period.toSec();
//              std::cout<<"position diff X "<<current_velocity_in_base.x()*period.toSec()<<std::endl;

//              robot_state->setTargetFootPositionInBaseForLimb(Position(foot_positions.at(limb).vector()), limb);
              //            robot_state->setSurfaceNormal(limb, robot_state_->getSurfaceNormal(limb));
              //! WSHY: keep end effort position when early touch down
              if(!store_current_joint_state_flag_.at(limb)){
                  store_current_joint_state_flag_.at(limb) = true;
                  //                stored_limb_joint_position_[0] = robot_state_handle.getJointPositionRead()[3*i];
                  //                stored_limb_joint_position_[1] = robot_state_handle.getJointPositionRead()[3*i + 1];
                  //                stored_limb_joint_position_[2] = robot_state_handle.getJointPositionRead()[3*i + 2];
                  stored_limb_joint_position_.vector().segment(3*i,3) = all_joint_positions.vector().segment(3*i,3);
                } else {
                  commands[3*i] = stored_limb_joint_position_(3*i);
                  commands[3*i + 1] = stored_limb_joint_position_(3*i+1);
                  commands[3*i + 2] = stored_limb_joint_position_(3*i+2);
                }
//              ROS_WARN("Leg '%d' is in SwingLatelyTouchDown mode", i);
              leg_state.data[i] = 3;
              break;
            }
          case StateSwitcher::States::StanceLostContact:
            {
              robot_state_handle.foot_contact_[i] = 0;
              robot_state->setSupportLeg(limb, false);
              //! WSHY: directly move down to ground
              foot_positions.at(limb)(2) -= 0.01;
              robot_state->setTargetFootPositionInBaseForLimb(Position(foot_positions.at(limb).vector()), limb);

              //            surface_normals.at(limb) = base_orinetation.rotate(Vector(0,0,1));
              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
              //            robot_state->setSurfaceNormal(limb, robot_state_->getSurfaceNormal(limb));
              //! WSHY: keep end effort position when early touch down
              if(!store_current_joint_state_flag_.at(limb)){
                  store_current_joint_state_flag_.at(limb) = true;
                  //                stored_limb_joint_position_[0] = robot_state_handle.getJointPositionRead()[3*i];
                  //                stored_limb_joint_position_[1] = robot_state_handle.getJointPositionRead()[3*i + 1];
                  //                stored_limb_joint_position_[2] = robot_state_handle.getJointPositionRead()[3*i + 2];
                  stored_limb_joint_position_.vector().segment(3*i,3) = all_joint_positions.vector().segment(3*i,3);
                } else {
                  commands[3*i] = stored_limb_joint_position_(3*i);
                  commands[3*i + 1] = stored_limb_joint_position_(3*i+1);
                  commands[3*i + 2] = stored_limb_joint_position_(3*i+2);
                }
              ROS_WARN("Leg '%d' is Lost Contact!!!", i);
              leg_state.data[i] = -2;
              break;
            }
          case StateSwitcher::States::Init:
            {
              robot_state->setSupportLeg(limb, true);
              //            surface_normals.at(limb) = base_orinetation.rotate(Vector(0,0,1));
              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
              //            robot_state->setSurfaceNormal(limb, Vector(0, 0, 1));
              ROS_WARN("Leg '%d' is Init", i);
              break;
            }
          case StateSwitcher::States::SwingLateLiftOff:
            {
              robot_state_handle.foot_contact_[i] = 0;
              leg_state.data[i] = -1;
              robot_state->setSupportLeg(limb, false);
              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
//              ROS_WARN("Leg '%d' is in SwingLateLiftOff mode", i);
              break;
            }
          default:
            ROS_WARN("Unspecificed Limb State");

          }
//        std::cout<<"surface normal for leg "<<i<<"("<<surface_normals.at(limb)<<")"<<std::endl;

//        robot_state->setSupportLeg(limb, true);
//       RotationQuaternion base_orinetation = RotationQuaternion(robot_state_handle.getOrientation()[0],
//            robot_state_handle.getOrientation()[1],
//            robot_state_handle.getOrientation()[2],
//            robot_state_handle.getOrientation()[3]);
        robot_state->setSurfaceNormal(limb, base_orinetation.rotate(Vector(0,0,1)));
//        robot_state->setSurfaceNormal(limb, Vector(0,0,1));
      }

    lock.unlock();
    //! WSHY: set the desired state
    robot_state->setPositionWorldToBaseInWorldFrame(base_desired_position);
    robot_state->setOrientationBaseToWorld(base_desired_rotation);
    robot_state->setLinearVelocityBaseInWorldFrame(base_desired_linear_velocity);
    robot_state->setAngularVelocityBaseInBaseFrame(base_desired_angular_velocity);



    //! WSHY: update current base state from robot state handle
    robot_state->setCurrentLimbJoints(all_joint_positions);
    robot_state->setCurrentLimbJointVelocities(all_joint_velocities);

    // ROS_ERROR("State Estimate Position in X : %f",robot_state_handle.getPosition()[0]);
    // ROS_ERROR("State Estimate Position in Y : %f",robot_state_handle.getPosition()[1]);
    // ROS_ERROR("State Estimate Position in Z : %f",robot_state_handle.getPosition()[2]);
    Pose current_base_pose = Pose(Position(robot_state_handle.getPosition()[0],
                                  robot_state_handle.getPosition()[1],
                                  robot_state_handle.getPosition()[2]),
                         RotationQuaternion(robot_state_handle.getOrientation()[0],
                                            robot_state_handle.getOrientation()[1],
                                            robot_state_handle.getOrientation()[2],
                                            robot_state_handle.getOrientation()[3]));

    robot_state->setPoseBaseToWorld(current_base_pose);
    robot_state->setBaseStateFromFeedback(LinearVelocity(robot_state_handle.getLinearVelocity()[0],
                                                          robot_state_handle.getLinearVelocity()[1],
                                                          robot_state_handle.getLinearVelocity()[2]),
                                           LocalAngularVelocity(robot_state_handle.getAngularVelocity()[0],
                                                                robot_state_handle.getAngularVelocity()[1],
                                                                robot_state_handle.getAngularVelocity()[2]));


    /****************
* TODO(Shunyao) : set support leg and surface normal, update from robot state handle
****************/

    //! WSHY: compute joint torque
    bool keep_flag = false;
    if(!virtual_model_controller_->compute())
      {
        ROS_ERROR("VMC compute failed");
        ROS_WARN_STREAM(*virtual_model_controller_);
        keep_flag = true;
      }
//    ROS_WARN_STREAM("norminal : "<<contact_distribution_->getNormalDirectionOfFrictionPyramidInWorldFrame(free_gait::LimbEnum::LF_LEG).toImplementation()<<std::endl);
//    ROS_WARN_STREAM("First : "<<contact_distribution_->getFirstDirectionOfFrictionPyramidInWorldFrame(free_gait::LimbEnum::LF_LEG).toImplementation()<<std::endl);
//    ROS_WARN_STREAM("Second : "<<contact_distribution_->getSecondDirectionOfFrictionPyramidInWorldFrame(free_gait::LimbEnum::LF_LEG).toImplementation()<<std::endl);


//    ROS_DEBUG_STREAM(*virtual_model_controller_);

//    free_gait::JointEfforts all_joint_torque_command = robot_state->getAllJointEfforts();
//    for(int i =0;i<3;i++)
//      {
//        free_gait::JointEffortsLeg joint_torque_limb = robot_state->getJointEffortsForLimb(static_cast<free_gait::LimbEnum>(i));
//        jointTorquesLimit(joint_torque_limb, 60.0);
//        robot_state->setJointEffortsForLimb(static_cast<free_gait::LimbEnum>(i), joint_torque_limb);
//      }
    //std::cout<<"计算出来的关节力矩: "<<std::endl;
    for(int i = 0; i<4; i++)
      {
        /****************
         * TODO(Shunyao) :  decide support leg to apply joint torque
         ****************/
//        if(robot_state_)
//        double joint_torque_command = robot_state->getAllJointEfforts()(i);

        //MXR::NOTE:力分配后得到的关节力矩再通过handle读取!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        free_gait::JointEffortsLeg joint_torque_limb = robot_state->getJointEffortsForLimb(static_cast<free_gait::LimbEnum>(i));
//        for(int k=0;k<3;k++)
//          joint_command.effort[i*3 + k] = joint_torque_limb(k);
//        jointTorquesLimit(joint_torque_limb, 60.0);
        int start_index = i*3;
        for(int j =0;j<3;j++)
          {
            double joint_torque_command = joint_torque_limb(j);
            int index = start_index + j;

            ROS_DEBUG("Torque computed of joint %d is : %f\n",index,joint_torque_command);

            robot_state_handle.getJointEffortWrite()[index] = joint_torque_command;
            robot_state_handle.mode_of_joint_[i] = 4;




            //        if(joint_torque_command>300)
            //          joint_torque_command = 300;
            //        if(joint_torque_command<-300)
            //          joint_torque_command = -300;

            //        if(!keep_flag)
            //          {
            //            joints[i].setCommand(joint_torque_command);
            //          }
            joint_command.name[index] = joint_names[index];
//            if(num_of_stance_legs>1)
            joint_command.effort[index] = joint_torque_command;
            joint_command.position[index] = commands[index];
            joint_actual.name[index] = joint_names[index];

           // std::cout<<"@@@@@@@@@@@@@@  "<<joint_torque_command<<std::endl;
//            joint_actual.position[index] = all_joint_positions(index);
//            joint_actual.velocity[index] = all_joint_velocities(index);
//            joint_actual.effort[index] = all_joint_efforts(index);
          }
      }
//    joint_command_pub_.publish(joint_command);
    //! WSHY: for Swing Leg Control
//    std::vector<double> & commands = *commands_buffer.readFromRT();
    ros::Duration real_time_period = ros::Duration(period.toSec());
    free_gait::Force gravity_in_base = base_orinetation.rotate(free_gait::Force(0,0,-9.8));
    if(!robot_state->isSupportLeg(free_gait::LimbEnum::LF_LEG))
      {
        //ROS_INFO("LF_LEG is NOT Contacted");
        //! WSHY: compute gravity compensation
        free_gait::JointPositionsLeg joint_position_leg = free_gait::JointPositionsLeg(all_joint_positions.vector().segment(0,3));
        free_gait::JointEffortsLeg gravity_compensation_torque = robot_state->getGravityCompensationForLimb(free_gait::LimbEnum::LF_LEG,
                                                                                joint_position_leg,
                                                                                gravity_in_base);
//        single_leg_solver_->setvecQAct(joint_position_leg.vector(),free_gait::LimbEnum::LF_LEG);
        single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::LF_LEG,
                                   real_robot, foot_accelerations.at(free_gait::LimbEnum::LF_LEG).vector());

        for(int i = 0;i<3;i++)
          {
//MXR::NOTE:  THE Command[i] seems to be not used (set 0)
            double effort_command = computeTorqueFromPositionCommand(commands[i], i, real_time_period);
//MXR::NOTE:  The effort_command is too large!!
//            std::cout<<"%%%%%%%%%%%%%%%%%%%%"<<std::endl;
//            std::cout<<"effort_command_FOR LF_LEG   "<<effort_command<<std::endl;
//            std::cout<<"%%%%%%%%%%%%%%%%%%%%"<<std::endl;
            if(is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) || is_footstep_.at(free_gait::LimbEnum::LF_LEG)){
              effort_command = single_leg_solver_->getVecTauAct()[i];
//              std::cout<<"%%%%%%%%%%%%%%%%%%%%"<<std::endl;
//              std::cout<<"effort_command_FOR LF_LEG   "<<effort_command<<std::endl;
//              std::cout<<"%%%%%%%%%%%%%%%%%%%%"<<std::endl;
                                                }
            else if(!is_legmode_.at(free_gait::LimbEnum::LF_LEG))
              {
                //! WSHY: joint control mode, use Profile Position mode;
                robot_state_handle.mode_of_joint_[i] = 1;
                if(!real_robot)
                  effort_command += gravity_compensation_torque(i);
                position_joints[i].setCommand(commands[i]);
                ROS_INFO("Joint control");

              }
            else{

              effort_command = gravity_compensation_torque(i);}

            joints[i].setCommand(effort_command);
            //! WSHY: data logging
            joint_command.effort[i] = effort_command;
//            joint_actual.position[i] = single_leg_solver_->getQAcutal().row(1)[i];
//            joint_actual.velocity[i] = single_leg_solver_->getQDotAcutal().row(1)[i];
//            joint_actual.effort[i] = single_leg_solver_->getQDDotAcutal().row(1)[i];
          }
        //        ROS_INFO("LF_LEG is NOT Contacted");
      }else{ // for stance leg
        for(unsigned int i=0;i<3;i++)
          {
            joints[i].setCommand(joint_command.effort[i]);
          }
      }
    if(!robot_state->isSupportLeg(free_gait::LimbEnum::RF_LEG))
      {
        //ROS_INFO("RF_LEG is NOT Contacted");
        free_gait::JointPositionsLeg joint_position_leg = free_gait::JointPositionsLeg(all_joint_positions.vector().segment(3,3));
        free_gait::JointEffortsLeg gravity_compensation_torque = robot_state->getGravityCompensationForLimb(free_gait::LimbEnum::RF_LEG,
                                                                                joint_position_leg,
                                                                                gravity_in_base);

//        single_leg_solver_->setvecQAct(joint_position_leg.vector(),free_gait::LimbEnum::RF_LEG);
//        single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::RF_LEG);
        single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::RF_LEG,
                                   real_robot, foot_accelerations.at(free_gait::LimbEnum::RF_LEG).vector());
        for(int i = 0;i<3;i++)
          {
            double effort_command = computeTorqueFromPositionCommand(commands[i+3], i+3, real_time_period);
            if(is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) || is_footstep_.at(free_gait::LimbEnum::RF_LEG)){
              effort_command = single_leg_solver_->getVecTauAct()[i];
            }
            else if(!is_legmode_.at(free_gait::LimbEnum::RF_LEG))
              {
                //! WSHY: joint control mode, use Profile Position mode;
                robot_state_handle.mode_of_joint_[i+3] = 1;
                if(!real_robot)
                  effort_command += gravity_compensation_torque(i);
                position_joints[i+3].setCommand(commands[i+3]);
                ROS_INFO("Joint control");
              }
            else
              effort_command = gravity_compensation_torque(i);
            joints[i+3].setCommand(effort_command);
            joint_command.effort[i+3] = effort_command;

//            joint_actual.position[i+3] = single_leg_solver_->getQAcutal().row(1)[i];
//            joint_actual.velocity[i+3] = single_leg_solver_->getQDotAcutal().row(1)[i];
//            joint_actual.effort[i+3] = single_leg_solver_->getTauAcutal().row(0)[i]; //single_leg_solver_->getQDDotAcutal().row(1)[i];
          }
//                ROS_INFO("RF_LEG is NOT Contacted");
      }else{
        for(unsigned int i=0;i<3;i++)
          {
            joints[i+3].setCommand(joint_command.effort[i+3]);
          }
      }
    if(!robot_state->isSupportLeg(free_gait::LimbEnum::RH_LEG))
      {

        //ROS_INFO("RH_LEG is NOT Contacted");

        free_gait::JointPositionsLeg joint_position_leg = free_gait::JointPositionsLeg(all_joint_positions.vector().segment(6,3));
        free_gait::JointEffortsLeg gravity_compensation_torque = robot_state->getGravityCompensationForLimb(free_gait::LimbEnum::RH_LEG,
                                                                                joint_position_leg,
                                                                                gravity_in_base);

//        single_leg_solver_->setvecQAct(joint_position_leg.vector(),free_gait::LimbEnum::RH_LEG);
//        single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::RH_LEG);
//        std::cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$"<<std::endl;
//        std::cout<<"foot_accelerations.at(free_gait::LimbEnum::RH_LEG).vector()  "<<foot_accelerations.at(free_gait::LimbEnum::RH_LEG).vector()<<std::endl;
//        std::cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$"<<std::endl;
        single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::RH_LEG,
                                   real_robot, foot_accelerations.at(free_gait::LimbEnum::RH_LEG).vector());
        for(int i = 0;i<3;i++)
          {
//            std::cout<<"%%%%%%%%%%%%%%%%%%%%"<<std::endl;
//            std::cout<<"position_command_RH   "<<commands[i]<<std::endl;
//            std::cout<<"%%%%%%%%%%%%%%%%%%%%"<<std::endl;
            double effort_command =computeTorqueFromPositionCommand(commands[i+6], i+6, real_time_period);
            if(is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) || is_footstep_.at(free_gait::LimbEnum::RH_LEG)){
                //ROS_WARN("first branch!!!!!!!!!!!!!!");
              effort_command = single_leg_solver_->getVecTauAct()[i];
//              std::cout<<"%%%%%%%%%%%%%%%%%%%%"<<std::endl;
//              std::cout<<"effort_command_FOR RH_LEG   "<<effort_command<<std::endl;
//              std::cout<<"%%%%%%%%%%%%%%%%%%%%"<<std::endl;
            }
            else if(!is_legmode_.at(free_gait::LimbEnum::RH_LEG))
              {
                //("second branch!!!!!!!!!!!!!!");
                //! WSHY: joint control mode, use Profile Position mode;
                robot_state_handle.mode_of_joint_[i+6] = 1;
                if(!real_robot)
                  effort_command += gravity_compensation_torque(i);
                position_joints[i+6].setCommand(commands[i+6]);
                ROS_INFO("Joint control");
              }
            else{
                //ROS_WARN("third branch!!!!!!!!!!!!!!");
              effort_command = gravity_compensation_torque(i);}
            joints[i+6].setCommand(effort_command);
            //joints[i+6].getCommand();
            joint_command.effort[i+6] = effort_command;

//            joint_actual.position[i+6] = single_leg_solver_->getQAcutal().row(1)[i];
//            joint_actual.velocity[i+6] = single_leg_solver_->getQDotAcutal().row(1)[i];
//            joint_actual.effort[i+6] = single_leg_solver_->getTauAcutal().row(0)[i]; //single_leg_solver_->getQDDotAcutal().row(1)[i];
          }
//                ROS_INFO("RH_LEG is NOT Contacted");
      }else{
        for(unsigned int i=0;i<3;i++)
          {
            joints[i+6].setCommand(joint_command.effort[i+6]);
          }
      }
    if(!robot_state->isSupportLeg(free_gait::LimbEnum::LH_LEG))
      {
        free_gait::JointPositionsLeg joint_position_leg = free_gait::JointPositionsLeg(all_joint_positions.vector().segment(9,3));
        free_gait::JointEffortsLeg gravity_compensation_torque = robot_state->getGravityCompensationForLimb(free_gait::LimbEnum::LH_LEG,
                                                                                joint_position_leg,
                                                                                gravity_in_base);
//        single_leg_solver_->setvecQAct(joint_position_leg.vector(),free_gait::LimbEnum::LH_LEG);
//        single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::LH_LEG);
        single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::LH_LEG,
                                   real_robot, foot_accelerations.at(free_gait::LimbEnum::LH_LEG).vector());

        for(int i = 0;i<3;i++)
          {
            double effort_command =computeTorqueFromPositionCommand(commands[i+9], i+9, real_time_period);
            if(is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) || is_footstep_.at(free_gait::LimbEnum::LH_LEG)){
              effort_command = single_leg_solver_->getVecTauAct()[i];
            }
            else if(!is_legmode_.at(free_gait::LimbEnum::LH_LEG))
              {
                //! WSHY: joint control mode, use Profile Position mode;
                robot_state_handle.mode_of_joint_[i+9] = 1;
                if(!real_robot)
                  effort_command += gravity_compensation_torque(i);
                position_joints[i+9].setCommand(commands[i+9]);
                ROS_INFO("Joint control");
              }
            else
              effort_command = gravity_compensation_torque(i);
            joints[i+9].setCommand(effort_command);
            joint_command.effort[i+9] = effort_command;

//            joint_actual.position[i+9] = single_leg_solver_->getQAcutal().row(1)[i];
//            joint_actual.velocity[i+9] = single_leg_solver_->getQDotAcutal().row(1)[i];
//            joint_actual.effort[i+9] = single_leg_solver_->getTauAcutal().row(0)[i]; //single_leg_solver_->getQDDotAcutal().row(1)[i];
          }
//                ROS_INFO("LH_LEG is NOT Contacted");
      }else{
        for(unsigned int i=0;i<3;i++)
          {
            joints[i+9].setCommand(joint_command.effort[i+9]);
          }
      }
//    lock.unlock();

//    base_orientation=robot_state->getOrientationBaseToWorld();
//    //cout<<"rotation    "<<base_orientation<<endl;
//    Eigen::Quaterniond orient;
//    orient.x()=base_orinetation.x();
//    orient.y()=base_orinetation.y();
//    orient.z()=base_orinetation.z();
//    orient.w()=base_orinetation.w();
//    Eigen::Matrix3d rotation_matrix_trans = orient.normalized().toRotationMatrix();
//    P_LF.x()=robot_state->getPoseFootInBaseFrame(free_gait::LimbEnum::LF_LEG).getPosition().x();
//    P_LF.y()=robot_state->getPoseFootInBaseFrame(free_gait::LimbEnum::LF_LEG).getPosition().y();
//    P_LF.z()=robot_state->getPoseFootInBaseFrame(free_gait::LimbEnum::LF_LEG).getPosition().z();
//    P_RF.x()=robot_state->getPoseFootInBaseFrame(free_gait::LimbEnum::RF_LEG).getPosition().x();
//    P_RF.y()=robot_state->getPoseFootInBaseFrame(free_gait::LimbEnum::RF_LEG).getPosition().y();
//    P_RF.z()=robot_state->getPoseFootInBaseFrame(free_gait::LimbEnum::RF_LEG).getPosition().z();
//    P_LH.x()=robot_state->getPoseFootInBaseFrame(free_gait::LimbEnum::LH_LEG).getPosition().x();
//    P_LH.y()=robot_state->getPoseFootInBaseFrame(free_gait::LimbEnum::LH_LEG).getPosition().y();
//    P_LH.z()=robot_state->getPoseFootInBaseFrame(free_gait::LimbEnum::LH_LEG).getPosition().z();
//    P_RH.x()=robot_state->getPoseFootInBaseFrame(free_gait::LimbEnum::RH_LEG).getPosition().x();
//    P_RH.y()=robot_state->getPoseFootInBaseFrame(free_gait::LimbEnum::RH_LEG).getPosition().y();
//    P_RH.z()=robot_state->getPoseFootInBaseFrame(free_gait::LimbEnum::RH_LEG).getPosition().z();

//    POSE_IN_WORLD.x()=foot_odom.pose.pose.position.x;
//    POSE_IN_WORLD.y()=foot_odom.pose.pose.position.y;
//    POSE_IN_WORLD.z()=foot_odom.pose.pose.position.z;
//    //POSE_IN_WORLD<<foot_odom.pose.pose.position.x,foot_odom.pose.pose.position.y,foot_odom.pose.pose.position.z;
//    cout<<"POSE_IN_WORLD   "<<POSE_IN_WORLD<<endl;
//    //cout<<rotation_matrix_trans*P_LF<<endl;
//    P_LF_r=rotation_matrix_trans*P_LF;
//    P_LF_w=rotation_matrix_trans*P_LF+POSE_IN_WORLD;
////    P_LF_w.x()+=foot_odom.pose.pose.position.x;
////    P_LF_w.y()+=foot_odom.pose.pose.position.y;
////    P_LF_w.z()+=foot_odom.pose.pose.position.z;
//    P_RF_r=rotation_matrix_trans*P_RF;
//    P_RF_w=rotation_matrix_trans*P_RF+POSE_IN_WORLD;
////    P_RF_w.x()+=foot_odom.pose.pose.position.x;
////    P_RF_w.y()+=foot_odom.pose.pose.position.y;
////    P_RF_w.z()+=foot_odom.pose.pose.position.z;
//    P_RH_r=rotation_matrix_trans*P_RH;
//    P_RH_w=rotation_matrix_trans*P_RH+POSE_IN_WORLD;
////    P_RH_w.x()+=foot_odom.pose.pose.position.x;
////    P_RH_w.y()+=foot_odom.pose.pose.position.y;
////    P_RH_w.z()+=foot_odom.pose.pose.position.z;
//    P_LH_r=rotation_matrix_trans*P_LH;
//    P_LH_w=rotation_matrix_trans*P_LH+POSE_IN_WORLD;
////    P_LH_w.x()+=foot_odom.pose.pose.position.x;
////    P_LH_w.y()+=foot_odom.pose.pose.position.y;
////    P_LH_w.z()+=foot_odom.pose.pose.position.z;

//    if(pre_P_LF.x()==0.0&&pre_P_LF.y()==0.0&&pre_P_LF.z()==0.0){
//        pre_P_LF=P_LF_r;
//        pre_p_RF=P_RF_r;
//        pre_P_RH=P_RH_r;
//        pre_P_LH=P_LH_r;
//    }
//    if(robot_state_handle.foot_contact_[0]==1){
//        //cout<<"first contact!!!"<<endl;
//        footpos_delta.data[0]=P_LF_r[0]-pre_P_LF[0];
//        footpos_delta.data[1]=P_LF_r[1]-pre_P_LF[1];
//        footpos_delta.data[2]=P_LF_r[2]-pre_P_LF[2];
//    }
//    if(robot_state_handle.foot_contact_[1]==1){
//        //cout<<"second contact!!!"<<endl;
//        footpos_delta.data[3]=P_RF_r[0]-pre_p_RF[0];
//        footpos_delta.data[4]=P_RF_r[1]-pre_p_RF[1];
//        footpos_delta.data[5]=P_RF_r[2]-pre_p_RF[2];
//    }
//    if(robot_state_handle.foot_contact_[2]==1){
//        //cout<<"third contact!!!"<<endl;
//        footpos_delta.data[6]=P_RH_r[0]-pre_P_RH[0];
//        footpos_delta.data[7]=P_RH_r[1]-pre_P_RH[1];
//        footpos_delta.data[8]=P_RH_r[2]-pre_P_RH[2];
//    }
//    if(robot_state_handle.foot_contact_[3]==1){
//        //cout<<"fourth contact!!!"<<endl;
//        footpos_delta.data[9]=P_LH_r[0]-pre_P_LH[0];
//        footpos_delta.data[10]=P_LH_r[1]-pre_P_LH[1];
//        footpos_delta.data[11]=P_LH_r[2]-pre_P_LH[2];
//    }
//    LF_foot_Pos_delta<<footpos_delta.data[0],footpos_delta.data[1],footpos_delta.data[2];
//    RF_foot_Pos_delta<<footpos_delta.data[3],footpos_delta.data[4],footpos_delta.data[5];
//    RH_foot_Pos_delta<<footpos_delta.data[6],footpos_delta.data[7],footpos_delta.data[8];
//    LH_foot_Pos_delta<<footpos_delta.data[9],footpos_delta.data[10],footpos_delta.data[11];
//    N_contact=robot_state_handle.foot_contact_[0]+robot_state_handle.foot_contact_[1]+
//            robot_state_handle.foot_contact_[2]+robot_state_handle.foot_contact_[3];
//    double weight;
//    if(N_contact==4.0){
//        weight = 0.25;
//    }else if(N_contact==3.0){
//        weight=0.33;
//    }else if(N_contact==2.0){
//        weight=0.5;
//    }else if(N_contact==1.0){
//        weight=1.0;
//    }else{
//        weight=0.0;
//    }
////    std::cout<<robot_state_handle.foot_contact_[0]<<" "<<robot_state_handle.foot_contact_[1]<<" "
////                                         <<robot_state_handle.foot_contact_[2]<<" "<<robot_state_handle.foot_contact_[3]<<std::endl;
//    x_delta = ((LF_foot_Pos_delta.x()*robot_state_handle.foot_contact_[0]+
//                       RF_foot_Pos_delta.x()*robot_state_handle.foot_contact_[1]+RH_foot_Pos_delta.x()*robot_state_handle.foot_contact_[2]+LH_foot_Pos_delta.x()*robot_state_handle.foot_contact_[3])*weight);
//    y_delta = (weight)*(LF_foot_Pos_delta.y()*robot_state_handle.foot_contact_[0]+
//            RF_foot_Pos_delta.y()*robot_state_handle.foot_contact_[1]+RH_foot_Pos_delta.y()*robot_state_handle.foot_contact_[2]+LH_foot_Pos_delta.y()*robot_state_handle.foot_contact_[3]);
//    z_delta = (0.25)*(P_LH_r[2]+
//            P_LF_r[2]+P_RF_r[2]+P_RH_r[2]);

//    if((robot_state_handle.foot_contact_[0]==0.0)||(robot_state_handle.foot_contact_[1]==0.0)  //当足端不接触时,默认z方向位置不变
//            ||(robot_state_handle.foot_contact_[2]==0.0)||(robot_state_handle.foot_contact_[3]==0.0)){
//        z_delta = pre_pos;
//    }
//    pre_pos = z_delta;
//    foot_odom.pose.pose.position.x +=(-x_delta);
//    foot_odom.pose.pose.position.y +=(-y_delta);
//    foot_odom.pose.pose.position.z =(-z_delta+0.01);  //0.02 for foot radius???
//    //cout<<
//    foot_odom.pose.covariance[0]=0.005;
//    foot_odom.pose.covariance[7]=0.005;
//    foot_odom.pose.covariance[14]=0.005;

//    pre_P_LF=P_LF_r;
//    pre_p_RF=P_RF_r;
//    pre_P_RH=P_RH_r;
//    pre_P_LH=P_LH_r;
//    base_in_world=robot_state->getPositionWorldToBaseInWorldFrame();
//    ;
//    cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
//    odom_ptr_->GetLinearVelFromJointvel();
//    vel_test.twist.linear.x=odom_ptr_->odom_vel.x();
//    vel_test.twist.linear.y=odom_ptr_->odom_vel.y();
//    vel_test.twist.linear.z=odom_ptr_->odom_vel.z();
//    velPub_.publish(vel_test);
//    //cout<<odom_ptr_->V_Jacob[0]<<endl;
//    //cout<<odom_ptr_->odom_vel<<endl;
//   // cout<<odom_ptr_->odom_vel_inodom<<endl;
////    cout<<robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LF_LEG)<<endl;
////    cout<<robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RF_LEG)<<endl;
//    //cout<<"lf contact position:  "<<P_LF_w<<endl;
//    //cout<<"rf contact position:  "<<P_RF_w<<endl;
//    //cout<<"lh contact position:  "<<P_LH_w<<endl;
//    //cout<<"rh contact position:  "<<P_RH_w<<endl;
//    //cout<<"rf contact position:  "<<P_RF_w<<endl;
//    //cout<<"lf contact position:  "<<P_LF<<endl;
////    //cout<<"rf contact position:  "<<P_RF<<endl;
////    cout<<"X  "<<foot_odom.pose.pose.position.x<<endl;
////    cout<<"Y  "<<foot_odom.pose.pose.position.y<<endl;
////    cout<<"Z  "<<foot_odom.pose.pose.position.z<<endl;
////    cout<<"base pose:  "<<base_in_world<<endl;
////    cout<<"base orientation   "<<base_orinetation<<endl;
//    cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
    //! WSHY: data logging
    geometry_msgs::WrenchStamped vmc_force_torque, desired_vmc_ft;
    geometry_msgs::Pose pos_error;
    geometry_msgs::Twist twist_error;
    geometry_msgs::Pose desired_pose, actual_pose;
    geometry_msgs::Twist desired_twist, actual_twist;
    nav_msgs::Odometry desire_odom, actual_odom;
    free_gait_msgs::RobotState desired_robot_state, actual_robot_state;
    sim_assiants::FootContacts desired_contact;
    if(log_data||my_log)//base_actual_pose_.size()<log_length_)
      {
        motor_status_word_.push_back(status_word);
//        std_msgs::Time current_time;
//        current_time.data.setNow(ros::Time::now());
//        log_time_.push_back(current_time);
        vmc_force_torque.header.frame_id = "/base";
        vmc_force_torque.header.stamp = ros::Time::now();
        desired_vmc_ft.header.frame_id = "/base";
        desired_vmc_ft.header.stamp = ros::Time::now();
        Force net_force;
        Torque net_torque;
        virtual_model_controller_->getDistributedVirtualForceAndTorqueInBaseFrame(net_force, net_torque);
        kindr_ros::convertToRosGeometryMsg(Position(virtual_model_controller_->getDesiredVirtualForceInBaseFrame().vector()),
                                           desired_vmc_ft.wrench.force);
        kindr_ros::convertToRosGeometryMsg(Position(virtual_model_controller_->getDesiredVirtualTorqueInBaseFrame().vector()),
                                           desired_vmc_ft.wrench.torque);
        kindr_ros::convertToRosGeometryMsg(Position(net_force.vector()),
                                           vmc_force_torque.wrench.force);
        kindr_ros::convertToRosGeometryMsg(Position(net_torque.vector()),
                                           vmc_force_torque.wrench.torque);
//        kindr_ros::convertToRosGeometryMsg(Position(virtual_model_controller_->getPosError()),
//                                           pos_error);
//        kindr_ros::convertToRosGeometryMsg(LinearVelocity(virtual_model_controller_->getLinearVelError()),
//                                           pos_error);
        pos_error.position.x = virtual_model_controller_->getPosError().x();
        pos_error.position.y = virtual_model_controller_->getPosError().y();
        pos_error.position.z = virtual_model_controller_->getPosError().z();
       // pos_error.orientation.w = virtual_model_controller_->getOriError().w();
        pos_error.orientation.x = virtual_model_controller_->getOriError().x();    //MXR::NOTE: for euler angle ???
        pos_error.orientation.y = virtual_model_controller_->getOriError().y();
        pos_error.orientation.z = virtual_model_controller_->getOriError().z();
        twist_error.linear.x = virtual_model_controller_->getLinearVelError().x();
        twist_error.linear.y = virtual_model_controller_->getLinearVelError().y();
        twist_error.linear.z = virtual_model_controller_->getLinearVelError().z();
        twist_error.angular.x = virtual_model_controller_->getAngularVelError().x();
        twist_error.angular.y = virtual_model_controller_->getAngularVelError().y();
        twist_error.angular.z = virtual_model_controller_->getAngularVelError().z();

//        std::cout<<"twist_error.linear.x   "<<twist_error.linear.x<<std::endl;
//        std::cout<<"twist_error.linear.y   "<<twist_error.linear.y<<std::endl;
//        std::cout<<"twist_error.linear.z   "<<twist_error.linear.z<<std::endl;
//        std::cout<<"twist_error.angular.x   "<<twist_error.angular.x<<std::endl;
//        std::cout<<"twist_error.angular.y   "<<twist_error.angular.y<<std::endl;
//        std::cout<<"twist_error.angular.z   "<<twist_error.angular.z<<std::endl;

        vitual_force_torque_.push_back(vmc_force_torque);
        desired_vitual_force_torque_.push_back(desired_vmc_ft);


        desired_robot_state.lf_target.target_position.resize(1);
        desired_robot_state.lf_target.target_velocity.resize(1);
        desired_robot_state.lf_target.target_force.resize(1);
//        desired_robot_state.lf_leg_joints.position.resize(3);
//        desired_robot_state.lf_leg_joints.effort.resize(3);
//        desired_robot_state.lf_leg_joints.velocity.resize(3);
//        desired_robot_state.lf_leg_joints.name.resize(3);

        actual_robot_state.lf_target.target_position.resize(1);
        actual_robot_state.lf_target.target_velocity.resize(1);
        actual_robot_state.lf_target.target_force.resize(1);
//        actual_robot_state.lf_leg_joints.position.resize(3);
//        actual_robot_state.lf_leg_joints.effort.resize(3);
//        actual_robot_state.lf_leg_joints.velocity.resize(3);
//        actual_robot_state.lf_leg_joints.name.resize(3);

        desired_robot_state.rf_target.target_position.resize(1);
        desired_robot_state.rf_target.target_velocity.resize(1);
        desired_robot_state.rf_target.target_force.resize(1);
//        desired_robot_state.rf_leg_joints.position.resize(3);
//        desired_robot_state.rf_leg_joints.effort.resize(3);
//        desired_robot_state.rf_leg_joints.velocity.resize(3);
//        desired_robot_state.rf_leg_joints.name.resize(3);
        actual_robot_state.rf_target.target_position.resize(1);
        actual_robot_state.rf_target.target_velocity.resize(1);
        actual_robot_state.rf_target.target_force.resize(1);
//        actual_robot_state.rf_leg_joints.position.resize(3);
//        actual_robot_state.rf_leg_joints.effort.resize(3);
//        actual_robot_state.rf_leg_joints.velocity.resize(3);
//        actual_robot_state.rf_leg_joints.name.resize(3);

        desired_robot_state.rh_target.target_position.resize(1);
        desired_robot_state.rh_target.target_velocity.resize(1);
        desired_robot_state.rh_target.target_force.resize(1);
//        desired_robot_state.rh_leg_joints.position.resize(3);
//        desired_robot_state.rh_leg_joints.effort.resize(3);
//        desired_robot_state.rh_leg_joints.velocity.resize(3);
//        desired_robot_state.rh_leg_joints.name.resize(3);
        actual_robot_state.rh_target.target_position.resize(1);
        actual_robot_state.rh_target.target_velocity.resize(1);
        actual_robot_state.rh_target.target_force.resize(1);
//        actual_robot_state.rh_leg_joints.position.resize(3);
//        actual_robot_state.rh_leg_joints.effort.resize(3);
//        actual_robot_state.rh_leg_joints.velocity.resize(3);
//        actual_robot_state.rh_leg_joints.name.resize(3);

        desired_robot_state.lh_target.target_position.resize(1);
        desired_robot_state.lh_target.target_velocity.resize(1);
        desired_robot_state.lh_target.target_force.resize(1);
//        desired_robot_state.lh_leg_joints.position.resize(3);
//        desired_robot_state.lh_leg_joints.effort.resize(3);
//        desired_robot_state.lh_leg_joints.velocity.resize(3);
//        desired_robot_state.lh_leg_joints.name.resize(3);
        actual_robot_state.lh_target.target_position.resize(1);
        actual_robot_state.lh_target.target_velocity.resize(1);
        actual_robot_state.lh_target.target_force.resize(1);
//        actual_robot_state.lh_leg_joints.position.resize(3);
//        actual_robot_state.lh_leg_joints.effort.resize(3);
//        actual_robot_state.lh_leg_joints.velocity.resize(3);
//        actual_robot_state.lh_leg_joints.name.resize(3);



        //! WSHY: Rotate Position and Velocity to Base Frame
        Pose current_pose_in_base;//
        Position base_desired_position_in_base = current_base_pose.getRotation().inverseRotate(base_desired_position);
        current_pose_in_base.getPosition() = current_base_pose.getRotation().inverseRotate(current_base_pose.getPosition());
        current_pose_in_base.getRotation() = current_base_pose.getRotation();
        LinearVelocity desired_vel_in_base, current_vel_in_base;
        LinearVelocity current_vel_in_world(robot_state_handle.getLinearVelocity()[0],
                                           robot_state_handle.getLinearVelocity()[1],
                                           robot_state_handle.getLinearVelocity()[2]);
        desired_vel_in_base = current_base_pose.getRotation().inverseRotate(base_desired_linear_velocity);
        current_vel_in_base = current_base_pose.getRotation().inverseRotate(current_vel_in_world);


        kindr_ros::convertToRosGeometryMsg(base_desired_position_in_base, desired_pose.position);
        kindr_ros::convertToRosGeometryMsg(base_desired_rotation, desired_pose.orientation);
        kindr_ros::convertToRosGeometryMsg(desired_vel_in_base, desired_twist.linear);
        kindr_ros::convertToRosGeometryMsg(base_desired_angular_velocity, desired_twist.angular);
        kindr_ros::convertToRosGeometryMsg(current_vel_in_base, actual_twist.linear);
        kindr_ros::convertToRosGeometryMsg(current_pose_in_base, actual_pose);
        //! WSHY: for pose & twist in World Frame (uncomment the followinf 8 lines)
//        kindr_ros::convertToRosGeometryMsg(base_desired_position, desired_pose.position);
//        kindr_ros::convertToRosGeometryMsg(base_desired_rotation, desired_pose.orientation);
//        kindr_ros::convertToRosGeometryMsg(base_desired_linear_velocity, desired_twist.linear);
//        kindr_ros::convertToRosGeometryMsg(base_desired_angular_velocity, desired_twist.angular);
//        kindr_ros::convertToRosGeometryMsg(current_pose, actual_pose);

//        actual_twist.linear.x = robot_state_handle.getLinearVelocity()[0];
//        actual_twist.linear.y = robot_state_handle.getLinearVelocity()[1];
//        actual_twist.linear.z = robot_state_handle.getLinearVelocity()[2];

        actual_twist.angular.x = robot_state_handle.getAngularVelocity()[0];
        actual_twist.angular.y = robot_state_handle.getAngularVelocity()[1];
        actual_twist.angular.z = robot_state_handle.getAngularVelocity()[2];
        desire_odom.pose.pose = desired_pose;
        desire_odom.twist.twist = desired_twist;
        actual_odom.pose.pose = actual_pose;
        actual_odom.twist.twist = actual_twist;

        z_height=actual_pose.position.z;

        base_actual_pose_.push_back(actual_odom);
        base_command_pose_.push_back(desire_odom);
        leg_states_.push_back(leg_state);
        joint_command_.push_back(joint_command);
        joint_actual_.push_back(joint_actual);
        pos_error_.push_back(pos_error);
        vel_error_.push_back(twist_error);


        desired_contact.foot_contacts.resize(4);
        std::vector<sensor_msgs::JointState> joint_states_leg, joint_commands_leg;
        joint_states_leg.resize(4);
        joint_commands_leg.resize(4);
        std::vector<free_gait::Force> real_contact_forces;
        real_contact_forces.resize(4);

        for(int i = 0; i<4; i++)
          {
            free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
            Force contact_force_in_base = contact_distribution_->getLegInfo(limb).desiredContactForce_;
            Force contact_force_in_world = robot_state->getOrientationBaseToWorld().rotate(contact_force_in_base);
            desired_contact.foot_contacts[i].contact_force.wrench.force.x = contact_force_in_base(0);
            desired_contact.foot_contacts[i].contact_force.wrench.force.y = contact_force_in_base(1);
            desired_contact.foot_contacts[i].contact_force.wrench.force.z = contact_force_in_base(2);
//            ROS_INFO("Log surface normals");
            desired_contact.foot_contacts[i].surface_normal.vector.x = surface_normals.at(limb)(0);
            desired_contact.foot_contacts[i].surface_normal.vector.y = surface_normals.at(limb)(1);
            desired_contact.foot_contacts[i].surface_normal.vector.z = surface_normals.at(limb)(2);

//            if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::StanceNormal)
//              desired_contact.foot_contacts[i].is_contact = true;
//            if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::SwingNormal)
//              desired_contact.foot_contacts[i].is_contact = false;
            desired_contact.foot_contacts[i].is_contact = robot_state->isSupportLeg(limb);//real_contact_.at(limb);

            leg_phase.data[2*i] = st_phase.at(limb);
            leg_phase.data[2*i + 1] = sw_phase.at(limb);
            Eigen::Vector3d joint_torque_leg;
            for(int j = 0;j<3;j++)
              {
                joint_states_leg[i].effort.push_back(joint_actual.effort[3*i+j]);
                joint_states_leg[i].position.push_back(joint_actual.position[3*i+j]);
                joint_states_leg[i].velocity.push_back(joint_actual.velocity[3*i+j]);
                joint_states_leg[i].name.push_back(joint_names[3*i+j]);
                joint_torque_leg(i) = joint_actual.effort[3*i+j];

                joint_commands_leg[i].effort.push_back(joint_command.effort[3*i+j]);
                joint_commands_leg[i].position.push_back(joint_command.position[3*i+j]);
                joint_commands_leg[i].name.push_back(joint_names[3*i+j]);

              }
//            Eigen::Matrix3d jacobian_t = robot_state_->getTranslationJacobianFromBaseToFootInBaseFrame(limb).transpose();
//            real_contact_forces[i] = Force(jacobian_t.inverse()*joint_torque_leg);


          }
        leg_phases_.push_back(leg_phase);
        foot_desired_contact_.push_back(desired_contact);

        desired_robot_state.base_pose = desire_odom;
        kindr_ros::convertToRosGeometryMsg(Position(foot_positions.at(free_gait::LimbEnum::LF_LEG).vector()),
                                           desired_robot_state.lf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(Position(foot_positions.at(free_gait::LimbEnum::RF_LEG).vector()),
                                           desired_robot_state.rf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(Position(foot_positions.at(free_gait::LimbEnum::RH_LEG).vector()),
                                           desired_robot_state.rh_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(Position(foot_positions.at(free_gait::LimbEnum::LH_LEG).vector()),
                                           desired_robot_state.lh_target.target_position[0].point);

        kindr_ros::convertToRosGeometryMsg(LinearVelocity(foot_velocities.at(free_gait::LimbEnum::LF_LEG).vector()),
                                           desired_robot_state.lf_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(LinearVelocity(foot_velocities.at(free_gait::LimbEnum::RF_LEG).vector()),
                                           desired_robot_state.rf_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(LinearVelocity(foot_velocities.at(free_gait::LimbEnum::RH_LEG).vector()),
                                           desired_robot_state.rh_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(LinearVelocity(foot_velocities.at(free_gait::LimbEnum::LH_LEG).vector()),
                                           desired_robot_state.lh_target.target_velocity[0].vector);

        desired_robot_state.lf_target.target_force[0].vector = desired_contact.foot_contacts[0].contact_force.wrench.force;
        desired_robot_state.rf_target.target_force[0].vector = desired_contact.foot_contacts[1].contact_force.wrench.force;
        desired_robot_state.rh_target.target_force[0].vector = desired_contact.foot_contacts[2].contact_force.wrench.force;
        desired_robot_state.lh_target.target_force[0].vector = desired_contact.foot_contacts[3].contact_force.wrench.force;

        desired_robot_state.lf_leg_joints = joint_commands_leg[0];
        desired_robot_state.rf_leg_joints = joint_commands_leg[1];
        desired_robot_state.rh_leg_joints = joint_commands_leg[2];
        desired_robot_state.lh_leg_joints = joint_commands_leg[3];

        desired_robot_state.lf_leg_mode.support_leg = limbs_desired_state.at(free_gait::LimbEnum::LF_LEG)->getState();
        desired_robot_state.rf_leg_mode.support_leg = limbs_desired_state.at(free_gait::LimbEnum::RF_LEG)->getState();
        desired_robot_state.rh_leg_mode.support_leg = limbs_desired_state.at(free_gait::LimbEnum::RH_LEG)->getState();
        desired_robot_state.lh_leg_mode.support_leg = limbs_desired_state.at(free_gait::LimbEnum::LH_LEG)->getState();

//        desired_robot_state.lf_leg_mode.support_leg = robot_state->isSupportLeg()

        desired_robot_state_.push_back(desired_robot_state);

        actual_robot_state.base_pose = actual_odom;
        kindr_ros::convertToRosGeometryMsg(robot_state->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LF_LEG),
                                           actual_robot_state.lf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(robot_state->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::RF_LEG),
                                           actual_robot_state.rf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(robot_state->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::RH_LEG),
                                           actual_robot_state.rh_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(robot_state->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LH_LEG),
                                           actual_robot_state.lh_target.target_position[0].point);

        kindr_ros::convertToRosGeometryMsg(robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LF_LEG),
                                           actual_robot_state.lf_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RF_LEG),
                                           actual_robot_state.rf_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RH_LEG),
                                           actual_robot_state.rh_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LH_LEG),
                                           actual_robot_state.lh_target.target_velocity[0].vector);

//        kindr_ros::convertToRosGeometryMsg(real_contact_forces[0],
//                                           actual_robot_state.lf_target.target_force[0].vector);
//        kindr_ros::convertToRosGeometryMsg(real_contact_forces[1],
//                                           actual_robot_state.rf_target.target_force[0].vector);
//        kindr_ros::convertToRosGeometryMsg(real_contact_forces[2],
//                                           actual_robot_state.rh_target.target_force[0].vector);
//        kindr_ros::convertToRosGeometryMsg(real_contact_forces[3],
//                                           actual_robot_state.lh_target.target_force[0].vector);

        actual_robot_state.lf_leg_joints = joint_states_leg[0];
        actual_robot_state.rf_leg_joints = joint_states_leg[1];
        actual_robot_state.rh_leg_joints = joint_states_leg[2];
        actual_robot_state.lh_leg_joints = joint_states_leg[3];

        actual_robot_state.lf_leg_mode.support_leg = real_contact_.at(free_gait::LimbEnum::LF_LEG);
        actual_robot_state.rf_leg_mode.support_leg = real_contact_.at(free_gait::LimbEnum::RF_LEG);
        actual_robot_state.rh_leg_mode.support_leg = real_contact_.at(free_gait::LimbEnum::RH_LEG);
        actual_robot_state.lh_leg_mode.support_leg = real_contact_.at(free_gait::LimbEnum::LH_LEG);
//        std::cout<<"real_contact_.at(free_gait::LimbEnum::LF_LEG)     "<<real_contact_.at(free_gait::LimbEnum::LF_LEG)<<std::endl;
//        std::cout<<"real_contact_.at(free_gait::LimbEnum::RF_LEG)     "<<real_contact_.at(free_gait::LimbEnum::RF_LEG)<<std::endl;
//        std::cout<<"real_contact_.at(free_gait::LimbEnum::RH_LEG)     "<<real_contact_.at(free_gait::LimbEnum::RH_LEG)<<std::endl;
//        std::cout<<"real_contact_.at(free_gait::LimbEnum::LH_LEG)     "<<real_contact_.at(free_gait::LimbEnum::LH_LEG)<<std::endl;

        actual_robot_state.lf_leg_mode.surface_normal.vector.z = real_contact_force_.at(free_gait::LimbEnum::LF_LEG).z();
        actual_robot_state.rf_leg_mode.surface_normal.vector.z = real_contact_force_.at(free_gait::LimbEnum::RF_LEG).z();
        actual_robot_state.rh_leg_mode.surface_normal.vector.z = real_contact_force_.at(free_gait::LimbEnum::RH_LEG).z();
        actual_robot_state.lh_leg_mode.surface_normal.vector.z = real_contact_force_.at(free_gait::LimbEnum::LH_LEG).z();

        actual_robot_state_.push_back(actual_robot_state);
      }

    if(my_log){
        base_command_pub_.publish(desire_odom);
        base_actual_pub_.publish(actual_odom);
        leg_state_pub_.publish(leg_state);
        joint_command_pub_.publish(joint_command);
        joint_actual_pub_.publish(joint_actual);
        contact_desired_pub_.publish(desired_contact);
        leg_phase_pub_.publish(leg_phase);
        desired_robot_state_pub_.publish(desired_robot_state);
        actual_robot_state_pub_.publish(actual_robot_state);
        vmc_info_pub_.publish(vmc_force_torque);
        desired_vmc_info_pub_.publish(desired_vmc_ft);
        motor_status_word_pub_.publish(status_word);
        pos_error_pub.publish(pos_error);
        vel_error_pub.publish(twist_error);
    }

  }

  double RosBalanceController::computeTorqueFromPositionCommand(double command, int i, const ros::Duration& period)
  {
//    ros::Duration real_time_period = ros::Duration(period.toSec()/0.55);
    double command_position = command;

    double error; //, vel_error;
    double commanded_effort;

    double current_position = joints[i].getPosition();

    // Make sure joint is within limits if applicable
    enforceJointLimits(command_position, i);

    // Compute position error
    if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
    {
     angles::shortest_angular_distance_with_limits(
        current_position,
        command_position,
        joint_urdfs_[i]->limits->lower,
        joint_urdfs_[i]->limits->upper,
        error);
    }
    else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
    {
      error = angles::shortest_angular_distance(current_position, command_position);
    }
    else //prismatic
    {
      error = command_position - current_position;
    }

    // Set the PID error and compute the PID command with nonuniform
    // time step size.
    commanded_effort = pid_controllers_[i].computeCommand(error, period);
    return commanded_effort;
  }
  /**
   * @brief RosBalanceController::baseCommandCallback, convert desired state, compute joint efforts
   * @param robot_state
   */
  void RosBalanceController::baseCommandCallback(const free_gait_msgs::RobotStateConstPtr& robot_state_msg)
  {
    base_desired_position = Position(robot_state_msg->base_pose.pose.pose.position.x,
                                      robot_state_msg->base_pose.pose.pose.position.y,
                                      robot_state_msg->base_pose.pose.pose.position.z);
    base_desired_rotation = RotationQuaternion(robot_state_msg->base_pose.pose.pose.orientation.w,
                                                          robot_state_msg->base_pose.pose.pose.orientation.x,
                                                          robot_state_msg->base_pose.pose.pose.orientation.y,
                                                          robot_state_msg->base_pose.pose.pose.orientation.z);
    base_desired_linear_velocity = LinearVelocity(robot_state_msg->base_pose.twist.twist.linear.x,
                                                                 robot_state_msg->base_pose.twist.twist.linear.y,
                                                                 robot_state_msg->base_pose.twist.twist.linear.z);
    base_desired_angular_velocity = LocalAngularVelocity(robot_state_msg->base_pose.twist.twist.angular.x,
                                                                              robot_state_msg->base_pose.twist.twist.angular.y,
                                                                              robot_state_msg->base_pose.twist.twist.angular.z);

//    Pose current_base_pose = Pose(Position(robot_state_handle.getPosition()[0],
//                                  robot_state_handle.getPosition()[1],
//                                  robot_state_handle.getPosition()[2]),
//                         RotationQuaternion(robot_state_handle.getOrientation()[0],
//                                            robot_state_handle.getOrientation()[1],
//                                            robot_state_handle.getOrientation()[2],
//                                            robot_state_handle.getOrientation()[3]));
//    ROS_INFO("Desired base pose: Position: ");
//    std::cout<<base_desired_position<<std::endl;
//    kindr::EulerAnglesZyxPD rotation_desired(base_desired_rotation);
//    ROS_INFO("Desired base pose: Rotation: ");
//    std::cout<<rotation_desired<<std::endl;
//    ROS_INFO("Current base pose: Position: ");
//    std::cout<<current_base_pose.getPosition()<<std::endl;
//    kindr::EulerAnglesZyxPD rotation_current(current_base_pose.getRotation());
//    ROS_INFO("Current base pose: Rotation: ");
//    std::cout<<rotation_current<<std::endl;
//    ROS_INFO("Orienitaion Error is : ");
//    std::cout<<base_desired_rotation.boxMinus(current_base_pose.getRotation())<<std::endl;

//    free_gait::JointPositions all_joint_positions;
    std::vector<double> joint_commands;
    joint_commands.resize(12);
    for(unsigned int i = 0;i<3;i++)
    {
        /****************
        * TODO(Shunyao) : only for the non-support leg to follow the joint position and
        *velocity command
        ****************/
      joint_commands[i] = robot_state_msg->lf_leg_joints.position[i];
      joint_commands[i+3] = robot_state_msg->rf_leg_joints.position[i];
      joint_commands[i+6] = robot_state_msg->rh_leg_joints.position[i];
      joint_commands[i+9] = robot_state_msg->lh_leg_joints.position[i];
    }
    commands_buffer.writeFromNonRT(joint_commands);

//    LimbVector foot_positions, foot_velocities, foot_accelerations;
    //! WSHY: update cartesian foot motion command
    Position foot_position, foot_velocity, foot_acceleration;

    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lf_target.target_position[0].point,
        foot_position);
    foot_positions[free_gait::LimbEnum::LF_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lf_target.target_velocity[0].vector,
        foot_velocity);
    foot_velocities[free_gait::LimbEnum::LF_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lf_target.target_acceleration[0].vector,
        foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::LF_LEG] = Vector(foot_acceleration.toImplementation());

        //std::cout<<"foot_accelerations[free_gait::LimbEnum::LF_LEG] "<<Vector(foot_acceleration.toImplementation())<<std::endl;

    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rf_target.target_position[0].point,
        foot_position);
    foot_positions[free_gait::LimbEnum::RF_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rf_target.target_velocity[0].vector,
        foot_velocity);
    foot_velocities[free_gait::LimbEnum::RF_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rf_target.target_acceleration[0].vector,
        foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::RF_LEG] = Vector(foot_acceleration.toImplementation());

        //std::cout<<"foot_accelerations[free_gait::LimbEnum::RF_LEG] "<<Vector(foot_acceleration.toImplementation())<<std::endl;

    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rh_target.target_position[0].point,
        foot_position);
    foot_positions[free_gait::LimbEnum::RH_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rh_target.target_velocity[0].vector,
        foot_velocity);
    foot_velocities[free_gait::LimbEnum::RH_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rh_target.target_acceleration[0].vector,
        foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::RH_LEG] = Vector(foot_acceleration.toImplementation());
        //std::cout<<"foot_accelerations[free_gait::LimbEnum::RH_LEG] "<<Vector(foot_acceleration.toImplementation())<<std::endl;

    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lh_target.target_position[0].point,
        foot_position);
    foot_positions[free_gait::LimbEnum::LH_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lh_target.target_velocity[0].vector,
        foot_velocity);
    foot_velocities[free_gait::LimbEnum::LH_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lh_target.target_acceleration[0].vector,
        foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::LH_LEG] = Vector(foot_acceleration.toImplementation());
        //std::cout<<"foot_accelerations[free_gait::LimbEnum::LH_LEG] "<<Vector(foot_acceleration.toImplementation())<<std::endl;




//    command_foot_buffer.writeFromNonRT(foot_positions);
//    command_foot_vel_buffer.writeFromNonRT(foot_velocities);

    robot_state_->setPositionWorldToBaseInWorldFrame(base_desired_position);
    robot_state_->setOrientationBaseToWorld(RotationQuaternion(base_desired_rotation));
    robot_state_->setLinearVelocityBaseInWorldFrame(base_desired_linear_velocity);
    robot_state_->setAngularVelocityBaseInBaseFrame(base_desired_angular_velocity);

    if(robot_state_msg->lf_leg_mode.name == "joint")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = false;
      }
    if(robot_state_msg->lf_leg_mode.name == "leg_mode")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = true;
      }
    if(robot_state_msg->lf_leg_mode.name == "cartesian")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = false;
      }
    if(robot_state_msg->lf_leg_mode.name == "footstep")
      {
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = true;
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = false;
      }

    if(robot_state_msg->rf_leg_mode.name == "joint")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = false;
      }
    if(robot_state_msg->rf_leg_mode.name == "leg_mode")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = true;
      }
    if(robot_state_msg->rf_leg_mode.name == "cartesian")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = false;
      }
    if(robot_state_msg->rf_leg_mode.name == "footstep")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = true;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = false;
      }

    if(robot_state_msg->rh_leg_mode.name == "joint")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = false;
      }
    if(robot_state_msg->rh_leg_mode.name == "leg_mode")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = true;
      }
    if(robot_state_msg->rh_leg_mode.name == "cartesian")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = false;
      }
    if(robot_state_msg->rh_leg_mode.name == "footstep")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = true;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = false;
      }

    if(robot_state_msg->lh_leg_mode.name == "joint")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = false;
      }
    if(robot_state_msg->lh_leg_mode.name == "leg_mode")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = true;
      }
    if(robot_state_msg->lh_leg_mode.name == "cartesian")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = false;
      }
    if(robot_state_msg->lh_leg_mode.name == "footstep")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = true;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = false;
      }

    //MXR::NOTE: 根据gait_generate_client设置的support_Leg来确定理想的足端接触状态
    if(robot_state_msg->lf_leg_mode.support_leg){
        robot_state_->setSupportLeg(free_gait::LimbEnum::LF_LEG, true);
        robot_state_->setSurfaceNormal(free_gait::LimbEnum::LF_LEG,
                                       Vector(robot_state_msg->lf_leg_mode.surface_normal.vector.x,
                                              robot_state_msg->lf_leg_mode.surface_normal.vector.y,
                                              robot_state_msg->lf_leg_mode.surface_normal.vector.z));

        limbs_desired_state.at(free_gait::LimbEnum::LF_LEG)->setState(StateSwitcher::States::StanceNormal);
        if(st_flag.at(free_gait::LimbEnum::LF_LEG)){
            st_flag.at(free_gait::LimbEnum::LF_LEG) = false;
            t_st0.at(free_gait::LimbEnum::LF_LEG)= ros::Time::now();
          }
        sw_flag.at(free_gait::LimbEnum::LF_LEG) = false;
        st_phase.at(free_gait::LimbEnum::LF_LEG) = robot_state_msg->lf_leg_mode.phase;
        sw_phase.at(free_gait::LimbEnum::LF_LEG) = 0;
      } else {
//        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

        robot_state_->setSupportLeg(free_gait::LimbEnum::LF_LEG, false);
        robot_state_->setSurfaceNormal(free_gait::LimbEnum::LF_LEG,Vector(0,0,1));
        limbs_desired_state.at(free_gait::LimbEnum::LF_LEG)->setState(StateSwitcher::States::SwingNormal);
        if(!sw_flag.at(free_gait::LimbEnum::LF_LEG)){
          t_sw0.at(free_gait::LimbEnum::LF_LEG) = ros::Time::now();
          sw_flag.at(free_gait::LimbEnum::LF_LEG) = true;
          }
        st_flag.at(free_gait::LimbEnum::LF_LEG) = true;
        sw_phase.at(free_gait::LimbEnum::LF_LEG) = robot_state_msg->lf_leg_mode.phase;
        st_phase.at(free_gait::LimbEnum::LF_LEG) = 0;

      };
    if(robot_state_msg->rf_leg_mode.support_leg){
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(1), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(1),
                                       Vector(robot_state_msg->rf_leg_mode.surface_normal.vector.x,
                                              robot_state_msg->rf_leg_mode.surface_normal.vector.y,
                                              robot_state_msg->rf_leg_mode.surface_normal.vector.z));
        limbs_desired_state.at(free_gait::LimbEnum::RF_LEG)->setState(StateSwitcher::States::StanceNormal);
        if(st_flag.at(free_gait::LimbEnum::RF_LEG)){
            st_flag.at(free_gait::LimbEnum::RF_LEG) = false;
            t_st0.at(free_gait::LimbEnum::RF_LEG)= ros::Time::now();
          }
        sw_flag.at(free_gait::LimbEnum::RF_LEG) = false;
        st_phase.at(free_gait::LimbEnum::RF_LEG) = robot_state_msg->rf_leg_mode.phase;
        sw_phase.at(free_gait::LimbEnum::RF_LEG) = 0;
      } else {
//        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(1), false);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(1),Vector(0,0,1));
        limbs_desired_state.at(free_gait::LimbEnum::RF_LEG)->setState(StateSwitcher::States::SwingNormal);
        if(!sw_flag.at(free_gait::LimbEnum::RF_LEG)){
          t_sw0.at(free_gait::LimbEnum::RF_LEG) = ros::Time::now();
          sw_flag.at(free_gait::LimbEnum::RF_LEG) = true;
          }
        st_flag.at(free_gait::LimbEnum::RF_LEG) = true;
        sw_phase.at(free_gait::LimbEnum::RF_LEG) = robot_state_msg->rf_leg_mode.phase;
        st_phase.at(free_gait::LimbEnum::RF_LEG) = 0;

      };
    if(robot_state_msg->rh_leg_mode.support_leg){
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(2), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(2),
                                       Vector(robot_state_msg->rh_leg_mode.surface_normal.vector.x,
                                              robot_state_msg->rh_leg_mode.surface_normal.vector.y,
                                              robot_state_msg->rh_leg_mode.surface_normal.vector.z));
        limbs_desired_state.at(free_gait::LimbEnum::RH_LEG)->setState(StateSwitcher::States::StanceNormal);
        if(st_flag.at(free_gait::LimbEnum::RH_LEG)){
            st_flag.at(free_gait::LimbEnum::RH_LEG) = false;
            t_st0.at(free_gait::LimbEnum::RH_LEG)= ros::Time::now();
          }
        sw_flag.at(free_gait::LimbEnum::RH_LEG) = false;
        st_phase.at(free_gait::LimbEnum::RH_LEG) = robot_state_msg->rh_leg_mode.phase;
        sw_phase.at(free_gait::LimbEnum::RH_LEG) = 0;
      } else {
//        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(2), false);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(2),Vector(0,0,1));
        limbs_desired_state.at(free_gait::LimbEnum::RH_LEG)->setState(StateSwitcher::States::SwingNormal);
        if(!sw_flag.at(free_gait::LimbEnum::RH_LEG)){
          t_sw0.at(free_gait::LimbEnum::RH_LEG) = ros::Time::now();
          sw_flag.at(free_gait::LimbEnum::RH_LEG) = true;
          }
        st_flag.at(free_gait::LimbEnum::RH_LEG) = true;
        sw_phase.at(free_gait::LimbEnum::RH_LEG) = robot_state_msg->rh_leg_mode.phase;
        st_phase.at(free_gait::LimbEnum::RH_LEG) = 0;

      };
    if(robot_state_msg->lh_leg_mode.support_leg){
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(3), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(3),
                                       Vector(robot_state_msg->lh_leg_mode.surface_normal.vector.x,
                                              robot_state_msg->lh_leg_mode.surface_normal.vector.y,
                                              robot_state_msg->lh_leg_mode.surface_normal.vector.z));
        limbs_desired_state.at(free_gait::LimbEnum::LH_LEG)->setState(StateSwitcher::States::StanceNormal);
        if(st_flag.at(free_gait::LimbEnum::LH_LEG)){
            st_flag.at(free_gait::LimbEnum::LH_LEG) = false;
            t_st0.at(free_gait::LimbEnum::LH_LEG)= ros::Time::now();
          }
        sw_flag.at(free_gait::LimbEnum::LH_LEG) = false;
        st_phase.at(free_gait::LimbEnum::LH_LEG) = robot_state_msg->lh_leg_mode.phase;
        sw_phase.at(free_gait::LimbEnum::LH_LEG) = 0;
      } else {
//        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(3), false);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(3),Vector(0,0,1));
        limbs_desired_state.at(free_gait::LimbEnum::LH_LEG)->setState(StateSwitcher::States::SwingNormal);
        if(!sw_flag.at(free_gait::LimbEnum::LH_LEG)){
          t_sw0.at(free_gait::LimbEnum::LH_LEG) = ros::Time::now();
          sw_flag.at(free_gait::LimbEnum::LH_LEG) = true;
          }
        st_flag.at(free_gait::LimbEnum::LH_LEG) = true;
        sw_phase.at(free_gait::LimbEnum::LH_LEG) = robot_state_msg->lh_leg_mode.phase;
        st_phase.at(free_gait::LimbEnum::LH_LEG) = 0;

      };

//  std::cout<<*robot_state_<<std::endl;

  }

  void RosBalanceController::lf_forceCB(const geometry_msgs::WrenchStamped::ConstPtr &msg){
        lf_contact_force.wrench.force = msg->wrench.force;
  }
  void RosBalanceController::rf_forceCB(const geometry_msgs::WrenchStamped::ConstPtr &msg){
        rf_contact_force.wrench.force = msg->wrench.force;
  }
  void RosBalanceController::rh_forceCB(const geometry_msgs::WrenchStamped::ConstPtr &msg){
        rh_contact_force.wrench.force = msg->wrench.force;
  }
  void RosBalanceController::lh_forceCB(const geometry_msgs::WrenchStamped::ConstPtr &msg){
        lh_contact_force.wrench.force = msg->wrench.force;
  }
  void RosBalanceController::contactProCB(const std_msgs::Float64MultiArray::ConstPtr &msg){
//      foot_contact.data[0]=msg->data[0];//lf
//      foot_contact.data[1]=msg->data[1];//rf
//      foot_contact.data[2]=msg->data[2];//rh
//      foot_contact.data[3]=msg->data[3];//lh
//      if(!real_robot){
//          real_contact_.at(free_gait::LimbEnum::LF_LEG) =  foot_contact.data[0]>0.8?1:0;
//          real_contact_.at(free_gait::LimbEnum::RF_LEG) =  foot_contact.data[1]>0.8?1:0;
//          real_contact_.at(free_gait::LimbEnum::LH_LEG) =  foot_contact.data[3]>0.8?1:0;
//          real_contact_.at(free_gait::LimbEnum::RH_LEG) =  foot_contact.data[2]>0.8?1:0;
//      }
  }
  void RosBalanceController::contactStateMachine()
  {

      Eigen::Vector3d v_lf,v_rf,v_rh,v_lh;
      Eigen::Vector3d f_lf,f_rf,f_rh,f_lh;
      f_lf<<lf_contact_force.wrench.force.x,lf_contact_force.wrench.force.y,lf_contact_force.wrench.force.z;
      f_rf<<rf_contact_force.wrench.force.x,rf_contact_force.wrench.force.y,rf_contact_force.wrench.force.z;
      f_rh<<rh_contact_force.wrench.force.x,rh_contact_force.wrench.force.y,rh_contact_force.wrench.force.z;
      f_lh<<lh_contact_force.wrench.force.x,lh_contact_force.wrench.force.y,lh_contact_force.wrench.force.z;
      v_lf<<robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LF_LEG).x(),
              robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LF_LEG).y(),
              robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LF_LEG).z();
      v_rf<<robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RF_LEG).x(),
              robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RF_LEG).y(),
              robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RF_LEG).z();
      v_lh<<robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LH_LEG).x(),
              robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LH_LEG).y(),
              robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LH_LEG).z();
      v_rh<<robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RH_LEG).x(),
              robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RH_LEG).y(),
              robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RH_LEG).z();
      double f_lf_norm = sqrt(f_lf.squaredNorm());
      double f_rf_norm = sqrt(f_rf.squaredNorm());
      double f_rh_norm = sqrt(f_rh.squaredNorm());
      double f_lh_norm = sqrt(f_lh.squaredNorm());
      double v_lf_norm = v_lf.squaredNorm();
      double v_rf_norm = v_rf.squaredNorm();
      double v_rh_norm = v_rh.squaredNorm();
      double v_lh_norm = v_lh.squaredNorm();

      if(robot_state->isSupportLeg(free_gait::LimbEnum::LF_LEG)){
          foot_vel.data[0]=v_lf_norm;
          double sum = (robot_state->isSupportLeg(free_gait::LimbEnum::RF_LEG)*v_rf_norm+
                  robot_state->isSupportLeg(free_gait::LimbEnum::RH_LEG)*v_rh_norm+
                  robot_state->isSupportLeg(free_gait::LimbEnum::LH_LEG)*v_lh_norm)/(robot_state->isSupportLeg(free_gait::LimbEnum::RF_LEG)+
                                                                                     robot_state->isSupportLeg(free_gait::LimbEnum::RH_LEG)+
                                                                                     robot_state->isSupportLeg(free_gait::LimbEnum::LH_LEG));
          foot_vel.data[4]=sum;
          foot_vel.data[12]=f_lf_norm;
          if((v_lf_norm<0.1*sum||v_lf_norm>10*sum)&&v_lf_norm>0.5){
              //ROS_WARN("LF_leg is slipping!!!!!!!!!!!!");
              foot_vel.data[8]=1;
          }else{
              foot_vel.data[8]=0;
          }

      }
      if(robot_state->isSupportLeg(free_gait::LimbEnum::RF_LEG)){
          foot_vel.data[1]=v_rf_norm;
          double sum = (robot_state->isSupportLeg(free_gait::LimbEnum::LF_LEG)*v_lf_norm+
                  robot_state->isSupportLeg(free_gait::LimbEnum::RH_LEG)*v_rh_norm+
                  robot_state->isSupportLeg(free_gait::LimbEnum::LH_LEG)*v_lh_norm)/(robot_state->isSupportLeg(free_gait::LimbEnum::LF_LEG)+
                                                                                     robot_state->isSupportLeg(free_gait::LimbEnum::RH_LEG)+
                                                                                     robot_state->isSupportLeg(free_gait::LimbEnum::LH_LEG));
          foot_vel.data[5]=sum;
          foot_vel.data[13]=f_rf_norm;
          if((v_rf_norm<0.1*sum||v_rf_norm>10*sum)&&v_rf_norm>0.5){
              //ROS_WARN("RF_leg is slipping!!!!!!!!!!!!");
              foot_vel.data[9]=1;
          }else{
              foot_vel.data[9]=0;
          }

      }
      if(robot_state->isSupportLeg(free_gait::LimbEnum::RH_LEG)){
          foot_vel.data[2]=v_rh_norm;
          double sum = (robot_state->isSupportLeg(free_gait::LimbEnum::RF_LEG)*v_rf_norm+
                  robot_state->isSupportLeg(free_gait::LimbEnum::LF_LEG)*v_lf_norm+
                  robot_state->isSupportLeg(free_gait::LimbEnum::LH_LEG)*v_lh_norm)/(robot_state->isSupportLeg(free_gait::LimbEnum::RF_LEG)+
                                                                                     robot_state->isSupportLeg(free_gait::LimbEnum::LF_LEG)+
                                                                                     robot_state->isSupportLeg(free_gait::LimbEnum::LH_LEG));
          foot_vel.data[6]=sum;
          foot_vel.data[14]=f_rh_norm;
          if((v_rh_norm<0.1*sum||v_rh_norm>10*sum)&&v_rh_norm>0.5){
              //ROS_WARN("RH_leg is slipping!!!!!!!!!!!!");
              foot_vel.data[10]=1;
          }else{
              foot_vel.data[10]=0;
          }

      }
      if(robot_state->isSupportLeg(free_gait::LimbEnum::LH_LEG)){
          foot_vel.data[3]=v_lh_norm;
          double sum = (robot_state->isSupportLeg(free_gait::LimbEnum::RF_LEG)*v_rf_norm+
                  robot_state->isSupportLeg(free_gait::LimbEnum::RH_LEG)*v_rh_norm+
                  robot_state->isSupportLeg(free_gait::LimbEnum::LF_LEG)*v_lf_norm)/(robot_state->isSupportLeg(free_gait::LimbEnum::RF_LEG)+
                                                                                     robot_state->isSupportLeg(free_gait::LimbEnum::RH_LEG)+
                                                                                     robot_state->isSupportLeg(free_gait::LimbEnum::LF_LEG));
          foot_vel.data[7]=sum;
          foot_vel.data[15]=f_lh_norm;
          if((v_lh_norm<0.1*sum||v_lh_norm>10*sum)&&v_lh_norm>0.5){
              //ROS_WARN("LH_leg is slipping!!!!!!!!!!!!");
              foot_vel.data[11]=1;
          }else{
              foot_vel.data[11]=0;
          }

      }

      footVelPub_.publish(foot_vel);
//      std::cout<<"==================================="<<std::endl;
//      std::cout<<"v_lf_norm   "<<v_lf_norm<<std::endl;
//      std::cout<<"v_rf_norm   "<<v_rf_norm<<std::endl;
//      std::cout<<"v_rh_norm   "<<v_rh_norm<<std::endl;
//      std::cout<<"v_lh_norm   "<<v_lh_norm<<std::endl;
//      std::cout<<"==================================="<<std::endl;
      //<<robot_state->get
    for(unsigned int i = 0;i<4;i++)
      {
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);

        if(real_robot && !ignore_contact_sensor)
          {
            ROS_ERROR_ONCE("real robot coming there!!!!!!!!!!!!!!!");
            if(z_height<=0.25){
                real_contact_.at(limb)=foot_contact.data[i]>0.5?true:false;
            }else{
                            real_contact_force_.at(limb).z() = robot_state_handle.contact_pressure_[i];
                            //cout<<"robot_state_handle.contact_pressure_[i]   "<<robot_state_handle.contact_pressure_[i]<<endl;
                            if((robot_state_handle.contact_pressure_[i]) > 25.0)  //MXR::NOTE: FOR laikago this parameter should be changed more carefully
                              {
                                //ROS_INFO("contact!!!!!!!!!!!!!!");
                                real_contact_.at(limb) = true;
                              }else {
                                //ROS_INFO("miss contact!!!!!!!!!!!!!!");
                                real_contact_.at(limb) = false;
                                        //;
                              }
            }

//            std::cout<<"================================"<<std::endl;
//            std::cout<<i<<"  "<<real_contact_.at(limb)<<std::endl;
//            std::cout<<"================================"<<std::endl;

//            real_contact_force_.at(limb).z() = robot_state_handle.contact_pressure_[i];
//            if((robot_state_handle.contact_pressure_[i]) > 25.0)  //MXR::NOTE: FOR laikago this parameter should be changed more carefully
//              {
//                //ROS_INFO("contact!!!!!!!!!!!!!!");
//                real_contact_.at(limb) = true;
//              }else {
//                //ROS_INFO("miss contact!!!!!!!!!!!!!!");
//                real_contact_.at(limb) = false;
//                        //;
//              }
          }


//        if(robot_state->isSupportLeg(limb)){

//        }
//        std::cout<<"==================================="<<std::endl;
//        std::cout<<i<<"    "<<robot_state->getEndEffectorVelocityInBaseForLimb(limb);
//        std::cout<<"==================================="<<std::endl;
//        std::cout<<i<<"     "<<real_contact_.at(limb)<<std::endl;

//        if(real_robot)
//          {
////            if(robot_state_handle.contact_pressure_[i] > 1)
////              real_contact_.at(limb) = true;
////            else
////              real_contact_.at(limb) = false;
//            real_contact_.at(limb) = robot_state_handle.foot_contact_[i];

//          }

        if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::SwingNormal)
          {
            if(ignore_contact_sensor)
              continue;
//            if(!is_footstep_.at(limb) && !is_cartisian_motion_.at(limb))
//              continue;
            if(real_contact_.at(limb) && (limbs_state.at(limb)->getState() == StateSwitcher::States::StanceNormal
                                          || limbs_state.at(limb)->getState() == StateSwitcher::States::SwingLateLiftOff))
              {
                limbs_state.at(limb)->setState(StateSwitcher::States::SwingLateLiftOff);
                continue;
              }
            if(!real_contact_.at(limb) && (limbs_state.at(limb)->getState() == StateSwitcher::States::SwingLateLiftOff
                                        || limbs_state.at(limb)->getState() == StateSwitcher::States::SwingBumpedIntoObstacle
                                        || limbs_state.at(limb)->getState() == StateSwitcher::States::SwingNormal
                                        || limbs_state.at(limb)->getState() == StateSwitcher::States::StanceNormal))
              {
                limbs_state.at(limb)->setState(StateSwitcher::States::SwingNormal);
                continue;
              }
            if(sw_phase.at(limb)>0.5 && real_contact_.at(limb) && limbs_state.at(limb)->getState() == StateSwitcher::States::SwingNormal)
              {
                limbs_state.at(limb)->setState(StateSwitcher::States::SwingEarlyTouchDown);
                continue;
              }
//            if(real_contact_.at(limb) && limbs_state.at(limb)->getLastState() == StateSwitcher::States::SwingNormal)
//              {
//                limbs_state.at(limb)->setState(StateSwitcher::States::SwingBumpedIntoObstacle);
//                continue;
//              }
//            if(real_contact_.at(limb) && limbs_state.at(limb)->getLastState() == StateSwitcher::States::SwingBumpedIntoObstacle)
//              {
//                limbs_state.at(limb)->setState(StateSwitcher::States::SwingNormal);
//                continue;
//              }

          }

        if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::StanceNormal)
          {
            if(ignore_contact_sensor)
              {
                limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
                continue;
              }

//            if(!is_footstep_.at(limb) && !is_cartisian_motion_.at(limb))
//              {
//                if(real_contact_.at(limb))
//                  limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
//                else
//                  limbs_state.at(limb)->setState(StateSwitcher::States::SwingNormal);
//                continue;
//              }

            if(limbs_state.at(limb)->getState()==StateSwitcher::States::SwingEarlyTouchDown)
              {
                limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
                continue;
              }
            if(real_contact_.at(limb)
               && (limbs_state.at(limb)->getState()==StateSwitcher::States::SwingNormal
               || limbs_state.at(limb)->getState()==StateSwitcher::States::SwingLatelyTouchDown))
              {
                limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
                continue;
              }
            if(!real_contact_.at(limb) && (limbs_state.at(limb)->getState()==StateSwitcher::States::SwingNormal))
              {
                limbs_state.at(limb)->setState(StateSwitcher::States::SwingLatelyTouchDown);
                continue;
              }
            if(real_contact_.at(limb) && limbs_state.at(limb)->getState()==StateSwitcher::States::StanceNormal)
            {
                limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
                continue;
              }

//            if(!real_contact_.at(limb) && limbs_state.at(limb)->getLastState()==StateSwitcher::States::StanceNormal)
//            {
//                limbs_state.at(limb)->setState(StateSwitcher::States::StanceLostContact);
//               continue;
//            }


      }


      }
  }
  void RosBalanceController::footContactsCallback(const sim_assiants::FootContactsConstPtr& foot_contacts)
  {
    /****************
* TODO(Shunyao) : change contact state for the early or late contact
****************/
    unsigned int i = 0;
      ROS_WARN_ONCE("FOOT CALLBACK!!!!!!!!");
    for(unsigned int i = 0;i<foot_contacts->foot_contacts.size();i++)
      {
        auto contact = foot_contacts->foot_contacts[i];
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
//        ROS_INFO("swing time for leg %d is : %f", i, t_sw0.at(limb).toSec());
//        real_contact_.at(limb) = contact.is_contact;
//        real_contact_force_.at(limb).z() = contact.contact_force.wrench.force.z;
        real_contact_.at(limb) = contact.is_contact;
        real_contact_force_.at(limb).z() = contact.contact_force.wrench.force.z;
//        real_contact_.at(static_cast<free_gait::LimbEnum>(0))=0;
//        real_contact_.at(static_cast<free_gait::LimbEnum>(3))=0;
//        if(contact.is_contact)
//          limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
//        else
//          limbs_state.at(limb)->setState(StateSwitcher::States::SwingNormal);
//        ROS_INFO("desired limb state of leg %d is %d", i, static_cast<int>(limbs_desired_state.at(limb)->getState()));

        /*
         ********
         */
//        if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::SwingNormal)
//          {
//            limbs_state.at(limb)->setState(StateSwitcher::States::SwingNormal);
//            if(ignore_contact_sensor)
//              continue;
//            if(!is_footstep_.at(limb) && !is_cartisian_motion_.at(limb))
//              continue;
//            limbs_state.at(limb)->setState(StateSwitcher::States::SwingNormal);
//            if(sw_phase.at(limb)>0.5)//(ros::Time::now().toSec() - t_sw0.at(limb).toSec()) > 0.2)
//              {
//                if(contact.is_contact)
//                  {
//                    limbs_state.at(limb)->setState(StateSwitcher::States::SwingEarlyTouchDown);
//                  }/*else{
//                    limbs_state.at(limb)->setState(StateSwitcher::States::SwingBumpedIntoObstacle);
//                  }*/
//              }else if(sw_phase.at(limb)>0.3){
//                if(contact.is_contact)
//                  {
//                    limbs_state.at(limb)->setState(StateSwitcher::States::SwingBumpedIntoObstacle);
//                  }
//              }else if(sw_phase.at(limb)<0.3){
//                if(contact.is_contact)
//                  {
//                    limbs_state.at(limb)->setState(StateSwitcher::States::SwingLateLiftOff);
//                  }
//              }




//          }

//        if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::StanceNormal)
//          {
//            if(ignore_contact_sensor)
//              {
//                limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
//                continue;
//              }
//            if(!is_footstep_.at(limb) && !is_cartisian_motion_.at(limb))
//              {
//                if(contact.is_contact)
//                  limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
//                else
//                  limbs_state.at(limb)->setState(StateSwitcher::States::SwingNormal);
//                continue;
//              }

//            if(contact.is_contact){
//                limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
//              } else if(st_phase.at(limb) < 0.2){
//                limbs_state.at(limb)->setState(StateSwitcher::States::SwingLatelyTouchDown);
//              }
//            if(st_phase.at(limb) > 0.2)//(ros::Time::now().toSec() - t_st0.at(limb).toSec()) > 0.2)
//              {
//                if(!contact.is_contact){
//                    limbs_state.at(limb)->setState(StateSwitcher::States::StanceLostContact);
//                  }
//              }

//          }

        /*
         ********
         */
////        robot_state_handle.foot_contact_[i] = static_cast<int>(limbs_state.at(limb)->getState());
////        i++;
//        limbs_last_state.at(limb)->setState(limbs_state.at(limb)->getState());
      }
  }

  void RosBalanceController::starting(const ros::Time& time)
  {
//    for(int i = 0;i<4;i++)
//      robot_state_handle.foot_contact_[i] = 1;
//    base_command_pose_.clear();
//    base_actual_pose_.clear();
    leg_states_.clear();
    joint_actual_.clear();
    joint_command_.clear();
    foot_desired_contact_.clear();
    leg_phases_.clear();
    desired_robot_state_.clear();
    actual_robot_state_.clear();
    vitual_force_torque_.clear();
    desired_vitual_force_torque_.clear();
    log_time_.clear();
    motor_status_word_.clear();
    base_actual_pose_.clear();
    base_command_pose_.clear();
    pos_error_.clear();
    vel_error_.clear();
    for(int i = 0;i<joints.size();i++)
      {
        joints[i].setCommand(robot_state_handle.getJointEffortRead()[i]);
      }
    for(int i =0;i<4;i++)
      {
        limbs_state.at(static_cast<free_gait::LimbEnum>(i))->setState(StateSwitcher::States::StanceNormal);
        robot_state->setSupportLeg(static_cast<free_gait::LimbEnum>(i), true);
      }
  };

  void RosBalanceController::stopping(const ros::Time& time)
  {
    for(int i = 0;i<joints.size();i++)
      {
        joints[i].setCommand(0);
      }
  };

  void RosBalanceController::enforceJointLimits(double &command, unsigned int index)
    {
      // Check that this joint has applicable limits
      if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
      {
        if( command > joint_urdfs_[index]->limits->upper ) // above upper limnit
        {
          command = joint_urdfs_[index]->limits->upper;
        }
        else if( command < joint_urdfs_[index]->limits->lower ) // below lower limit
        {
          command = joint_urdfs_[index]->limits->lower;
        }
      }
    }

  bool RosBalanceController::jointTorquesLimit(free_gait::JointEffortsLeg& joint_torque, double max_torque)
  {
//    int max_index;
    double max_value=0;
    for(int i=0;i<3;i++)
      {
        if(fabs(joint_torque(i))>max_value)
          {
            max_value = fabs(joint_torque(i));
//            max_index = i;
          }
      }
    if(max_value<max_torque)
      max_torque = max_value;
    double ratio = max_torque/max_value;
    for(int i=0;i<3;i++)
      {
        joint_torque(i) = joint_torque(i)*ratio;
      }
    return true;
  }

  bool RosBalanceController::logDataCapture(std_srvs::Empty::Request& req,
                      std_srvs::Empty::Response& res)
  {
    ROS_INFO("Call to Capture Log Data");
    cout<<"desired_robot_state_.size()   "<<desired_robot_state_.size()<<endl;
    for(int index = 0; index<desired_robot_state_.size(); index++)
      {
        cout<<"index   "<<index<<endl;
        base_command_pub_.publish(base_command_pose_[index]);
        base_actual_pub_.publish(base_actual_pose_[index]);
        leg_state_pub_.publish(leg_states_[index]);
        joint_command_pub_.publish(joint_command_[index]);
        joint_actual_pub_.publish(joint_actual_[index]);
        contact_desired_pub_.publish(foot_desired_contact_[index]);
        leg_phase_pub_.publish(leg_phases_[index]);
        desired_robot_state_pub_.publish(desired_robot_state_[index]);
        actual_robot_state_pub_.publish(actual_robot_state_[index]);
        vmc_info_pub_.publish(vitual_force_torque_[index]);
        desired_vmc_info_pub_.publish(desired_vitual_force_torque_[index]);
        motor_status_word_pub_.publish(motor_status_word_[index]);
        pos_error_pub.publish(pos_error_[index]);
        vel_error_pub.publish(vel_error_[index]);


        ros::Duration(0.0025).sleep();
      }
    return true;
  }

}



PLUGINLIB_EXPORT_CLASS(balance_controller::RosBalanceController, controller_interface::ControllerBase)
