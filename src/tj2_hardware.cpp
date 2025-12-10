#include "tj2_hardware.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <random>
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>

namespace marvin_ros2_control
{
 

hardware_interface::CallbackReturn MarvinHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Initializing TJ2 Hardware Interface...");

  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("MarvinHardware"));
  clock_ = std::make_shared<rclcpp::Clock>();
  node_ = rclcpp::Node::make_shared("MarvinHardware");
  
  node_->declare_parameter<double>("max_velocity", 10.0);
  node_->declare_parameter<double>("max_acceleration", 10.0);
  node_->declare_parameter<bool>("use_drag_mode", false);
  node_->declare_parameter<bool>("ctrl_mode", static_cast<int>(RobotCtrlMode::POSITION));
  node_->declare_parameter<std::vector<double>>("joint_imp_gain", {2,2,2,1.6,1,1,1});
  node_->declare_parameter<std::vector<double>>("joint_imp_damp", {0.4,0.4,0.4,0.4,0.4,0.4,0.4});
  node_->declare_parameter<std::vector<double>>("cart_imp_gain", {500,500,500,10,10,10,0});
  node_->declare_parameter<std::vector<double>>("cart_imp_damp", {0.1,0.1,0.1,0.3,0.3,1});

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
    
    // Get the number of joints from the hardware info
    size_t num_joints = info.joints.size();
    robot_arm_left_right_ = static_cast<int>(RobotArmConfig::LEFT_ARM);
    robot_ctrl_mode_ = static_cast<int>(RobotCtrlMode::POSITION);

    if (info_.hardware_parameters.find("left_right_arm") != info_.hardware_parameters.end()) {
      robot_arm_left_right_ = std::stoi(info_.hardware_parameters.at("left_right_arm"));
      RCLCPP_INFO(get_logger(), "Found left_right_arm parameter: %d", robot_arm_left_right_);
    } else {
      RCLCPP_WARN(get_logger(), "No left_right_arm parameter found, using default: %d", robot_arm_left_right_);
    }

    if (info_.hardware_parameters.find("ctrl_mode") != info_.hardware_parameters.end()) {
      robot_ctrl_mode_ = std::stoi(info_.hardware_parameters.at("ctrl_mode"));
      RCLCPP_INFO(get_logger(), "Found ctrl mode parameter: %d", robot_ctrl_mode_);
    } else {
      RCLCPP_WARN(get_logger(), "No ctrl mode found, using default: %d", robot_ctrl_mode_);
    }
   
    RCLCPP_INFO(get_logger(), "Initializing %zu joints", num_joints);

    // Resize all arrays based on the number of joints
    hw_position_commands_.resize(num_joints, 0.0);
    hw_velocity_commands_.resize(num_joints, 0.0);
    hw_position_states_.resize(num_joints, 0.0);
    hw_velocity_states_.resize(num_joints, 0.0);
    hw_effort_states_.resize(num_joints, 0.0);

    // 初始化夹爪参数 如果有的话
    has_gripper_ = false;
    gripper_joint_index_ = -1;
    gripper_initilized_ = false;
    gripper_joint_name_ = {};
    contains_gripper();

    if (has_gripper_)
    {
      if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM) || robot_arm_left_right_ == static_cast<int>(RobotArmConfig::RIGHT_ARM))
      {
        
        gripper_position_command_ = {-1.0};
        gripper_position_ = {0.0};
        gripper_velocity_ = {0.0};
        gripper_effort_ = {0.0};
        last_gripper_command_ = {-1.0};
        last_gripper_position_ = {-1.0};
        gripper_stopped_ = {true};
        step_size_ = {0.0};
        gripper_ptr_.reserve(1);
        // std::unique_ptr<marvin_ros2_control::ModbusGripper> gripper_ptr = std::make_unique<marvin_ros2_control::JDGripper>();
        if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM))
        {
          gripper_ptr_.emplace_back(std::make_unique<marvin_ros2_control::JDGripper>(OnClearChDataA, OnSetChDataA));
        }
        else
        {
          gripper_ptr_.emplace_back(std::make_unique<marvin_ros2_control::JDGripper>(OnClearChDataB, OnSetChDataB));
        }
      }
      else if(robot_arm_left_right_ == static_cast<int>(RobotArmConfig::DUAL_ARM))
      {
        
        gripper_position_command_ = {-1.0, -1.0};
        gripper_position_ = {0.0, 0.0};
        gripper_velocity_ = {0.0, 0.0};
        gripper_effort_ = {0.0, 0.0};
        last_gripper_command_ = {-1.0, -1.0};
        last_gripper_position_ = {-1.0, -1.0};
        gripper_stopped_ = {true, true};
        step_size_ = {0.0, 0.0};
        //std::unique_ptr<marvin_ros2_control::ModbusGripper> left_gripper = std::make_unique<marvin_ros2_control::JDGripper>();
        //std::unique_ptr<marvin_ros2_control::ModbusGripper> right_gripper = std::make_unique<marvin_ros2_control::JDGripper>();
        //gripper_ptr_.push_back(std::move(left_gripper));
        //gripper_ptr_.push_back(std::move(right_gripper));
        gripper_ptr_.reserve(2);
        gripper_ptr_.emplace_back(std::make_unique<marvin_ros2_control::JDGripper>(OnClearChDataA, OnSetChDataA));
        gripper_ptr_.emplace_back(std::make_unique<marvin_ros2_control::JDGripper>(OnClearChDataB, OnSetChDataB));
      }
    } 
    
    // Initialize hardware connection status
    hardware_connected_ = false;
    simulation_active_ = false;
    param_callback_handle_ = node_->add_on_set_parameters_callback(
            std::bind(&MarvinHardware::paramCallback, this, std::placeholders::_1));
    return hardware_interface::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult
MarvinHardware::paramCallback(const std::vector<rclcpp::Parameter> & params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : params) {
        if (param.get_name() == "joint_imp_gain" || param.get_name() == "joint_imp_damp" && robot_ctrl_mode_ == static_cast<int>(RobotCtrlMode::JOINT_IMPEDANCE)) {
          // change gain parameter
          RCLCPP_INFO(get_logger(), "change impedance parameters");
        }
        if (param.get_name() == "ctrl_mode")
        {
          robot_ctrl_mode_ = param.as_int();
          RCLCPP_INFO(get_logger(), "param robot ctrl mode changed to %d", robot_ctrl_mode_);
          if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM))
          {
            setLeftArmCtrl();
          }
          else if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::RIGHT_ARM))
          {
            setRightArmCtrl();
          }
          else if(robot_arm_left_right_ == static_cast<int>(RobotArmConfig::DUAL_ARM))
          {
            setLeftArmCtrl();
            setRightArmCtrl();
          }
        }
    }
    return result;
}

void MarvinHardware::setLeftArmCtrl()
{
  /// clear error first
  double kineParam[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double dynPara[10] = {2.0, 0.0, 0.0, 96.411347, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  OnClearSet();
  OnClearErr_A();
  RCLCPP_INFO(get_logger(), "set tool load parameters");
  OnSetSend();
  usleep(100000);

  if (robot_ctrl_mode_ == static_cast<int>(RobotCtrlMode::POSITION))
  {
    OnClearSet();
    OnSetTargetState_A(1) ; //3:torque mode; 1:position mode
    OnSetSend();
    usleep(100000);
  }
  else
  {
    OnClearSet();
    OnSetTargetState_A(3) ; //3:torque mode; 1:position mode
    OnSetTool_A(kineParam, dynPara);
    if (robot_ctrl_mode_ == static_cast<int>(RobotCtrlMode::JOINT_IMPEDANCE))
    {
      double K[7] = {2,2,2,1.6,1,1,1};//预设为参数最大上限，供参考。
      double D[7] = {0.4,0.4,0.4,0.4,0.4,0.4,0.4};//预设为参数最大上限，供参考。
      OnSetJointKD_A(K, D);
      OnSetImpType_A(1);
    }
    else if (robot_ctrl_mode_ == static_cast<int>(RobotCtrlMode::CART_IMPEDANCE))
    {
      double K[7] = {1800,1800,1800,40,40,40,20}; //预设为参数最大上限，供参考。
      double D[7] = {0.6,0.6,0.6,0.4,0.4,0.4, 0.4};//预设为参数最大上限，供参考。
      OnSetCartKD_A(K, D, 2);
      OnSetImpType_A(2);
    }
    OnSetSend();
    usleep(100000);
  }
  
  /// set maximum speed and acceleration
  OnClearSet();
  OnSetJointLmt_A(10, 10) ;
  OnSetSend();
  usleep(100000);
}

void MarvinHardware::setRightArmCtrl()
{
  /// clear error first
  double kineParam[6] = {0, 0, 0, 0, 0, 0};
  double dynPara[10] = {1.8, 0.0, 0.0, 96.411347, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  OnClearSet();
  OnClearErr_B();


  OnSetSend();
  usleep(100000);
  if (robot_ctrl_mode_ == static_cast<int>(RobotCtrlMode::POSITION))
  {
    OnClearSet();
    OnSetTargetState_B(1) ; //3:torque mode; 1:position mode
    OnSetSend();
    usleep(100000);
  }
  else
  {
    OnClearSet();
    OnSetTargetState_B(3) ; //3:torque mode; 1:position mode
    OnSetTool_B(kineParam, dynPara);
    if (robot_ctrl_mode_ == static_cast<int>(RobotCtrlMode::JOINT_IMPEDANCE))
    {
      double K[7] = {2,2,2,1.6,1,1,1};//预设为参数最大上限，供参考。
      double D[7] = {0.4,0.4,0.4,0.4,0.4,0.4,0.4};//预设为参数最大上限，供参考。
      OnSetJointKD_B(K, D);
      OnSetImpType_B(1);
    }
    else if (robot_ctrl_mode_ == static_cast<int>(RobotCtrlMode::CART_IMPEDANCE))
    {
      double K[7] = {2500,2500,2500,60,60,60,20}; //预设为参数最大上限，供参考。
      double D[7] = {0.6,0.6,0.6,0.2,0.2,0.2, 0.4};//预设为参数最大上限，供参考。
      OnSetCartKD_B(K, D, 2);
      OnSetImpType_B(2);
    }
    OnSetSend();
    usleep(100000);
  }
  
  /// set maximum speed and acceleration
  OnClearSet();
  OnSetJointLmt_B(10, 10) ;
  OnSetSend();
  usleep(100000);
}

hardware_interface::CallbackReturn MarvinHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Configuring TJ2 Hardware Interface...");


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MarvinHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Cleaning up TJ2 Hardware Interface...");
  
  if (hardware_connected_) {
    disconnectFromHardware();
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

void MarvinHardware::contains_gripper()
{
  int joint_index = 0;
  for (const auto& joint : info_.joints) {
        // 检查关节名称中是否包含 gripper 或 hand
        std::string joint_name_lower = joint.name;
        std::transform(joint_name_lower.begin(), joint_name_lower.end(), 
                      joint_name_lower.begin(), ::tolower);
        
        if (joint_name_lower.find("gripper") != std::string::npos || 
            joint_name_lower.find("hand") != std::string::npos) {
            // 这是夹爪关节
            has_gripper_ = true;
            gripper_joint_name_.push_back(joint.name);
            gripper_joint_index_ = joint_index;
            RCLCPP_INFO(get_node()->get_logger(), "Detected gripper joint: %s (index %d)", 
                       joint.name.c_str(), gripper_joint_index_);
        } else {
            // 这是机械臂关节gripper
            joint_names_.push_back(joint.name);
        }
        joint_index++;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Detected gripper joint: %d ", gripper_joint_name_.size());
}

hardware_interface::CallbackReturn MarvinHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Activating TJ2 Hardware Interface...");
  simulation_mode_ = false;
  if (simulation_mode_) {
    RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Running in simulation mode");
    simulation_active_ = true;
    
    // Initialize simulation states
    for (size_t i = 0; i < hw_position_states_.size(); i++) {
      hw_position_states_[i] = 0.0;
      hw_velocity_states_[i] = 0.0;
      hw_effort_states_[i] = 0.0;
      hw_position_commands_[i] = 0.0;
      hw_velocity_commands_[i] = 0.0;
    }
    
  } else {
    RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Running in real hardware mode");
    // Connect to real hardware
    if (!connectToHardware()) {
      RCLCPP_ERROR(rclcpp::get_logger("MarvinHardware"), "Failed to connect to TJ2 hardware");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Read initial joint states from hardware
    if (!readFromHardware(robot_arm_left_right_, true)) {
      RCLCPP_ERROR(rclcpp::get_logger("MarvinHardware"), "Failed to read initial joint states from hardware");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize commands with current positions
    for (size_t i = 0; i < hw_position_commands_.size(); i++) {
      hw_position_commands_[i] = hw_position_states_[i];
      hw_velocity_commands_[i] = hw_velocity_states_[i];
      // RCLCPP_ERROR(rclcpp::get_logger("MarvinHardware"), "initia position ...  %f", hw_velocity_commands_[i]);
    }
  }
  OnClearSet();
  OnLogOff();
  OnLocalLogOff();
  OnSetSend();
  usleep(100000);
  

  if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM))
  {
    setLeftArmCtrl();
  }
  else if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::RIGHT_ARM))
  {
    setRightArmCtrl();
  }
  else if(robot_arm_left_right_ == static_cast<int>(RobotArmConfig::DUAL_ARM))
  {
    setLeftArmCtrl();
    setRightArmCtrl();
  }
  
  OnGetBuf(&frame_data_);
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "current state of A arm:%d\n",frame_data_.m_State[0].m_CurState);
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "cmd state of A arm:%d\n",frame_data_.m_State[0].m_CmdState);
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "error code of A arms:%d\n",frame_data_.m_State[0].m_ERRCode);
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "cmd of vel and acc:%d %d\n",frame_data_.m_In[0].m_Joint_Vel_Ratio,frame_data_.m_In[0].m_Joint_Acc_Ratio);
  
  // connect to gripper
  if (has_gripper_)
  {
    bool gripper_connected = connect_gripper();
    if (gripper_connected)
    {
      RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Gripper Connected");
      // start gripper control thread
      gripper_ctrl_thread_ = std::thread(&MarvinHardware::gripper_callback, this);
      gripper_ctrl_thread_.detach();
      std::thread recv_thread(&MarvinHardware::recv_thread_func, this);
      recv_thread.detach();
      // read onceon_activateon_activate
      // int cur_pos_status = 0;
      // int cur_speed_status = 0;
      // int cur_effort_status = 0;
      // bool success = ModbusGripper::getStatus(cur_effort_status, cur_speed_status, cur_pos_status, OnClearChDataA);
      // RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper read position %d", cur_pos_status);
      // gripper_position_ = (9000 - cur_pos_status) / 9000.0;
      // gripper_position_ = 0.0;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("MarvinHardware"), "Gripper Not Connected");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "TJ2 Hardware Interface activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void MarvinHardware::gripper_callback()
{
  while(hardware_connected_)
  {
    // OnClearSet();
    // if(!gripper_stopped_)
    // {
    //   int cur_pos_status = 0;
    //   int cur_speed_status = 0;
    //   int cur_effort_status = 0;
    //   bool success = ModbusGripper::getStatus(cur_effort_status, cur_speed_status, cur_pos_status, OnClearChDataA);
    //   RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper read position %d", cur_pos_status);
    //   gripper_position_ = (9000 - cur_pos_status) / 9000.0;
    //   RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper position %f", gripper_position_);
    //   if (gripper_position_ == last_gripper_position_)
    //   {
    //     gripper_stopped_ = true;
    //   }
    //   else
    //   {
    //     last_gripper_position_ = gripper_position_;
    //   }
    // }
    for(int i =0; i < gripper_stopped_.size(); i++)
    {
      if(!gripper_stopped_[i])
      {
        //RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper write position %f", step_size_);
        gripper_position_[i] = gripper_position_[i] + step_size_[i];
        //RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper write position %f", gripper_position_);
        //RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper write position %f", last_gripper_command_);
        // RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper write position %d", last_gripper_command_ == gripper_position_);
        if (abs(gripper_position_[i]- last_gripper_command_[i]) < 0.001)
        {
          gripper_stopped_[i] = true;
          step_size_[i] = 0.0;
        }
      }

    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    /// write when the gripper commannd is different
    bool success;
    for(int i =0; i < gripper_stopped_.size(); i++)
    {
      if (last_gripper_command_[i] != gripper_position_command_[i])
      {
        RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper %d write position %f", i, gripper_position_command_[i]);
        // write commands to gripper
        int cur_pos_set = 256 - 256 * gripper_position_command_[i];
        int cur_speed_set = 100;
        int cur_effort_set = 100;
        RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper write position %d", cur_pos_set);
        success = gripper_ptr_[i]->move_gripper(cur_effort_set, cur_speed_set, cur_pos_set);
        last_gripper_command_[i] = gripper_position_command_[i];
        gripper_stopped_[i]= false;
        step_size_[i]= (last_gripper_command_[i] - gripper_position_[i]) / 10.0;
      }
    }
    // OnSetSend();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

bool MarvinHardware::connect_gripper()
{
   bool result = true;
   for(int i =0; i < gripper_ptr_.size(); i++)
   {
      result = result && gripper_ptr_[i]->initialize();
   }
   return result;
}

hardware_interface::CallbackReturn MarvinHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Deactivating TJ2 Hardware Interface...");

  simulation_active_ = false;
  OnClearSet();
  if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM))
  {
    OnSetDragSpace_A(0);
    OnSetTargetState_A(0);
    
  }
  else if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::RIGHT_ARM))
  {
    OnSetTargetState_B(0);
  }
  else if(robot_arm_left_right_ == static_cast<int>(RobotArmConfig::DUAL_ARM))
  {
    OnSetTargetState_A(0);
    OnSetTargetState_B(0);
  }
  
  OnSetSend();
  usleep(100000);

  if (hardware_connected_) {
    disconnectFromHardware();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MarvinHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Shutting down TJ2 Hardware Interface...");
  
  if (hardware_connected_) {
    disconnectFromHardware();
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MarvinHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(rclcpp::get_logger("MarvinHardware"), "Error in TJ2 Hardware Interface");
  
  // Attempt to safely disconnect from hardware
  if (hardware_connected_) {
    disconnectFromHardware();
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MarvinHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("MarvinHardware"), "Exporting state interfacesExporting for %zu joints", info_.joints.size());
  std::vector<hardware_interface::StateInterface> state_interfaces;
  int joint_name_sz = joint_names_.size();
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Exporting state interfaces for %zu joints", joint_name_sz);
 
  for (size_t i = 0; i < joint_name_sz; i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_position_states_[i]));
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocity_states_[i]));
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_EFFORT,
        &hw_effort_states_[i]));
  }
  if (has_gripper_)
  {
    for(int i =0; i < gripper_joint_name_.size(); i++)
    {
      
      state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        gripper_joint_name_[i],
        hardware_interface::HW_IF_POSITION,
        &gripper_position_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        gripper_joint_name_[i],
        hardware_interface::HW_IF_VELOCITY,
        &gripper_velocity_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        gripper_joint_name_[i],
        hardware_interface::HW_IF_EFFORT,
        &gripper_effort_[i]));
    }
    
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MarvinHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("MarvinHardware"), "Exporting command interfaces for %zu joints", info_.joints.size());

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  int joint_name_sz = joint_names_.size();
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Exporting command  interfaces for %zu joints", joint_name_sz);
  for (size_t i = 0; i < joint_name_sz; i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_position_commands_[i]));
    
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocity_commands_[i]));
  }

  if (has_gripper_)
  {
    for(int i =0; i < gripper_joint_name_.size(); i++)
    {

      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          gripper_joint_name_[i],
          hardware_interface::HW_IF_POSITION,
          &gripper_position_command_[i]));
    }
  }

  return command_interfaces;
}

hardware_interface::return_type MarvinHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // if (simulation_active_) {
  //   simulateHardware(period);
  //   return hardware_interface::return_type::OK;
  // }

  if (!hardware_connected_) {
    // RCLCPP_ERROR_THROTTLE(
    //   rclcpp::get_logger("MarvinHardware"), 
    //   *std::make_shared<rclcpp::Clock>(), 5000, 
    //   "Not connected to hardware");
    return hardware_interface::return_type::ERROR;
  }

  if (!readFromHardware(robot_arm_left_right_, false)) {
    // RCLCPP_ERROR_THROTTLE(
    //   rclcpp::get_logger("MarvinHardware"), 
    //   *std::make_shared<rclcpp::Clock>(), 5000, 
    //   "Failed to read from hardware");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MarvinHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // if (simulation_active_) {
  //   // In simulation, commands are handled in the read method
  //   return hardware_interface::return_type::OK;
  // }

  if (!hardware_connected_) {
    return hardware_interface::return_type::ERROR;
  }
  /// convert rad to degree
  // RCLCPP_ERROR(rclcpp::get_logger("MarvinHardware"), "write command ...  %f", hw_position_commands_[2]);
  std::vector<double> hw_commands;
  for(int i =0; i < hw_position_commands_.size(); i++)
  {
    hw_commands.push_back(radToDegree(hw_position_commands_[i]));
  }
  // RCLCPP_ERROR(rclcpp::get_logger("MarvinHardware"), "write command deg ...  %f", hw_position_commands_[2]);
  // Enforce joint limits before sending commands
  // enforceJointLimits();  

  if (!writeToHardware(robot_arm_left_right_, hw_commands)) {
    // RCLCPP_ERROR_THROTTLE(
    //   rclcpp::get_logger("MarvinHardware"),
    //   *clock_, 5000,
    //   "Failed to write to hardware");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}
bool MarvinHardware::recv_thread_func() {
    long ch = 2;                    
    unsigned char data_buf[256] = {0};
    char hex_str[512];

    while (hardware_connected_) {
        if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM))
        {
          int size = OnGetChDataA(data_buf, &ch);  
          // if (size > 0 && ch == 2) {
          //     hex_to_str(data_buf, size, hex_str, sizeof(hex_str));
          //     printf("接收字节数: %d, 通道: %ld, HEX: %s\n", size, ch, hex_str);
          // }
        }
        else if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::RIGHT_ARM))
        {
          int size = OnGetChDataB(data_buf, &ch);  
        }
        else if(robot_arm_left_right_ == static_cast<int>(RobotArmConfig::DUAL_ARM))
        {
          int size = OnGetChDataA(data_buf, &ch);
          size = OnGetChDataB(data_buf, &ch);
        }
        usleep(200 * 1000); 
    }
}

bool MarvinHardware::connectToHardware()
{
  device_ip_ = "192.168.1.190";
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Connecting to TJ2 at %s:%d", device_ip_.c_str(), device_port_);
  
  // TODO: Implement actual Dobot connection using Dobot SDK
  // Example:
  unsigned char octet1;
  unsigned char octet2;
  unsigned char octet3;
  unsigned char octet4;
  
  // hardware_connected_ = OnLinkTo(octet1,octet2,octet3,octet4) == true;
  hardware_connected_ = OnLinkTo(192,168,1,190);
      
  
  // Simulate connection for demonstration
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // hardware_connected_ = true;
  if (hardware_connected_)
  {
    RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Successfully connected to TJ2 hardware");
    usleep(100000);
    OnClearSet();
    OnClearErr_A();
    OnClearErr_B();
    OnSetSend();
    usleep(100000);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Failed to connect to TJ2 hardware");
  }
  
  return hardware_connected_;
}

void MarvinHardware::disconnectFromHardware()
{
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Disconnecting from Dobot CR5 hardware");
  
  // TODO: Implement actual Dobot disconnection
  OnRelease();
  
  hardware_connected_ = false;
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Disconnected from Dobot CR5 hardware");
}

bool MarvinHardware::readFromHardware(int robot_arm_left_right_, bool initial_frame)
{
  
  OnGetBuf(&frame_data_);
    // RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Reading from hardware interface ... %d", frame_data_.m_Out[robot_arm_left_right_].m_OutFrameSerial);
  if (initial_frame)
  {
    previous_message_frame_ = frame_data_.m_Out[robot_arm_left_right_].m_OutFrameSerial;
  }
  if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM)|| robot_arm_left_right_ == static_cast<int>(RobotArmConfig::RIGHT_ARM))
  {
    for (size_t i = 0; i < 7; i++) {
      hw_position_states_[i] = degreeToRad(frame_data_.m_Out[robot_arm_left_right_].m_FB_Joint_Pos[i]);
      hw_velocity_states_[i] = frame_data_.m_Out[robot_arm_left_right_].m_FB_Joint_Vel[i];
      hw_effort_states_[i] = frame_data_.m_Out[robot_arm_left_right_].m_FB_Joint_SToq[i];
    }
  }
  else if(robot_arm_left_right_ == static_cast<int>(RobotArmConfig::DUAL_ARM))
  {
    for (size_t i = 0; i < 7; i++) {
      hw_position_states_[i] = degreeToRad(frame_data_.m_Out[static_cast<int>(RobotArmConfig::LEFT_ARM)].m_FB_Joint_Pos[i]);
      hw_velocity_states_[i] = frame_data_.m_Out[static_cast<int>(RobotArmConfig::LEFT_ARM)].m_FB_Joint_Vel[i];
      hw_effort_states_[i] = frame_data_.m_Out[static_cast<int>(RobotArmConfig::LEFT_ARM)].m_FB_Joint_SToq[i];
    }

    for (size_t i = 0; i < 7; i++) {
      hw_position_states_[i + 7] = degreeToRad(frame_data_.m_Out[static_cast<int>(RobotArmConfig::RIGHT_ARM)].m_FB_Joint_Pos[i]);
      hw_velocity_states_[i + 7] = frame_data_.m_Out[static_cast<int>(RobotArmConfig::RIGHT_ARM)].m_FB_Joint_Vel[i];
      hw_effort_states_[i + 7] = frame_data_.m_Out[static_cast<int>(RobotArmConfig::RIGHT_ARM)].m_FB_Joint_SToq[i];
    }
  }
  previous_message_frame_ = frame_data_.m_Out[robot_arm_left_right_].m_OutFrameSerial;

    if (previous_message_frame_ - frame_data_.m_Out[robot_arm_left_right_].m_OutFrameSerial < 2)
    {
      return true;
    }
   else
   {
        RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Missing more than 2 frames");
        return true;
   }
}

bool MarvinHardware::writeToHardware(int robot_arm_left_right, std::vector<double> & hw_commands)
{
  // TODO: Implement actual joint command sending to Dobot
  bool result = true;
  OnClearSet(); 
  if (robot_arm_left_right == static_cast<int>(RobotArmConfig::LEFT_ARM))
  {
    result = OnSetJointCmdPos_A(hw_commands.data());
  }
  else if (robot_arm_left_right == static_cast<int>(RobotArmConfig::RIGHT_ARM))
  {
    result = OnSetJointCmdPos_B(hw_commands.data());
  }
  else if (robot_arm_left_right == static_cast<int>(RobotArmConfig::DUAL_ARM))
  {
    result = OnSetJointCmdPos_A(hw_commands.data());
    // RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "write to left arm %d", result);
    result = result && OnSetJointCmdPos_B(hw_commands.data() + 7);
    // RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "write to right arm %d", result);
  }
  OnSetSend();
  return result;
}

void MarvinHardware::simulateHardware(const rclcpp::Duration & period)
{
  // Simple simulation: move toward commanded position with velocity limits
  for (size_t i = 0; i < hw_position_states_.size(); i++) {
    double position_error = hw_position_commands_[i] - hw_position_states_[i];
    double max_velocity_change = velocity_limits_[i] * period.seconds();
    
    // Calculate desired velocity
    double desired_velocity = std::copysign(
      std::min(std::abs(position_error) / period.seconds(), velocity_limits_[i]),
      position_error
    );
    
    // Apply velocity limits
    double velocity_change = desired_velocity - hw_velocity_states_[i];
    if (std::abs(velocity_change) > max_velocity_change) {
      velocity_change = std::copysign(max_velocity_change, velocity_change);
    }
    
    hw_velocity_states_[i] += velocity_change;
    hw_position_states_[i] += hw_velocity_states_[i] * period.seconds();
    
    // Simulate effort based on acceleration
    hw_effort_states_[i] = velocity_change / period.seconds() * 0.1;
  }
}

void MarvinHardware::enforceJointLimits()
{
  if (hw_position_commands_.size() != position_lower_limits_.size() || 
      hw_position_commands_.size() != position_upper_limits_.size()) {
    RCLCPP_ERROR(*logger_, "Array size mismatch in enforceJointLimits!");
    return;
  }

  for (size_t i = 0; i < hw_position_commands_.size(); i++) {
    if (hw_position_commands_[i] < position_lower_limits_[i]) {
      hw_position_commands_[i] = position_lower_limits_[i];
      RCLCPP_WARN_THROTTLE(
        *logger_, 
        *clock_, 10000,  // Use the class member clock
        "Joint %s position command (%.3f) below lower limit (%.3f)", 
        info_.joints[i].name.c_str(), hw_position_commands_[i], position_lower_limits_[i]);
    } else if (hw_position_commands_[i] > position_upper_limits_[i]) {
      hw_position_commands_[i] = position_upper_limits_[i];
      RCLCPP_WARN_THROTTLE(
        *logger_, 
        *clock_, 10000,
        "Joint %s position command (%.3f) above upper limit (%.3f)", 
        info_.joints[i].name.c_str(), hw_position_commands_[i], position_upper_limits_[i]);
    }
    
    // Add safety check for velocity arrays too
    if (i < hw_velocity_commands_.size() && i < velocity_limits_.size() && 
        std::abs(hw_velocity_commands_[i]) > velocity_limits_[i]) {
      hw_velocity_commands_[i] = std::copysign(velocity_limits_[i], hw_velocity_commands_[i]);
    }
  }
}

bool MarvinHardware::initializeJointLimits()
{
  // position_lower_limits_.resize(info_.joints.size());
  // position_upper_limits_.resize(info_.joints.size());
  // velocity_limits_.resize(info_.joints.size());
  // effort_limits_.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++) {
    const auto & joint = info_.joints[i];
    
    // Parse position limits
    for (const auto & command_interface : joint.command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_POSITION) {
        if (!command_interface.min.empty()) {
          position_lower_limits_[i] = std::stod(command_interface.min);
        }
        if (!command_interface.max.empty()) {
          position_upper_limits_[i] = std::stod(command_interface.max);
        }
      } else if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
        if (!command_interface.max.empty()) {
          velocity_limits_[i] = std::stod(command_interface.max);
        }
      }
    }
    
    // Parse effort limits from joint limits in URDF
    effort_limits_[i] = std::stod(joint.command_interfaces[0].max);
    
    RCLCPP_DEBUG(
      rclcpp::get_logger("MarvinHardware"),
      "Joint %s: pos=[%.3f, %.3f], vel_limit=%.3f, effort_limit=%.3f",
      joint.name.c_str(), position_lower_limits_[i], position_upper_limits_[i],
      velocity_limits_[i], effort_limits_[i]);
  }

  return true;
}

void MarvinHardware::logJointStates()
{
  std::stringstream ss;
  ss << "Joint States - ";
  for (size_t i = 0; i < hw_position_states_.size(); i++) {
    ss << info_.joints[i].name << ": pos=" << std::fixed << std::setprecision(3) << hw_position_states_[i]
       << ", vel=" << hw_velocity_states_[i] << ", cmd=" << hw_position_commands_[i];
    if (i < hw_position_states_.size() - 1) ss << " | ";
  }
  RCLCPP_DEBUG(rclcpp::get_logger("MarvinHardware"), "%s", ss.str().c_str());
}

}  // namespace marvin_ros2_control


PLUGINLIB_EXPORT_CLASS(marvin_ros2_control::MarvinHardware, hardware_interface::SystemInterface)

