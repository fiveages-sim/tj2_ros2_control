#include "marvin_hardware.h"

#include <chrono>
#include <cmath>
#include <thread>
#include <random>
#include <sstream>
#include <algorithm>
#include <cctype>
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>

namespace marvin_ros2_control
{
    // 辅助函数：标准化字符串（转大写，去除空格）
    static std::string normalizeString(const std::string& str)
    {
        std::string result = str;
        std::transform(result.begin(), result.end(), result.begin(), ::toupper);
        result.erase(std::remove_if(result.begin(), result.end(), ::isspace), result.end());
        return result;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params)
    {
        if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "Initializing Marvin Hardware Interface...");

        // 解析配置参数
        const auto get_param = [this](const std::string& name, const std::string& default_val) {
            if (auto it = info_.hardware_parameters.find(name); 
                it != info_.hardware_parameters.end()) {
                return it->second;
            }
            return default_val;
        };

        // 解析数组参数的辅助函数
        const auto parse_double_array = [](const std::string& str, const std::vector<double>& default_val) -> std::vector<double> {
            if (str.empty()) {
                return default_val;
            }
            std::vector<double> result;
            std::string cleaned = str;
            cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '['), cleaned.end());
            cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), ']'), cleaned.end());
            std::istringstream iss(cleaned);
            std::string token;
            while (std::getline(iss, token, ',')) {
                if (!token.empty()) {
                    result.push_back(std::stod(token));
                }
            }
            return result.empty() ? default_val : result;
        };

        // Get the number of joints from the hardware info
        size_t num_joints = params.hardware_info.joints.size();

        // 获取机器人配置参数
        std::string arm_config_raw = get_param("arm_type", "LEFT");
        std::string arm_config_normalized = normalizeString(arm_config_raw);
        
        if (arm_config_normalized == "LEFT")
        {
            robot_arm_config_ = "LEFT";
            robot_arm_index_ = ARM_LEFT;
        }
        else if (arm_config_normalized == "RIGHT")
        {
            robot_arm_config_ = "RIGHT";
            robot_arm_index_ = ARM_RIGHT;
        }
        else if (arm_config_normalized == "DUAL")
        {
            robot_arm_config_ = "DUAL";
            robot_arm_index_ = ARM_DUAL;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Unknown arm_type '%s', using default LEFT. Valid options: LEFT, RIGHT, DUAL", arm_config_raw.c_str());
            robot_arm_config_ = "LEFT";
            robot_arm_index_ = ARM_LEFT;
        }

        // 获取控制模式参数（直接使用字符串）
        std::string ctrl_mode_raw = get_param("ctrl_mode", "POSITION");
        std::string ctrl_mode_normalized = normalizeString(ctrl_mode_raw);
        
        // 支持多种字符串格式
        if (ctrl_mode_normalized == "POSITION" || ctrl_mode_normalized == "POS" || ctrl_mode_normalized == "2")
        {
            robot_ctrl_mode_ = "POSITION";
        }
        else if (ctrl_mode_normalized == "JOINT_IMPEDANCE" || ctrl_mode_normalized == "JOINT_IMP" || ctrl_mode_normalized == "JOINT" || ctrl_mode_normalized == "3")
        {
            robot_ctrl_mode_ = "JOINT_IMPEDANCE";
        }
        else if (ctrl_mode_normalized == "CART_IMPEDANCE" || ctrl_mode_normalized == "CART_IMP" || ctrl_mode_normalized == "CART" || ctrl_mode_normalized == "CARTESIAN" || ctrl_mode_normalized == "4")
        {
            robot_ctrl_mode_ = "CART_IMPEDANCE";
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Unknown control mode '%s', using default POSITION", ctrl_mode_raw.c_str());
            robot_ctrl_mode_ = "POSITION";
        }

        RCLCPP_INFO(get_logger(), "Robot arm configuration: %s", robot_arm_config_.c_str());
        RCLCPP_INFO(get_logger(), "Control mode: %s", robot_ctrl_mode_.c_str());

        // 获取硬件连接参数
        device_ip_ = get_param("device_ip", "192.168.1.190");
        device_port_ = std::stoi(get_param("device_port", "8080"));
        RCLCPP_INFO(get_logger(), "Device IP: %s, Port: %d", device_ip_.c_str(), device_port_);

        // 获取阻抗控制参数（如果提供）
        std::string joint_imp_gain_str = get_param("joint_imp_gain", "");
        std::string joint_imp_damp_str = get_param("joint_imp_damp", "");
        std::string cart_imp_gain_str = get_param("cart_imp_gain", "");
        std::string cart_imp_damp_str = get_param("cart_imp_damp", "");

        // 解析阻抗参数
        joint_imp_gain_ = parse_double_array(joint_imp_gain_str, {2, 2, 2, 1.6, 1, 1, 1});
        joint_imp_damp_ = parse_double_array(joint_imp_damp_str, {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4});
        cart_imp_gain_ = parse_double_array(cart_imp_gain_str, {500, 500, 500, 10, 10, 10, 0});
        cart_imp_damp_ = parse_double_array(cart_imp_damp_str, {0.1, 0.1, 0.1, 0.3, 0.3, 1});

        // 获取其他参数
        max_velocity_ = std::stod(get_param("max_velocity", "10.0"));
        max_acceleration_ = std::stod(get_param("max_acceleration", "10.0"));
        use_drag_mode_ = (get_param("use_drag_mode", "false") == "true");

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
            if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_RIGHT)
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
                if (robot_arm_index_ == ARM_LEFT)
                {
                    gripper_ptr_.emplace_back(
                        std::make_unique<JDGripper>(OnClearChDataA, OnSetChDataA));
                }
                else
                {
                    gripper_ptr_.emplace_back(
                        std::make_unique<JDGripper>(OnClearChDataB, OnSetChDataB));
                }
            }
            else if (robot_arm_index_ == ARM_DUAL)
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
                gripper_ptr_.emplace_back(
                    std::make_unique<marvin_ros2_control::JDGripper>(OnClearChDataA, OnSetChDataA));
                gripper_ptr_.emplace_back(
                    std::make_unique<marvin_ros2_control::JDGripper>(OnClearChDataB, OnSetChDataB));
            }
        }

        // Initialize hardware connection status
        hardware_connected_ = false;
        simulation_active_ = false;
        
        RCLCPP_INFO(get_logger(), "Marvin Hardware Interface initialized successfully");
        return hardware_interface::CallbackReturn::SUCCESS;
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

        if (robot_ctrl_mode_ == "POSITION")
        {
            OnClearSet();
            OnSetTargetState_A(1); //3:torque mode; 1:position mode
            OnSetSend();
            usleep(100000);
        }
        else
        {
            OnClearSet();
            OnSetTargetState_A(3); //3:torque mode; 1:position mode
            OnSetTool_A(kineParam, dynPara);
            if (robot_ctrl_mode_ == "JOINT_IMPEDANCE")
            {
                // 使用从硬件参数中读取的阻抗参数
                if (joint_imp_gain_.size() >= 7 && joint_imp_damp_.size() >= 7)
                {
                    double K[7];
                    double D[7];
                    for (size_t i = 0; i < 7; i++)
                    {
                        K[i] = joint_imp_gain_[i];
                        D[i] = joint_imp_damp_[i];
                    }
                    OnSetJointKD_A(K, D);
                    OnSetImpType_A(1);
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Joint impedance parameters size mismatch, using defaults");
                    double K[7] = {2, 2, 2, 1.6, 1, 1, 1};
                    double D[7] = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
                    OnSetJointKD_A(K, D);
                    OnSetImpType_A(1);
                }
            }
            else if (robot_ctrl_mode_ == "CART_IMPEDANCE")
            {
                // 使用从硬件参数中读取的阻抗参数
                if (cart_imp_gain_.size() >= 7 && cart_imp_damp_.size() >= 7)
                {
                    double K[7];
                    double D[7];
                    for (size_t i = 0; i < 7; i++)
                    {
                        K[i] = cart_imp_gain_[i];
                        D[i] = cart_imp_damp_[i];
                    }
                    OnSetCartKD_A(K, D, 2);
                    OnSetImpType_A(2);
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Cartesian impedance parameters size mismatch, using defaults");
                    double K[7] = {1800, 1800, 1800, 40, 40, 40, 20};
                    double D[7] = {0.6, 0.6, 0.6, 0.4, 0.4, 0.4, 0.4};
                    OnSetCartKD_A(K, D, 2);
                    OnSetImpType_A(2);
                }
            }
            OnSetSend();
            usleep(100000);
        }

        /// set maximum speed and acceleration
        OnClearSet();
        OnSetJointLmt_A(static_cast<int>(max_velocity_), static_cast<int>(max_acceleration_));
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
        if (robot_ctrl_mode_ == "POSITION")
        {
            OnClearSet();
            OnSetTargetState_B(1); //3:torque mode; 1:position mode
            OnSetSend();
            usleep(100000);
        }
        else
        {
            OnClearSet();
            OnSetTargetState_B(3); //3:torque mode; 1:position mode
            OnSetTool_B(kineParam, dynPara);
            if (robot_ctrl_mode_ == "JOINT_IMPEDANCE")
            {
                // 使用从硬件参数中读取的阻抗参数
                if (joint_imp_gain_.size() >= 7 && joint_imp_damp_.size() >= 7)
                {
                    double K[7];
                    double D[7];
                    for (size_t i = 0; i < 7; i++)
                    {
                        K[i] = joint_imp_gain_[i];
                        D[i] = joint_imp_damp_[i];
                    }
                    OnSetJointKD_B(K, D);
                    OnSetImpType_B(1);
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Joint impedance parameters size mismatch, using defaults");
                    double K[7] = {2, 2, 2, 1.6, 1, 1, 1};
                    double D[7] = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
                    OnSetJointKD_B(K, D);
                    OnSetImpType_B(1);
                }
            }
            else if (robot_ctrl_mode_ == "CART_IMPEDANCE")
            {
                // 使用从硬件参数中读取的阻抗参数
                if (cart_imp_gain_.size() >= 7 && cart_imp_damp_.size() >= 7)
                {
                    double K[7];
                    double D[7];
                    for (size_t i = 0; i < 7; i++)
                    {
                        K[i] = cart_imp_gain_[i];
                        D[i] = cart_imp_damp_[i];
                    }
                    OnSetCartKD_B(K, D, 2);
                    OnSetImpType_B(2);
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Cartesian impedance parameters size mismatch, using defaults");
                    double K[7] = {2500, 2500, 2500, 60, 60, 60, 20};
                    double D[7] = {0.6, 0.6, 0.6, 0.2, 0.2, 0.2, 0.4};
                    OnSetCartKD_B(K, D, 2);
                    OnSetImpType_B(2);
                }
            }
            OnSetSend();
            usleep(100000);
        }

        /// set maximum speed and acceleration
        OnClearSet();
        OnSetJointLmt_B(static_cast<int>(max_velocity_), static_cast<int>(max_acceleration_));
        OnSetSend();
        usleep(100000);
    }

    hardware_interface::CallbackReturn MarvinHardware::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Configuring Marvin Hardware Interface...");


        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up Marvin Hardware Interface...");

        if (hardware_connected_)
        {
            disconnectFromHardware();
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void MarvinHardware::contains_gripper()
    {
        int joint_index = 0;
        for (const auto& joint : info_.joints)
        {
            // 检查关节名称中是否包含 gripper 或 hand
            std::string joint_name_lower = joint.name;
            std::transform(joint_name_lower.begin(), joint_name_lower.end(),
                           joint_name_lower.begin(), ::tolower);

            if (joint_name_lower.find("gripper") != std::string::npos ||
                joint_name_lower.find("hand") != std::string::npos)
            {
                // 这是夹爪关节
                has_gripper_ = true;
                gripper_joint_name_.push_back(joint.name);
                gripper_joint_index_ = joint_index;
                RCLCPP_INFO(get_node()->get_logger(), "Detected gripper joint: %s (index %zu)",
                            joint.name.c_str(), gripper_joint_index_);
            }
            else
            {
                // 这是机械臂关节gripper
                joint_names_.push_back(joint.name);
            }
            joint_index++;
        }
        RCLCPP_INFO(get_node()->get_logger(), "Detected gripper joint: %zu ", gripper_joint_name_.size());
    }

    hardware_interface::CallbackReturn MarvinHardware::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Activating Marvin Hardware Interface...");
        simulation_mode_ = false;
        if (simulation_mode_)
        {
            RCLCPP_INFO(get_logger(), "Running in simulation mode");
            simulation_active_ = true;

            // Initialize simulation states
            for (size_t i = 0; i < hw_position_states_.size(); i++)
            {
                hw_position_states_[i] = 0.0;
                hw_velocity_states_[i] = 0.0;
                hw_effort_states_[i] = 0.0;
                hw_position_commands_[i] = 0.0;
                hw_velocity_commands_[i] = 0.0;
            }
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Running in real hardware mode");
            // Connect to real hardware
            if (!connectToHardware())
            {
                RCLCPP_ERROR(get_logger(), "Failed to connect to Marvin hardware");
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Read initial joint states from hardware
            if (!readFromHardware(true))
            {
                RCLCPP_ERROR(get_logger(), "Failed to read initial joint states from hardware");
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Initialize commands with current positions
            for (size_t i = 0; i < hw_position_commands_.size(); i++)
            {
                hw_position_commands_[i] = hw_position_states_[i];
                hw_velocity_commands_[i] = hw_velocity_states_[i];
                // RCLCPP_ERROR(get_logger(), "initia position ...  %f", hw_velocity_commands_[i]);
            }
        }
        OnClearSet();
        OnLogOff();
        OnLocalLogOff();
        OnSetSend();
        usleep(100000);


        if (robot_arm_index_ == ARM_LEFT)
        {
            setLeftArmCtrl();
        }
        else if (robot_arm_index_ == ARM_RIGHT)
        {
            setRightArmCtrl();
        }
        else if (robot_arm_index_ == ARM_DUAL)
        {
            setLeftArmCtrl();
            setRightArmCtrl();
        }

        OnGetBuf(&frame_data_);
        RCLCPP_INFO(get_logger(), "current state of A arm:%d\n",
                    frame_data_.m_State[0].m_CurState);
        RCLCPP_INFO(get_logger(), "cmd state of A arm:%d\n", frame_data_.m_State[0].m_CmdState);
        RCLCPP_INFO(get_logger(), "error code of A arms:%d\n",
                    frame_data_.m_State[0].m_ERRCode);
        RCLCPP_INFO(get_logger(), "cmd of vel and acc:%d %d\n",
                    frame_data_.m_In[0].m_Joint_Vel_Ratio, frame_data_.m_In[0].m_Joint_Acc_Ratio);

        // connect to gripper
        if (has_gripper_)
        {
            bool gripper_connected = connect_gripper();
            if (gripper_connected)
            {
                RCLCPP_INFO(get_logger(), "Gripper Connected");
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
                // RCLCPP_INFO(get_logger(), "gripper read position %d", cur_pos_status);
                // gripper_position_ = (9000 - cur_pos_status) / 9000.0;
                // gripper_position_ = 0.0;
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Gripper Not Connected");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        RCLCPP_INFO(get_logger(), "Marvin Hardware Interface activated successfully");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void MarvinHardware::gripper_callback()
    {
        while (hardware_connected_)
        {
            // OnClearSet();
            // if(!gripper_stopped_)
            // {
            //   int cur_pos_status = 0;
            //   int cur_speed_status = 0;
            //   int cur_effort_status = 0;
            //   bool success = ModbusGripper::getStatus(cur_effort_status, cur_speed_status, cur_pos_status, OnClearChDataA);
            //   RCLCPP_INFO(get_logger(), "gripper read position %d", cur_pos_status);
            //   gripper_position_ = (9000 - cur_pos_status) / 9000.0;
            //   RCLCPP_INFO(get_logger(), "gripper position %f", gripper_position_);
            //   if (gripper_position_ == last_gripper_position_)
            //   {
            //     gripper_stopped_ = true;
            //   }
            //   else
            //   {
            //     last_gripper_position_ = gripper_position_;
            //   }
            // }
            for (size_t i = 0; i < gripper_stopped_.size(); i++)
            {
                if (!gripper_stopped_[i])
                {
                    //RCLCPP_INFO(get_logger(), "gripper write position %f", step_size_);
                    gripper_position_[i] = gripper_position_[i] + step_size_[i];
                    //RCLCPP_INFO(get_logger(), "gripper write position %f", gripper_position_);
                    //RCLCPP_INFO(get_logger(), "gripper write position %f", last_gripper_command_);
                    // RCLCPP_INFO(get_logger(), "gripper write position %d", last_gripper_command_ == gripper_position_);
                    if (abs(gripper_position_[i] - last_gripper_command_[i]) < 0.001)
                    {
                        gripper_stopped_[i] = true;
                        step_size_[i] = 0.0;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            /// write when the gripper commannd is different
            for (size_t i = 0; i < gripper_stopped_.size(); i++)
            {
                if (last_gripper_command_[i] != gripper_position_command_[i])
                {
                    RCLCPP_INFO(get_logger(), "gripper %zu write position %f", i,
                                gripper_position_command_[i]);
                    // write commands to gripper
                    int cur_pos_set = 256 - 256 * gripper_position_command_[i];
                    int cur_speed_set = 100;
                    int cur_effort_set = 100;
                    RCLCPP_INFO(get_logger(), "gripper write position %d", cur_pos_set);
                    (void)gripper_ptr_[i]->move_gripper(cur_effort_set, cur_speed_set, cur_pos_set);
                    last_gripper_command_[i] = gripper_position_command_[i];
                    gripper_stopped_[i] = false;
                    step_size_[i] = (last_gripper_command_[i] - gripper_position_[i]) / 10.0;
                }
            }
            // OnSetSend();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    bool MarvinHardware::connect_gripper()
    {
        bool result = true;
        for (size_t i = 0; i < gripper_ptr_.size(); i++)
        {
            result = result && gripper_ptr_[i]->initialize();
        }
        return result;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Deactivating Marvin Hardware Interface...");

        simulation_active_ = false;
        OnClearSet();
        if (robot_arm_index_ == ARM_LEFT)
        {
            OnSetDragSpace_A(0);
            OnSetTargetState_A(0);
        }
        else if (robot_arm_index_ == ARM_RIGHT)
        {
            OnSetTargetState_B(0);
        }
        else if (robot_arm_index_ == ARM_DUAL)
        {
            OnSetTargetState_A(0);
            OnSetTargetState_B(0);
        }

        OnSetSend();
        usleep(100000);

        if (hardware_connected_)
        {
            disconnectFromHardware();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_shutdown(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Shutting down Marvin Hardware Interface...");

        if (hardware_connected_)
        {
            disconnectFromHardware();
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_error(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_ERROR(get_logger(), "Error in Marvin Hardware Interface");

        // Attempt to safely disconnect from hardware
        if (hardware_connected_)
        {
            disconnectFromHardware();
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> MarvinHardware::export_state_interfaces()
    {
        RCLCPP_DEBUG(get_logger(), "Exporting state interfacesExporting for %zu joints",
                     info_.joints.size());
        std::vector<hardware_interface::StateInterface> state_interfaces;
        size_t joint_name_sz = joint_names_.size();
        RCLCPP_INFO(get_logger(), "Exporting state interfaces for %zu joints", joint_name_sz);

        for (size_t i = 0; i < joint_name_sz; i++)
        {
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
            for (size_t i = 0; i < gripper_joint_name_.size(); i++)
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
        RCLCPP_DEBUG(get_logger(), "Exporting command interfaces for %zu joints",
                     info_.joints.size());

        std::vector<hardware_interface::CommandInterface> command_interfaces;
        size_t joint_name_sz = joint_names_.size();
        RCLCPP_INFO(get_logger(), "Exporting command  interfaces for %zu joints",
                    joint_name_sz);
        for (size_t i = 0; i < joint_name_sz; i++)
        {
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
            for (size_t i = 0; i < gripper_joint_name_.size(); i++)
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
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // if (simulation_active_) {
        //   simulateHardware(period);
        //   return hardware_interface::return_type::OK;
        // }

        if (!hardware_connected_)
        {
            // RCLCPP_ERROR_THROTTLE(
            //   get_logger(),
            //   *std::make_shared<rclcpp::Clock>(), 5000,
            //   "Not connected to hardware");
            return hardware_interface::return_type::ERROR;
        }

        if (!readFromHardware(false))
        {
            // RCLCPP_ERROR_THROTTLE(
            //   get_logger(),
            //   *std::make_shared<rclcpp::Clock>(), 5000,
            //   "Failed to read from hardware");
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MarvinHardware::write(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // if (simulation_active_) {
        //   // In simulation, commands are handled in the read method
        //   return hardware_interface::return_type::OK;
        // }

        if (!hardware_connected_)
        {
            return hardware_interface::return_type::ERROR;
        }
        /// convert rad to degree
        // RCLCPP_ERROR(get_logger(), "write command ...  %f", hw_position_commands_[2]);
        std::vector<double> hw_commands;
        for (size_t i = 0; i < hw_position_commands_.size(); i++)
        {
            hw_commands.push_back(radToDegree(hw_position_commands_[i]));
        }
        // RCLCPP_ERROR(get_logger(), "write command deg ...  %f", hw_position_commands_[2]);
        // Enforce joint limits before sending commands

        if (!writeToHardware(hw_commands))
        {
            // RCLCPP_ERROR_THROTTLE(
            //   get_logger(),
            //   *clock_, 5000,
            //   "Failed to write to hardware");
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    bool MarvinHardware::recv_thread_func()
    {
        long ch = 2;
        unsigned char data_buf[256] = {0};

        while (hardware_connected_)
        {
            if (robot_arm_index_ == ARM_LEFT)
            {
                (void)OnGetChDataA(data_buf, &ch);
                // if (size > 0 && ch == 2) {
                //     hex_to_str(data_buf, size, hex_str, sizeof(hex_str));
                //     printf("接收字节数: %d, 通道: %ld, HEX: %s\n", size, ch, hex_str);
                // }
            }
            else if (robot_arm_index_ == ARM_RIGHT)
            {
                (void)OnGetChDataB(data_buf, &ch);
            }
            else if (robot_arm_index_ == ARM_DUAL)
            {
                (void)OnGetChDataA(data_buf, &ch);
                (void)OnGetChDataB(data_buf, &ch);
            }
            usleep(200 * 1000);
        }
        return true;
    }

    bool MarvinHardware::connectToHardware()
    {
        RCLCPP_INFO(get_logger(), "Connecting to Marvin at %s:%d", device_ip_.c_str(), device_port_);

        // TODO: Implement actual Marvin connection using Marvin SDK
        // Example:
        // unsigned char octet1;
        // unsigned char octet2;
        // unsigned char octet3;
        // unsigned char octet4;

        // 解析 IP 地址
        std::vector<int> ip_parts;
        std::string ip_str = device_ip_;
        std::istringstream iss(ip_str);
        std::string token;
        while (std::getline(iss, token, '.'))
        {
            if (!token.empty())
            {
                ip_parts.push_back(std::stoi(token));
            }
        }
        
        if (ip_parts.size() != 4)
        {
            RCLCPP_ERROR(get_logger(), "Invalid IP address format: %s", device_ip_.c_str());
            return false;
        }
        
        // hardware_connected_ = OnLinkTo(octet1,octet2,octet3,octet4) == true;
        hardware_connected_ = OnLinkTo(ip_parts[0], ip_parts[1], ip_parts[2], ip_parts[3]);


        // Simulate connection for demonstration
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // hardware_connected_ = true;
        if (hardware_connected_)
        {
            RCLCPP_INFO(get_logger(), "Successfully connected to Marvin hardware");
            usleep(100000);
            OnClearSet();
            OnClearErr_A();
            OnClearErr_B();
            OnSetSend();
            usleep(100000);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Failed to connect to Marvin hardware");
        }

        return hardware_connected_;
    }

    void MarvinHardware::disconnectFromHardware()
    {
        RCLCPP_INFO(get_logger(), "Disconnecting from Marvin hardware");

        // TODO: Implement actual Marvin disconnection
        OnRelease();

        hardware_connected_ = false;
        RCLCPP_INFO(get_logger(), "Disconnected from Marvin hardware");
    }

    bool MarvinHardware::readFromHardware(bool initial_frame)
    {
        OnGetBuf(&frame_data_);
        // RCLCPP_INFO(get_logger(), "Reading from hardware interface ... %d", frame_data_.m_Out[robot_arm_index_].m_OutFrameSerial);
        if (initial_frame)
        {
            previous_message_frame_ = frame_data_.m_Out[robot_arm_index_].m_OutFrameSerial;
        }
        if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_RIGHT)
        {
            for (size_t i = 0; i < 7; i++)
            {
                hw_position_states_[i] = degreeToRad(frame_data_.m_Out[robot_arm_index_].m_FB_Joint_Pos[i]);
                hw_velocity_states_[i] = frame_data_.m_Out[robot_arm_index_].m_FB_Joint_Vel[i];
                hw_effort_states_[i] = frame_data_.m_Out[robot_arm_index_].m_FB_Joint_SToq[i];
            }
        }
        else if (robot_arm_index_ == ARM_DUAL)
        {
            for (size_t i = 0; i < 7; i++)
            {
                hw_position_states_[i] = degreeToRad(frame_data_.m_Out[ARM_LEFT].m_FB_Joint_Pos[i]);
                hw_velocity_states_[i] = frame_data_.m_Out[ARM_LEFT].m_FB_Joint_Vel[i];
                hw_effort_states_[i] = frame_data_.m_Out[ARM_LEFT].m_FB_Joint_SToq[i];
            }

            for (size_t i = 0; i < 7; i++)
            {
                hw_position_states_[i + 7] = degreeToRad(frame_data_.m_Out[ARM_RIGHT].m_FB_Joint_Pos[i]);
                hw_velocity_states_[i + 7] = frame_data_.m_Out[ARM_RIGHT].m_FB_Joint_Vel[i];
                hw_effort_states_[i + 7] = frame_data_.m_Out[ARM_RIGHT].m_FB_Joint_SToq[i];
            }
        }
        previous_message_frame_ = frame_data_.m_Out[robot_arm_index_].m_OutFrameSerial;

        if (previous_message_frame_ - frame_data_.m_Out[robot_arm_index_].m_OutFrameSerial < 2)
        {
            return true;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Missing more than 2 frames");
            return true;
        }
    }

    bool MarvinHardware::writeToHardware(std::vector<double>& hw_commands)
    {
        // TODO: Implement actual joint command sending to Marvin
        bool result = true;
        OnClearSet();
        if (robot_arm_index_ == ARM_LEFT)
        {
            result = OnSetJointCmdPos_A(hw_commands.data());
        }
        else if (robot_arm_index_ == ARM_RIGHT)
        {
            result = OnSetJointCmdPos_B(hw_commands.data());
        }
        else if (robot_arm_index_ == ARM_DUAL)
        {
            result = OnSetJointCmdPos_A(hw_commands.data());
            // RCLCPP_INFO(get_logger(), "write to left arm %d", result);
            result = result && OnSetJointCmdPos_B(hw_commands.data() + 7);
            // RCLCPP_INFO(get_logger(), "write to right arm %d", result);
        }
        OnSetSend();
        return result;
    }

    void MarvinHardware::simulateHardware(const rclcpp::Duration& period)
    {
        // Simple simulation: move toward commanded position with velocity limits
        for (size_t i = 0; i < hw_position_states_.size(); i++)
        {
            double position_error = hw_position_commands_[i] - hw_position_states_[i];
            double max_velocity_change = velocity_limits_[i] * period.seconds();

            // Calculate desired velocity
            double desired_velocity = std::copysign(
                std::min(std::abs(position_error) / period.seconds(), velocity_limits_[i]),
                position_error
            );

            // Apply velocity limits
            double velocity_change = desired_velocity - hw_velocity_states_[i];
            if (std::abs(velocity_change) > max_velocity_change)
            {
                velocity_change = std::copysign(max_velocity_change, velocity_change);
            }

            hw_velocity_states_[i] += velocity_change;
            hw_position_states_[i] += hw_velocity_states_[i] * period.seconds();

            // Simulate effort based on acceleration
            hw_effort_states_[i] = velocity_change / period.seconds() * 0.1;
        }
    }

    bool MarvinHardware::initializeJointLimits()
    {
        // position_lower_limits_.resize(info_.joints.size());
        // position_upper_limits_.resize(info_.joints.size());
        // velocity_limits_.resize(info_.joints.size());
        // effort_limits_.resize(info_.joints.size());

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            const auto& joint = info_.joints[i];

            // Parse position limits
            for (const auto& command_interface : joint.command_interfaces)
            {
                if (command_interface.name == hardware_interface::HW_IF_POSITION)
                {
                    if (!command_interface.min.empty())
                    {
                        position_lower_limits_[i] = std::stod(command_interface.min);
                    }
                    if (!command_interface.max.empty())
                    {
                        position_upper_limits_[i] = std::stod(command_interface.max);
                    }
                }
                else if (command_interface.name == hardware_interface::HW_IF_VELOCITY)
                {
                    if (!command_interface.max.empty())
                    {
                        velocity_limits_[i] = std::stod(command_interface.max);
                    }
                }
            }

            // Parse effort limits from joint limits in URDF
            effort_limits_[i] = std::stod(joint.command_interfaces[0].max);

            RCLCPP_DEBUG(
                get_logger(),
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
        for (size_t i = 0; i < hw_position_states_.size(); i++)
        {
            ss << info_.joints[i].name << ": pos=" << std::fixed << std::setprecision(3) << hw_position_states_[i]
                << ", vel=" << hw_velocity_states_[i] << ", cmd=" << hw_position_commands_[i];
            if (i < hw_position_states_.size() - 1) ss << " | ";
        }
        RCLCPP_DEBUG(get_logger(), "%s", ss.str().c_str());
    }
} // namespace marvin_ros2_control


PLUGINLIB_EXPORT_CLASS(marvin_ros2_control::MarvinHardware, hardware_interface::SystemInterface)
