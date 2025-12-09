#ifndef MODBUS_IO_H
#define MODBUS_IO_H

#include <vector>
#include <cstdint>
#include <string>
#include <functional>
#include <memory>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

namespace marvin_ros2_control
{
    // Standard Modbus function codes (for reference)
    namespace modbus_functions {
        static constexpr uint8_t READ_HOLDING_REGISTERS = 0x03;
        static constexpr uint8_t READ_INPUT_REGISTERS = 0x04;
        static constexpr uint8_t WRITE_SINGLE_REGISTER = 0x06;
        static constexpr uint8_t WRITE_MULTIPLE_REGISTERS = 0x10;
    }

    // Modbus function types
    using Clear485Func = bool (*)();
    using Send485Func = bool (*)(uint8_t data[256], long size, long channel);

    class ModbusIO {
    public:
        // Logger initialization
        static void initLogger(const rclcpp::Logger& logger) { logger_ = logger; }
        
        // Generic Modbus operations
        static std::vector<uint16_t> readRegisters(uint8_t slave_id, uint16_t start_addr, 
                                                  uint16_t count, uint8_t function_code,
                                                  Clear485Func clear_485, Send485Func send_485);
        
        static bool writeSingleRegister(uint8_t slave_id, uint16_t register_addr, uint16_t value,
                                       uint8_t function_code,
                                       Clear485Func clear_485, Send485Func send_485);
        
        static bool writeMultipleRegisters(uint8_t slave_id, uint16_t start_addr, 
                                          const std::vector<uint16_t>& values,
                                          uint8_t function_code,
                                          Clear485Func clear_485, Send485Func send_485);
        
        // Response handling
        static std::vector<uint8_t> receiveResponse(int max_attempts = 20, int timeout_ms = 50);
        
    private:
        // Core communication
        static bool sendRequest(const std::vector<uint8_t>& request, 
                               Clear485Func clear_485, Send485Func send_485);
        
        // Request building
        static std::vector<uint8_t> buildRequest(uint8_t slave_id, uint8_t function_code, 
                                                const std::vector<uint8_t>& data);
        
        // CRC calculation
        static uint16_t calculateCRC(const uint8_t* data, size_t length);
        
        // Static logger instance
        static rclcpp::Logger logger_;
        
        // Constants
        static constexpr uint8_t COM1_CHANNEL = 2;
        static constexpr size_t MAX_MODBUS_REGISTERS = 125;
        static constexpr size_t MAX_BUFFER_SIZE = 256;
    };

    // ==============================================
    // Abstract Base Gripper Class
    // ==============================================

    class ModbusGripper {
    public:
        virtual ~ModbusGripper() = default;

        // Logger initialization (static)
        static void initLogger(const rclcpp::Logger& logger) { 
            logger_ = logger;
            ModbusIO::initLogger(logger);
        }
        
        // Pure virtual functions that must be implemented by derived classes
        virtual bool initialize(Clear485Func clear_485, Send485Func send_485) = 0;
        virtual bool move_gripper(int torque, int velocity, int position,
                         Clear485Func clear_485, Send485Func send_485) = 0;
        virtual bool getStatus(int& torque, int& velocity, int& position,
                              Clear485Func clear_485, Send485Func send_485) = 0;
        virtual void deinitialize(){};
        
        // Common utility functions
        virtual void resetState() { /* Can be overridden if needed */ }
        
    protected:
        // Protected helper methods for derived classes
        bool writeSingleRegister(uint8_t slave_id, uint16_t register_addr, uint16_t value,
                                uint8_t function_code,
                                Clear485Func clear_485, Send485Func send_485) {
            return ModbusIO::writeSingleRegister(slave_id, register_addr, value, 
                                                function_code, clear_485, send_485);
        }
        
        bool writeMultipleRegisters(uint8_t slave_id, uint16_t start_addr, 
                                   const std::vector<uint16_t>& values,
                                   uint8_t function_code,
                                   Clear485Func clear_485, Send485Func send_485) {
            return ModbusIO::writeMultipleRegisters(slave_id, start_addr, values,
                                                   function_code, clear_485, send_485);
        }
        
        std::vector<uint16_t> readRegisters(uint8_t slave_id, uint16_t start_addr, uint16_t count,
                                           uint8_t function_code,
                                           Clear485Func clear_485, Send485Func send_485) {
            return ModbusIO::readRegisters(slave_id, start_addr, count,
                                          function_code, clear_485, send_485);
        }
        
        // Static logger accessible by derived classes
        static rclcpp::Logger logger_;
    };

    // ==============================================
    // Specific Gripper Implementations (Forward Declarations)
    // ==============================================

    class ZXGripper : public ModbusGripper {
    public:
        ZXGripper();
        bool initialize(Clear485Func clear_485, Send485Func send_485) override;
        bool move_gripper(int torque, int velocity, int position,
                 Clear485Func clear_485, Send485Func send_485) override;
        bool getStatus(int& torque, int& velocity, int& position,
                      Clear485Func clear_485, Send485Func send_485) override;
        void deinitialize() override;
        void resetState() override;
       
    private:
        
        bool acceleration_set_;
        bool deceleration_set_;
        
        // ZX-specific Modbus configuration
        static constexpr uint8_t SLAVE_ID = 0x01;
        static constexpr uint8_t READ_FUNCTION = 0x03;      // Read Holding Registers
        static constexpr uint8_t WRITE_SINGLE_FUNCTION = 0x06;   // Write Single Register
        static constexpr uint8_t WRITE_MULTIPLE_FUNCTION = 0x10; // Write Multiple Registers
        
        static constexpr uint16_t INIT_REGISTER = 0x0100;
        static constexpr uint16_t INIT_VALUE = 0x0001;
        static constexpr uint16_t POSITION_REG_LOW = 0x0102;
        static constexpr uint16_t POSITION_REG_HIGH = 0x0103;
        static constexpr uint16_t VELOCITY_REG = 0x0104;
        static constexpr uint16_t TORQUE_REG = 0x0105;
        static constexpr uint16_t ACCELERATION_REG = 0x0106;
        static constexpr uint16_t DECELERATION_REG = 0x0107;
        static constexpr uint16_t TRIGGER_REG = 0x0108;
        static constexpr uint16_t STATUS_REG = 0x060D;
        
        static constexpr uint16_t DEFAULT_ACCELERATION = 2000;
        static constexpr uint16_t DEFAULT_DECELERATION = 2000;
        static constexpr uint16_t TRIGGER_VALUE = 0x0001;
    };

    // ==============================================
    // GH Gripper Implementations (Forward Declarations)
    // ==============================================
    class JDGripper : public ModbusGripper {
    public:
        JDGripper();
        bool initialize(Clear485Func clear_485, Send485Func send_485) override;
        bool move_gripper(int torque, int velocity, int position,
                 Clear485Func clear_485, Send485Func send_485) override;
        bool getStatus(int& torque, int& velocity, int& position,
                      Clear485Func clear_485, Send485Func send_485) override;

 
    private:
        // GH-specific Modbus configuration
        static constexpr uint8_t SLAVE_ID = 0x09;
        static constexpr uint8_t WRITE_SINGLE_FUNCTION = 0x06;   // Write Single Register
        static constexpr uint8_t WRITE_MULTIPLE_FUNCTION = 0x10; // Write Multiple Registers
        /// initialize gripper
        static constexpr uint16_t INIT_REGISTER = 0x03E8;
        static constexpr uint16_t INIT_VALUE = 0x0000;
        static constexpr uint16_t INIT_REGISTER_NUM = 0x0002;

        /// write position
        static constexpr uint16_t POSITION_REG = 0x03E8;
        
        /// read position
        static constexpr uint8_t READ_FUNCTION = 0x04;      // Read Holding Registers
        static constexpr uint16_t STATUS_REG = 0x07D0;
        static constexpr uint16_t READ_REG_NUM = 0x0003;
    };
}

#endif