#include "gripper_control.h"
#include "MarvinSDK.h"
#include <iostream>
#include <thread>
#include <chrono>
namespace tj2_ros2_control
{
    std::vector<uint16_t> ModbusIO::readHoldingRegisters(uint8_t slaveId, uint16_t startAddr, uint16_t count) {
        if (count > 125) count = 125;
        
        std::vector<uint8_t> data = {
            (uint8_t)(startAddr >> 8), (uint8_t)(startAddr & 0xFF),
            (uint8_t)(count >> 8), (uint8_t)(count & 0xFF)
        };
        
        auto request = buildRequest(slaveId, 0x03, data);
        if (!sendRequest(request)) return {};
        
        auto response = receiveResponse();
        if (response.size() < 5) return {};
        
        // 解析响应
        std::vector<uint16_t> result;
        uint8_t byteCount = response[2];
        for (int i = 0; i < byteCount; i += 2) {
            if (i + 3 < response.size()) {
                uint16_t value = (response[i + 3] << 8) | response[i + 4];
                result.push_back(value);
            }
        }
        
        return result;
    }

    std::vector<uint16_t> ModbusIO::readInputRegisters(uint8_t slaveId, uint16_t startAddr, uint16_t count) {
        if (count > 125) count = 125;
        
        std::vector<uint8_t> data = {
            (uint8_t)(startAddr >> 8), (uint8_t)(startAddr & 0xFF),
            (uint8_t)(count >> 8), (uint8_t)(count & 0xFF)
        };
        
        auto request = buildRequest(slaveId, 0x04, data);
        if (!sendRequest(request)) return {};
        
        auto response = receiveResponse();
        if (response.size() < 5) return {};
        
        std::vector<uint16_t> result;
        uint8_t byteCount = response[2];
        for (int i = 0; i < byteCount; i += 2) {
            if (i + 3 < response.size()) {
                uint16_t value = (response[i + 3] << 8) | response[i + 4];
                result.push_back(value);
            }
        }
        
        return result;
    }

    bool ModbusIO::writeSingleRegister(uint8_t slaveId, uint16_t registerAddr, uint16_t value) {
        std::vector<uint8_t> data = {
            (uint8_t)(registerAddr >> 8), (uint8_t)(registerAddr & 0xFF),
            (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)
        };
        
        auto request = buildRequest(slaveId, 0x06, data);
        if (!sendRequest(request)) return false;
        
        auto response = receiveResponse();
        return response.size() >= 8;
    }

    bool ModbusIO::writeMultipleRegisters(uint8_t slaveId, uint16_t startAddr, const std::vector<uint16_t>& values, vector<uint8_t>& response) {
        if (values.empty() || values.size() receiveResponse> 123) return false;
        
        std::vector<uint8_t> data = {
            (uint8_t)(startAddr >> 8), (uint8_t)(startAddr & 0xFF),
            (uint8_t)(values.size() >> 8), (uint8_t)(values.size() & 0xFF),
            (uint8_t)(values.size() * 2)
        };
        
        for (uint16_t value : values) {
            data.push_back((uint8_t)(value >> 8));
            data.push_back((uint8_t)(value & 0xFF));
        }
        
        auto request = buildRequest(slaveId, 0x10, data);
        if (!sendRequest(request)) return false;
        
        response = receiveResponse();
        return response.size() >= 8;
    }

    bool ModbusIO::sendRequest(const std::vector<uint8_t>& request) {
        return OnSetChDataA(request.data(), request.size(), 2); // COM1
    }

    std::vector<uint8_t> ModbusIO::receiveResponse() {
        unsigned char buffer[256];
        long channel = 2; // COM1
        
        // 等待响应
        for (int i = 0; i < 10; i++) {
            int received = OnGetChDataA(buffer, &channel);
            if (received > 0) {
                return std::vector<uint8_t>(buffer, buffer + received);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        return {};
    }

    std::vector<uint8_t> ModbusIO::buildRequest(uint8_t slaveId, uint8_t functionCode, const std::vector<uint8_t>& data) {
        std::vector<uint8_t> request;
        request.push_back(slaveId);
        request.push_back(functionCode);
        request.insert(request.end(), data.begin(), data.end());
        
        uint16_t crc = calculateCRC(request.data(), request.size());
        request.push_back(crc & 0xFF);
        request.push_back(crc >> 8);
        
        return request;
    }

    uint16_t ModbusIO::calculateCRC(const uint8_t* data, size_t length) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc = crc >> 1;
                }
            }
        }
        return crc;
    }

    bool ZXGripper::ZXGripperInit()
    {
        // write 01 06 01 00 00 01 49 F6
        // return 01 06 01 00 00 01 49 F6
        uint8_t slave_id = 0x01;
        uint16_t slave_add = 0x0100;
        uint16_t  value = 0x0001;
        std::vector<uint8_t> response;
        bool success = ModbusIO::writeSingleRegister(slave_id, slave_add, value, response);
        if (success)
        {
            if(response[6] == 1)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    bool ZXGripper::ZXGripperMove(int& trq_set, int& vel_set, int& pos_set)
    {
        // 01 10 01 02 00 02 04 00 00 00 64 7E 0D 位置
        // 01 06 01 04 00 64 C8 1C 速度
        // 01 06 01 06 07 D0 6B 9B 加速度
        // 01 06 01 05 00 64 99 DC 力矩
        // 01 06 01 07 07 D0 3A 5B 减速
        // 01 06 01 08 00 01 C8 34 运动触发

        uint8_t slave_id = 0x01;
        uint16_t slave_add_pos = 0x0102;
        uint16_t slave_add_velo = 0x0104;
        uint16_t slave_add_torque = 0x0105;
        uint16_t slave_add_accel = 0x0106;
        uint16_t slave_add_deaccel = 0x0107;
        uint16_t slave_add_trigger = 0x0108;
        uint16_t velo= vel_set & 0xFF;
        uint16_t trq = trq_set & 0xFF;
        uint16_t acc = 2000 & 0xFF;
        uint16_t deacc = 2000 & 0xFF;
        uint16_t trigger = 0x0001;
        vector<uint16_t> pos_vec = {0x00, pos_set & 0xFF};
        
        vector<uint8_t> response;
        bool result = writeMultipleRegisters(slave_id, slave_add_pos, pos_vec, response);
        result = result && writeSingleRegister(slave_id, slave_add_velo, velo_set, response);
        result = result && writeSingleRegister(slave_id, slave_add_torque,trq_set, response);
        result = result && writeSingleRegister(slave_id, slave_add_accel,acc_set, response);
        result = result && writeSingleRegister(slave_id, slave_add_deaccel,deacc_set,response);
        result = result && writeSingleRegister(slave_id, slave_add_trigger,trigger_set,response);
        return result;
    }

    bool ZXGripper::ZXGripperStatus(int& trq_set, int& vel_set, int& pos_set)
    {
        // 01 03 06 09 00 02
        uint8_t slave_id = 0x01;
        uint16_t slave_add = 0x0609;
        uint16_t  value = 0x0002;
        std::vector<uint16_t> response;
        response = ModbusIO::readHoldingRegisters(slave_id, slave_add, value);
        //
        if (success)
        {
            pos_set = response[response.size() - 1];
        }
        return success;
    }
}