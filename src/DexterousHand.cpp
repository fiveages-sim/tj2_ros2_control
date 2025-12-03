// SimpleDexterousHand.cpp
#include "SimpleDexterousHand.hpp"

// ===== 工具函数实现 =====

std::vector<uint8_t> hexStringToBytes(const std::string& hex_str) {
    std::vector<uint8_t> bytes;
    std::stringstream ss(hex_str);
    std::string byte_str;
    
    while (ss >> byte_str) {
        try {
            bytes.push_back(static_cast<uint8_t>(std::stoul(byte_str, nullptr, 16)));
        } catch (...) {
            // 忽略转换错误
        }
    }
    
    return bytes;
}

std::string bytesToHexString(const uint8_t* data, size_t size) {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    
    for (size_t i = 0; i < size; i++) {
        oss << std::setw(2) << static_cast<int>(data[i]) << " ";
    }
    
    std::string result = oss.str();
    if (!result.empty()) {
        result.pop_back(); // 移除最后一个空格
    }
    
    return result;
}

// 您的函数实现
void hex_to_str(const unsigned char* data, int size, char* output, int output_size) {
    int pos = 0;
    for (int i = 0; i < size && pos < output_size - 3; i++) {
        sprintf(output + pos, "%02X ", data[i]);
        pos += 3;
    }
    if (pos > 0) {
        output[pos - 1] = '\0';
    } else {
        output[0] = '\0';
    }
}

int hex_string_to_bytes(const char* hex_str, unsigned char* bytes, int max_bytes) {
    int count = 0;
    char byte_str[3] = {0};
    const char* pos = hex_str;

    while (*pos && count < max_bytes) {
        // 跳过空格
        while (*pos == ' ') pos++;
        if (!*pos) break;

        // 提取两个字符作为一个字节
        byte_str[0] = *pos++;
        if (!*pos) break;
        byte_str[1] = *pos++;

        // 转换为字节
        bytes[count++] = (unsigned char)strtol(byte_str, NULL, 16);
    }

    return count;
}

// ===== SimpleDexterousHand 实现 =====

SimpleDexterousHand::SimpleDexterousHand(HandSide side) 
    : hand_side_(side), can_id_(static_cast<uint32_t>(side)) {
    // 初始化状态
    for (auto& joint : joint_states_) {
        joint = JointState();
    }
    for (auto& sensor : finger_sensors_) {
        sensor = FingerSensor();
    }
}

SimpleDexterousHand::~SimpleDexterousHand() {
    // 如果需要清理，可以调用 OnRelease()
}

std::vector<uint8_t> SimpleDexterousHand::encodeCANFrame(uint32_t can_id, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> encoded;
    
    // 根据您的示例，CAN ID 按小端序发送
    // "01 06 00 00 00 01 48 0A" 中 "01 06 00 00" 是 CAN ID 0x00000601
    
    // 添加CAN ID (4字节，小端序)
    encoded.push_back(static_cast<uint8_t>(can_id & 0xFF));
    encoded.push_back(static_cast<uint8_t>((can_id >> 8) & 0xFF));
    encoded.push_back(static_cast<uint8_t>((can_id >> 16) & 0xFF));
    encoded.push_back(static_cast<uint8_t>((can_id >> 24) & 0xFF));
    
    // 添加数据
    encoded.insert(encoded.end(), data.begin(), data.end());
    
    return encoded;
}

bool SimpleDexterousHand::decodeCANFrame(const uint8_t* raw_data, int size, CANFrame& frame) {
    if (size < 4) {
        return false;
    }
    
    // 解析CAN ID (小端序)
    frame.id = (static_cast<uint32_t>(raw_data[3]) << 24) |
               (static_cast<uint32_t>(raw_data[2]) << 16) |
               (static_cast<uint32_t>(raw_data[1]) << 8) |
               static_cast<uint32_t>(raw_data[0]);
    
    // 提取数据部分
    if (size > 4) {
        frame.data.assign(raw_data + 4, raw_data + size);
    } else {
        frame.data.clear();
    }
    
    frame.timestamp = std::chrono::system_clock::now();
    return true;
}

bool SimpleDexterousHand::sendCANFrame(uint32_t can_id, const std::vector<uint8_t>& data) {
    // 编码CAN帧
    auto encoded_data = encodeCANFrame(can_id, data);
    
    // 打印调试信息
    std::cout << "发送CAN帧: ID=0x" << std::hex << can_id 
              << ", 数据=" << bytesToHexString(data.data(), data.size())
              << std::dec << std::endl;
    
    // 使用您的API发送
    long channel = 1; // 根据您的代码，通道设为1
    bool result = OnSetChDataA(encoded_data.data(), encoded_data.size(), channel);
    
    if (!result) {
        std::cerr << "发送CAN帧失败" << std::endl;
    }
    
    return result;
}

bool SimpleDexterousHand::receiveCANFrame(CANFrame& frame, int timeout_ms) {
    unsigned char data_buf[256];
    long channel = 1;
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (true) {
        // 使用您的API接收
        int received_size = OnGetChDataA(data_buf, &channel);
        
        if (received_size > 0) {
            // 解码CAN帧waitForResponse
            if (decodeCANFrame(data_buf, received_size, frame)) {
                // 打印调试信息
                char hex_str[512];
                hex_to_str(data_buf, received_size, hex_str, sizeof(hex_str));
                std::cout << "接收CAN帧: " << hex_str << std::endl;
                
                // 触发回调
                if (frame_callback_) {
                    frame_callback_(frame);
                }
                
                return true;
            }
        }
        
        // 检查超时
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > timeout_ms) {
            break;
        }
        
        usleep(1000); // 睡眠1ms
    }
    
    return false;
}

bool SimpleDexterousHand::sendCommand(uint8_t cmd_code, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> frame_data;
    frame_data.push_back(cmd_code);
    frame_data.insert(frame_data.end(), data.begin(), data.end());
    
    // 根据协议，不同指令有不同的DLC要求
    switch (cmd_code) {
        case 0x01: // 关节位置
        case 0x02: // 转矩限制
        case 0x05: // 速度
        case 0x07: // 加速度
        case 0x38: // 校准
        case 0x39: // 四指校准
            // 需要8字节
            while (frame_data.size() < 8) {
                frame_data.push_back(0x00);
            }
            break;
        default:
            // 其他指令保持原样
            break;
    }
    
    return sendCANFrame(can_id_, frame_data);
}

void SimpleDexterousHand::updateStateFromResponse(uint8_t cmd_code, const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    switch (cmd_code) {
        case 0x01: // 关节位置
            for (int i = 0; i < 7 && (i + 1) < data.size(); i++) {
                joint_states_[i].position = data[i + 1];
            }
            break;
            
        case 0x02: // 转矩限制
            for (int i = 0; i < 7 && (i + 1) < data.size(); i++) {
                joint_states_[i].torque_limit = data[i + 1];
            }
            break;
            
        case 0x20: // 法向压力
            for (int i = 0; i < 5 && (i + 1) < data.size(); i++) {
                finger_sensors_[i].normal_pressure = data[i + 1];
            }
            break;
            
        case 0x21: // 切向压力
            for (int i = 0; i < 5 && (i + 1) < data.size(); i++) {
                finger_sensors_[i].tangential_pressure = data[i + 1];
            }
            break;
            
        case 0x22: // 切向方向
            for (int i = 0; i < 5 && (i + 1) < data.size(); i++) {
                finger_sensors_[i].tangential_direction = data[i + 1];
            }
            break;
            
        case 0x23: // 接近感应
            for (int i = 0; i < 5 && (i + 1) < data.size(); i++) {
                finger_sensors_[i].proximity = data[i + 1];
            }
            break;
            
        case 0x28: // 拇指所有数据
            if (data.size() >= 5) {
                finger_sensors_[0].normal_pressure = data[1];
                finger_sensors_[0].tangential_pressure = data[2];
                finger_sensors_[0].tangential_direction = data[3];
                finger_sensors_[0].proximity = data[4];
            }
            break;
            
        case 0x33: // 关节温度
            for (int i = 0; i < 7 && (i + 1) < data.size(); i++) {
                joint_states_[i].temperature = data[i + 1];
            }
            break;
            
        case 0x35: // 关节错误码
            for (int i = 0; i < 7 && (i + 1) < data.size(); i++) {
                joint_states_[i].error_code = data[i + 1];
            }
            break;
    }
    
    // 触发传感器回调
    if (sensor_callback_) {
        for (int i = 0; i < 5; i++) {
            sensor_callback_(i, finger_sensors_[i]);
        }
    }
}

// ===== 灵巧手控制命令 =====

bool SimpleDexterousHand::setJointPositions(const std::array<uint8_t, 7>& positions) {
    std::vector<uint8_t> data(positions.begin(), positions.end());
    return sendCommand(0x01, data);
}

bool SimpleDexterousHand::setJointPosition(int joint_index, uint8_t position) {
    if (joint_index < 0 || joint_index >= 7) return false;
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto positions = std::array<uint8_t, 7>();
    for (int i = 0; i < 7; i++) {
        positions[i] = joint_states_[i].position;
    }
    positions[joint_index] = position;
    
    return setJointPositions(positions);
}

bool SimpleDexterousHand::readFingerSensors(int finger_type) {
    if (finger_type < 0x20 || finger_type > 0x23) {
        return false;
    }
    return sendCommand(static_cast<uint8_t>(finger_type));
}

bool SimpleDexterousHand::readFingerAllData(int finger_index) {
    uint8_t cmd_code = 0;
    switch (finger_index) {
        case 0: cmd_code = 0x28; break; // 拇指
        case 1: cmd_code = 0x29; break; // 食指
        case 2: cmd_code = 0x30; break; // 中指
        case 3: cmd_code = 0x31; break; // 无名指
        case 4: cmd_code = 0x32; break; // 小指
        default: return false;
    }
    return sendCommand(cmd_code);
}

bool SimpleDexterousHand::readJointTemperatures() {
    return sendCommand(0x33);
}

bool SimpleDexterousHand::readJointErrorCodes() {
    return sendCommand(0x35);
}

bool SimpleDexterousHand::readVersionInfo() {
    return sendCommand(0x64);
}

/// ------------ used for testing------------------

bool SimpleDexterousHand::grasp(uint8_t position) {
    std::array<uint8_t, 7> positions = {
        position, 0x80, position, position, position, position, 0x80
    };
    return setJointPositions(positions);
}

bool SimpleDexterousHand::release(uint8_t position) {
    return grasp(0xFF); // 0xFF为伸直
}

bool SimpleDexterousHand::pinchGrip(uint8_t thumb_pos, uint8_t index_pos) {
    std::array<uint8_t, 7> positions = {
        thumb_pos, 0x40, index_pos, 0xFF, 0xFF, 0xFF, 0x80
    };
    return setJointPositions(positions);
}

// ===== 状态获取 =====

std::array<JointState, 7> SimpleDexterousHand::getJointStates() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return joint_states_;
}

std::array<FingerSensor, 5> SimpleDexterousHand::getFingerSensors() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return finger_sensors_;
}

void SimpleDexterousHand::setFrameCallback(FrameCallback callback) {
    frame_callback_ = std::move(callback);
}

void SimpleDexterousHand::setSensorCallback(SensorCallback callback) {
    sensor_callback_ = std::move(callback);
}

bool SimpleDexterousHand::waitForResponse(uint8_t cmd_code, int timeout_ms) {
    // 简单的轮询等待
    auto start = std::chrono::steady_clock::now();
    
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count() < timeout_ms) {
        
        CANFrame frame;
        if (receiveCANFrame(frame, 10)) { // 10ms超时
            if (!frame.data.empty() && frame.data[0] == cmd_code) {
                updateStateFromResponse(cmd_code, frame.data);
                return true;
            }
        }
    }
    
    return false;
}

std::string SimpleDexterousHand::getStatusString() const {
    std::ostringstream oss;
    
    oss << "=== 灵巧手状态 ===" << std::endl;
    oss << "手部: " << (hand_side_ == LEFT_HAND ? "左手" : "右手") << std::endl;
    oss << "CAN ID: 0x" << std::hex << can_id_ << std::dec << std::endl;
    
    oss << "\n关节状态:" << std::endl;
    const char* joint_names[] = {"拇指弯曲", "拇指横摆", "食指", "中指", 
                                 "无名指", "小指", "拇指旋转"};
    
    auto joints = getJointStates();
    for (int i = 0; i < 7; i++) {
        oss << "  " << joint_names[i] << ": "
            << "位置=0x" << std::hex << static_cast<int>(joints[i].position)
            << ", 温度=" << std::dec << static_cast<int>(joints[i].temperature)
            << "°C, 错误码=0x" << std::hex << static_cast<int>(joints[i].error_code)
            << std::dec << std::endl;
    }
    
    oss << "\n手指传感器:" << std::endl;
    const char* finger_names[] = {"拇指", "食指", "中指", "无名指", "小指"};
    
    auto sensors = getFingerSensors();
    for (int i = 0; i < 5; i++) {
        oss << "  " << finger_names[i] << ": "
            << "法向压力=" << static_cast<int>(sensors[i].normal_pressure)
            << ", 切向压力=" << static_cast<int>(sensors[i].tangential_pressure)
            << ", 接近感应=" << static_cast<int>(sensors[i].proximity)
            << std::endl;
    }
    
    return oss.str();
}