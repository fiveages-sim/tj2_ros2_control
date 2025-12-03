// AsyncDexterousHand.hpp
#ifndef ASYNC_DEXTEROUS_HAND_HPP
#define ASYNC_DEXTEROUS_HAND_HPP

#include <cstdint>
#include <vector>
#include <string>
#include <array>
#include <functional>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <queue>
#include <condition_variable>
#include <iostream>
#include <unistd.h>

extern "C" {
    bool OnSetChDataA(const unsigned char* data, long size, long channel);
    int OnGetChDataA(unsigned char* data, long* channel);
}

// ==================== 数据结构 ====================
struct CANFrame {
    uint32_t id;
    std::vector<uint8_t> data;
    std::chrono::system_clock::time_point timestamp;
    
    std::string toString() const {
        char hex_str[512];
        hex_to_str(data.data(), data.size(), hex_str, sizeof(hex_str));
        return std::string("ID:0x") + 
               std::to_string(id) + " Data:" + hex_str;
    }
};

struct JointState {
    uint8_t position = 0x80;
    uint8_t torque_limit = 0x80;
    uint8_t speed = 0x80;
    uint8_t acceleration = 0x80;
    uint8_t temperature = 0;
    uint8_t error_code = 0;
    
    std::chrono::system_clock::time_point last_update;
};

struct FingerSensor {
    uint8_t normal_pressure = 0;
    uint8_t tangential_pressure = 0;
    uint8_t tangential_direction = 255; // 255=未知
    uint8_t proximity = 0;
    
    std::chrono::system_clock::time_point last_update;
};

struct HandState {
    std::array<JointState, 7> joints;
    std::array<FingerSensor, 5> fingers;
    
    std::chrono::system_clock::time_point timestamp;
    
    std::string toString() const {
        std::string result = "=== Hand State ===\n";
        
        const char* joint_names[] = {"ThumbBend", "ThumbSwing", "Index", "Middle", 
                                     "Ring", "Little", "ThumbRotate"};
        for (int i = 0; i < 7; i++) {
            result += joint_names[i] + std::string(": Pos=0x") + 
                     std::to_string(static_cast<int>(joints[i].position)) + "\n";
        }
        
        return result;
    }
};

// ==================== 工具函数 ====================
void hex_to_str(const unsigned char* data, int size, char* output, int output_size);
int hex_string_to_bytes(const char* hex_str, unsigned char* bytes, int max_bytes);

// ==================== 主类 ====================
class AsyncDexterousHand {
public:
    enum HandSide {
        LEFT_HAND  = 0x28,
        RIGHT_HAND = 0x27
    };
    
    using StateCallback = std::function<void(const HandState& state)>;
    using FrameCallback = std::function<void(const CANFrame& frame)>;
    using ErrorCallback = std::function<void(const std::string& error)>;
    
    AsyncDexterousHand(HandSide side = RIGHT_HAND);
    ~AsyncDexterousHand();
    
    // 启动/停止
    bool start();
    void stop();
    bool isRunning() const;
    
    // 回调设置
    void setStateCallback(StateCallback callback);
    void setFrameCallback(FrameCallback callback);
    void setErrorCallback(ErrorCallback callback);
    
    // ===== 异步控制命令 =====
    // 所有发送命令都立即返回，不等待响应
    
    // 关节控制
    void setJointPositions(const std::array<uint8_t, 7>& positions);
    void setJointPosition(int joint_index, uint8_t position);
    
    void setTorqueLimits(const std::array<uint8_t, 7>& limits);
    void setJointSpeeds(const std::array<uint8_t, 7>& speeds);
    void setJointAccelerations(const std::array<uint8_t, 7>& accelerations);
    
    // 读取命令（触发设备发送数据）
    void requestFingerNormalPressure();    // 0x20
    void requestFingerTangentialPressure(); // 0x21
    void requestFingerTangentialDirection(); // 0x22
    void requestFingerProximity();         // 0x23
    
    void requestFingerAllData(int finger_index); // 0x28-0x32
    void requestJointTemperatures();       // 0x33
    void requestJointErrorCodes();         // 0x35
    void requestVersionInfo();             // 0x64
    
    // 高级控制
    void grasp(uint8_t position = 0x80);
    void release(uint8_t position = 0xFF);
    void pinchGrip(uint8_t thumb_pos = 0x40, uint8_t index_pos = 0x40);
    
    // 状态获取（线程安全）
    HandState getCurrentState() const;
    JointState getJointState(int index) const;
    FingerSensor getFingerSensor(int index) const;
    
    // 统计数据
    struct Statistics {
        uint64_t tx_frames = 0;
        uint64_t rx_frames = 0;
        uint64_t parse_errors = 0;
        uint64_t last_rx_timestamp = 0;
    };
    
    Statistics getStatistics() const;
    
private:
    HandSide hand_side_;
    uint32_t can_id_;
    
    // 线程控制
    std::atomic<bool> running_{false};
    std::thread receive_thread_;
    std::thread process_thread_;
    
    // 接收缓冲区
    std::queue<CANFrame> receive_queue_;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    
    // 状态存储
    HandState current_state_;
    mutable std::mutex state_mutex_;
    
    // 回调函数
    StateCallback state_callback_;
    FrameCallback frame_callback_;
    ErrorCallback error_callback_;
    
    // 统计
    mutable std::mutex stats_mutex_;
    Statistics stats_;
    
    // 私有方法
    void receiveThreadFunc();     // 持续接收数据
    void processThreadFunc();     // 处理接收到的数据
    void sendRawFrame(const std::vector<uint8_t>& data); // 发送原始数据
    
    // CAN帧编码/解码
    std::vector<uint8_t> encodeCANFrame(uint32_t can_id, const std::vector<uint8_t>& data);
    bool decodeCANFrame(const uint8_t* raw_data, int size, CANFrame& frame);
    
    // 状态更新
    void updateStateFromFrame(const CANFrame& frame);
    
    // 工具函数
    void safeCallStateCallback();
    void safeCallFrameCallback(const CANFrame& frame);
    void safeCallErrorCallback(const std::string& error);
};

#endif // ASYNC_DEXTEROUS_HAND_HPP