#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <cmath> // for cos and sin

#include <glob.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include "serial/serial.h"

//sudo apt-get install ros-humble-vision-msgs
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
//#include "robot_custom_msgs/msg/line_laser_data.hpp"
//#include "robot_custom_msgs/msg/line_laser_data_array.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

#define USE_DEBUG_LOG false

#define USE_LINELASER_SENSOR 0

enum class CommandNumber : uint8_t {
    ProtocolV0 = 0x10,               // Protocol V0
    ProtocolV1_Objects = 0x11,       // Protocol V1 (Objects)
    ProtocolV1_2LL = 0x12,           // Protocol V1 (2LL)
    ProtocolV2_Objects = 0x13,       // Protocol V2 (Objects)
    ProtocolV2_2LL = 0x14,              // Protocol V2 (2LL)
    
    Error = 0x20,                       // Error

    OTA_Start = 0x30,                   // OTA Start

    VersionRequest = 0x50,              // Version Sequest (Request)
    CalibrationRequest = 0x51,          // Clibration Start Request (Request)
    SensorControlRequest = 0x52,        // Sensor Control Request (Request)

    VersionResponse = 0x60,             // Version Response ( Response )
    CalibrationResponse  = 0x61,        // Clibration Start Response  (Response )
    SensorControlResponse = 0x62,       // Sensor Control Response (Request)
};

struct LagacyPacketData
{
    uint8_t pre_amble_0;       
    uint8_t pre_amble_1;       
    uint8_t data_size;         
    uint8_t header;            
    uint8_t number_of_objects;
    uint8_t distance;          // 거리
    uint8_t x_0;               // X 좌표
    uint8_t x_1;               // X 좌표
    uint8_t y_0;               // Y 좌표
    uint8_t y_1;               // Y 좌표
    uint8_t width_0;           // 폭
    uint8_t width_1;           // 폭
    uint8_t height_0;          // 높이
    uint8_t height_1;          // 높이 
    uint8_t checksum;          
};

struct ObjectData
{
    uint8_t class_id;
    int16_t x;
    int16_t y;
    int16_t theta;         // 각도 (Theta)
    int16_t width;         // 너비 (Width)
    int16_t height;        // 높이 (Height)
    uint16_t distance;     // 거리 (Distance)
};

struct ObjectDataV2
{
    uint8_t class_id;      // 객체의 클래스 ID
    uint8_t confidence;    // 신뢰도 (0~100)
    int16_t x;             // X 좌표
    int16_t y;             // Y 좌표
    int16_t theta;         // 각도 (Theta)
    int16_t width;         // 너비 (Width)
    int16_t height;        // 높이 (Height)
    uint16_t distance;     // 거리 (Distance)
};

struct ObjectPacketData
{
    uint8_t pre_amble_0;
    uint8_t pre_amble_1;
    uint8_t data_size;
    uint8_t header;
    uint8_t number_of_objects;
    std::vector<ObjectData> objects;
    uint8_t checksum;
};

struct ObjectV2PacketData
{
    uint8_t pre_amble_0;             // 0xAA
    uint8_t pre_amble_1;             // 0x55
    uint8_t data_size;               // Variable: Number of following data bytes
    uint8_t header;                  // 0x13 (Command header)
    uint8_t version_1;               // New: First Version field
    uint8_t version_2;               // New: Second Version field
    uint8_t number_of_objects;       // Variable: Number of objects
    std::vector<ObjectDataV2> objects; // Variable-length list of objects
    uint8_t checksum;                // Checksum for the packet
};

#if USE_LINELASER_SENSOR > 0
struct LLData
{
    int16_t x;
    int16_t y;
    int16_t theta;
    int16_t width;
    int16_t height;
    uint16_t distance;
};

struct LLDataV2
{
    int16_t x;
    int16_t y;
    int16_t theta;
    // int16_t width;
    int8_t direction; // Added field for Direction
    int8_t reserved;  // Added field for Reserved
    int16_t height;
    uint16_t distance;
};

struct LLPacketData
{
    uint8_t pre_amble_0;
    uint8_t pre_amble_1;
    uint8_t data_size;
    uint8_t header;
    uint8_t number_of_2LL;
    std::vector<LLData> LL;
    uint8_t checksum;
};

struct LLV2PacketData
{
    uint8_t pre_amble_0;
    uint8_t pre_amble_1;
    uint8_t data_size;
    uint8_t header;
    uint8_t version_1; // New: First Version field
    uint8_t version_2; // New: Second Version field
    uint8_t number_of_2LL;
    std::vector<LLDataV2> LL;
    uint8_t checksum;
};
#endif

struct ErrorSection {
    uint8_t occurred;        // Error 발생 여부 (1: 발생, 0: Clear)
    uint8_t rank;            // 특정 Error 로 고정
    uint8_t errorCode[3];    // 3바이트 문자열
};

// Define the main data packet struct
struct ErrorDataPacket{
    uint8_t PRE_AMBLE_0;    // 0xAA
    uint8_t PRE_AMBLE_1;    // 0x55
    uint8_t Data_Size;      // Total size of the packet
    uint8_t Header_Command; // 0x20
    uint8_t Number_of_Objects; // Variable number of objects (ErrorSection count)

    std::vector<ErrorSection> Errors; // Vector to store error sections

    uint8_t CHECKSUM;       // ADD(4~끝-1)
};

struct VersionResponsePacket {
    uint8_t PRE_AMBLE_0;  // 0xAA
    uint8_t PRE_AMBLE_1;  // 0x55
    uint8_t Data_Size;    // 4
    uint8_t Header_Command; // 0x60
    uint8_t Version1;     // Version part 1
    uint8_t Version2;     // Version part 2
    uint8_t CHECKSUM;     // ADD(4~6)
};

struct CalibrationResultPacket
{
    uint8_t PRE_AMBLE_0;
    uint8_t PRE_AMBLE_1;
    uint8_t Data_Size;
    uint8_t Header_Command;
    uint8_t Result;
    uint8_t CHECKSUM;
};

struct SensorControPacket
{
    uint8_t PRE_AMBLE_0;
    uint8_t PRE_AMBLE_1;
    uint8_t Data_Size;
    uint8_t Header_Command;
    uint8_t Result;
    uint8_t CHECKSUM;
};


double current_pose_x = 0.0;
double current_pose_y = 0.0;
double current_pose_theta = 0.0;

double amcl_pose_x = 0.0;
double amcl_pose_y = 0.0;
double amcl_pose_angle = 0.0;

std::string aiVersion = "0.0";

class AIInterface : public rclcpp::Node
{
public:
    AIInterface() : Node("airbot_ai_interface")
    {
        // 파라미터 설정 (USB 포트 및 보레이트)
        this->declare_parameter("port","/dev/ttyAI");
        this->declare_parameter("baudrate",921600);
        this->declare_parameter("use_cam",1);
        this->declare_parameter("use_2LL",1);

        this->get_parameter("port", serial_port_);
        this->get_parameter("baudrate", baudrate_);
        this->get_parameter("use_cam", use_cam_);
        this->get_parameter("use_2LL", use_2LL_);

        RCLCPP_INFO(this->get_logger(), "port : %s, baudrate: %d", serial_port_.c_str(), baudrate_);
        RCLCPP_INFO(this->get_logger(), "use_cam : %d, use_2LL: %d", use_cam_, use_2LL_);

        amcl_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10, std::bind(&AIInterface::amcl_pose_callback, this, std::placeholders::_1));
        
        //publisher
        camera_data_publisher_ = this->create_publisher<robot_custom_msgs::msg::CameraDataArray>("camera_data", 10);
        
        ai_version_publisher_ = this->create_publisher<std_msgs::msg::String>("ai_version", 1);

        #if USE_LINELASER_SENSOR > 0
        lineLaser_cmd_sub = this->create_subscription<std_msgs::msg::Bool>("/cmd_linelaser", 10, std::bind(&AIInterface::lineLaserCommand_callback, this, std::placeholders::_1));
        line_laser_data_publisher_ = this->create_publisher<robot_custom_msgs::msg::LineLaserDataArray>("line_laser_data", 10);
        #endif

        req_version_sub_ = this->create_subscription<std_msgs::msg::UInt8>("/req_version", 10, std::bind(&AIInterface::requestVersionCallback, this, std::placeholders::_1));
        cmd_camera_sub_ = this->create_subscription<std_msgs::msg::Bool>("/cmd_camera", 10, std::bind(&AIInterface::cameraOnOffCallback, this, std::placeholders::_1));

        // Set up the serial connection
        try
        {
            serial_.setPort(serial_port_);
            serial_.setBaudrate(baudrate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);
            serial_.open();
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port.");
        }

        if (serial_.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Serial port initialized.");

            // 주기적으로 데이터를 송수신하기 위한 타이머 설정 (발신 및 수신 모두 처리)
            timer_ = this->create_wall_timer(
            std::chrono::milliseconds(40),  // 100 ms로 나중에 조정 필요함.
            std::bind(&AIInterface::readSerialData, this));
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Serial port not opened.");
        }
    }

    ~AIInterface()
    {
        if (serial_.isOpen()) {
            serial_.close();
        }
    }

public:
    std::string serial_port_;
    int baudrate_;
    int use_cam_;
    int use_2LL_;

private:
    serial::Serial serial_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<uint8_t> buffer_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lineLaser_cmd_sub;
    rclcpp::Publisher<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_data_publisher_;
    #if USE_LINELASER_SENSOR > 0
    rclcpp::Publisher<robot_custom_msgs::msg::LineLaserDataArray>::SharedPtr line_laser_data_publisher_;
    #endif
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ai_version_publisher_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr req_version_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_camera_sub_;

    std::pair<double, double> convertSensorPosition(double sensor_offset_x, double sensor_offset_y, double obstacle_distance, double sensor_orientation)
    {
        double obstacle_x_in_robot_frame = (obstacle_distance+sensor_offset_x) * cos(sensor_orientation);
        double obstacle_y_in_robot_frame = (obstacle_distance+sensor_offset_y) * sin(sensor_orientation);

        return std::make_pair(obstacle_x_in_robot_frame, obstacle_y_in_robot_frame);
    }

    void requestVersionCallback(const std_msgs::msg::UInt8::SharedPtr msg)
    {   
        publishVersion();
        sendVersionRequest();
    }

    void cameraOnOffCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {   
        bool on = msg->data;
        RCLCPP_INFO(this->get_logger(), "cameraOnOffCallback ");
        if(on){
            sendSensorControl(true);
            RCLCPP_INFO(this->get_logger(), "cameraOn ");
        }else{
            sendSensorControl(false);
            RCLCPP_INFO(this->get_logger(), "cameraOff ");
        }
    }

    void calibrationCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {   
        bool calib = msg->data;
        if(calib){
            sendCalibrationRequest(true);
        }else{
            sendCalibrationRequest(false);
        }
    }

    double quaternion_to_euler(const geometry_msgs::msg::Quaternion& quat)
    {
        tf2::Quaternion q;
        tf2::fromMsg(quat, q);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        return yaw; // Return yaw as theta
    }

    void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        const auto& pose = msg->pose.pose;
        amcl_pose_x = pose.position.x;
        amcl_pose_y = pose.position.y;
        amcl_pose_angle = quaternion_to_euler(pose.orientation);

        // RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f, Theta: %f", current_pose_x, current_pose_y, current_pose_theta);
        
    }

    #if USE_LINELASER_SENSOR > 0
    void lineLaserCommand_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool bLineLaserOnOff = msg->data;
        if(bLineLaserOnOff){
         lineLaserSensorOn();
        }else{
         lineLaserSensorOff();
        }
    }
    #endif

    // 발신 전용 함수: 특정 데이터를 발신할 때 호출 (필요시 추가 사용 가능)
    // AP->AI로 보내는 경우가 있을 경우를 대비해서
    void sendData(const std::string& data)
    {
        try
        {
            if (serial_.isOpen())
            {
                serial_.write(data);
                #if USE_DEBUG_LOG
                RCLCPP_INFO(this->get_logger(), "Custom data sent: %s", data.c_str());
                #endif
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Serial port not open for sending data.");
            }
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error sending data: %s", e.what());
        }
    }

    void sendVersionRequest()
    {
        // Create a vector for the data
        std::vector<uint8_t> data(5); // Size is 5 based on your structure

        // Fill in the packet fields
        data[0] = 0xAA; // PRE_AMBLE_0
        data[1] = 0x55; // PRE_AMBLE_1
        data[2] = 2;    // Data_Size
        data[3] = static_cast<uint8_t>(CommandNumber::VersionRequest); // Header (Command)

        // Calculate checksum: sum of Data_Size and Header (Command)
        data[4] = data[3]; // CHECKSUM is just ADD(4) -> Header value

        // Send the data
        if (serial_.isOpen())
            serial_.write(data);
    }

    void sendCalibrationRequest(bool start)
    {
        // Create a vector for the data
        std::vector<uint8_t> data(6); // Size is 6 based on your structure

        // Fill in the packet fields
        data[0] = 0xAA;        // PRE_AMBLE_0
        data[1] = 0x55;        // PRE_AMBLE_1
        data[2] = 3;           // Data_Size
        data[3] = static_cast<uint8_t>(CommandNumber::CalibrationRequest);
        data[4] = start ? 1 : 0; // Start/Stop (1 for Start, 0 for Stop)

        // Calculate checksum: ADD(4, 5) = Header + Start/Stop
        data[5] = data[3] + data[4]; // CHECKSUM

        // Send the data
        serial_.write(data);
    }

    void sendSensorControl(bool enable)
    {
        // Create a vector for the data
        std::vector<uint8_t> data(6); // Size is 6 based on your structure

        // Fill in the packet fields
        data[0] = 0xAA;        // PRE_AMBLE_0
        data[1] = 0x55;        // PRE_AMBLE_1
        data[2] = 3;           // Data_Size
        data[3] = static_cast<uint8_t>(CommandNumber::SensorControlRequest);
        data[4] = enable ? 1 : 0; // on/off (1 for on, 0 for off)
        data[5] = data[3] + data[4]; // CHECKSUM

        // Send the data
        serial_.write(data);
    }

    // 수신 전용 함수: 특정 데이터를 수신할 때 호출 (필요시 추가 사용 가능)
    // AI-> AP 쪽으로 카메라 장애물 인지 정보
    void readSerialData()
    {
        const int num_bytes_to_read = 1024;
        std::vector<uint8_t> new_data(num_bytes_to_read);

        try
        {   
            //RCLCPP_INFO(this->get_logger(), "available");
            size_t available_bytes = serial_.available();

            if (available_bytes > 0)
            {
                // colcon build
                // source install/setup.bash
                // ros2 launch airbot_ai_interface airbot_ai_interface_launch.py

                size_t bytes_to_read = std::min(available_bytes, static_cast<size_t>(num_bytes_to_read));
                ssize_t bytes_read = serial_.read(new_data.data(), bytes_to_read);
                //RCLCPP_INFO(this->get_logger(), "bytes_read : %d",bytes_read);
                
                if (bytes_read > 0)
                {
                    // 로그 출력 (hex 값)
                    // RCLCPP_INFO(this->get_logger(), "Received data (hex):");
                    // std::stringstream ss;
                    // for (int i = 0; i < bytes_read; ++i)
                    // {
                    //     ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(new_data[i]) << ' ';
                    // }

                    // RCLCPP_WARN로 hex 데이터를 출력
                    // RCLCPP_INFO(this->get_logger(), "Received data (hex): %s", ss.str().c_str());

                    new_data.resize(bytes_read);
                    appendData(new_data);
                    searchAndParseData();
                }
            }
            else
            {
                // if( aiVersion != "2.0"){ 
                //     aiVersion = "2.0";

                //     std_msgs::msg::String msg;
                //     msg.data = aiVersion;  // unsigned short -> uint8_t로 캐스팅

                //     ai_version_publisher_->publish(msg);
                // }
                // RCLCPP_WARN(this->get_logger(), "No data available to read.");
            }
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error reading data: %s", e.what());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "An error occurred while reading data: %s", e.what());
        }
    }

    //파일 전송 - 1024KB
    uint8_t calculateChecksum(const std::vector<char>& data)
    {
        uint8_t checksum = 0;
        for(const auto &byte : data)
        {
            checksum += static_cast<uint8_t>(byte); //바이트 값 더함
        }
        return checksum;
    }

    void appendData(const std::vector<uint8_t> &new_data)
    {
        buffer_.insert(buffer_.end(), new_data.begin(), new_data.end());
    }

    void searchAndParseData()
    {
        const std::array<uint8_t, 2> preamble = {0xAA, 0x55};

        while (buffer_.size() >= 5)
        {
            auto preamble_position = std::search(buffer_.begin(), buffer_.end(), preamble.begin(), preamble.end());
            if (preamble_position != buffer_.end())
            {
                size_t preamble_index = std::distance(buffer_.begin(), preamble_position);

                if (buffer_.size() - preamble_index < 5)
                {
                    //std::cerr << "Data size too small to contain all fields, waiting for more data." << std::endl;
                    return;
                }

                uint8_t data_size = buffer_[preamble_index + 2];
                if (buffer_.size() - preamble_index < data_size + 3)
                {
                    //std::cerr << "Data size does not match the expected size, waiting for more data." << std::endl;
                    return;
                }

                uint8_t checksum = 0;
                for (size_t i = preamble_index + 3; i < preamble_index + 3 + data_size - 1; ++i)
                {
                    checksum += buffer_[i];
                }

                if (checksum == buffer_[preamble_index + 3 + data_size - 1])
                {
                    // std::cerr << "Checksum success" << std::endl;
                    parseDataFields(std::vector<uint8_t>(buffer_.begin() + preamble_index, buffer_.begin() + preamble_index + 3 + data_size));
                    buffer_.erase(buffer_.begin(), buffer_.begin() + preamble_index + 3 + data_size);
                }
                else
                {
                    //std::cerr << "Checksum mismatch" << std::endl;
                    buffer_.erase(buffer_.begin(), buffer_.begin() + preamble_index + 3 + data_size);
                }
            }
            else
            {
                //std::cerr << "Preamble not found, removing data until next byte" << std::endl;
                buffer_.erase(buffer_.begin(), buffer_.begin() + 1);
            }
        }
    }

    bool objectParsePacket(const std::vector<uint8_t> &data, ObjectPacketData &packet)
    {
        packet.pre_amble_0 = data[0];
        packet.pre_amble_1 = data[1];
        packet.data_size = data[2];
        packet.header = data[3];
        packet.number_of_objects = data[4];

        size_t offset = 5;
        // RCLCPP_INFO(this->get_logger(),"objectParsePacket size : %u, data size : ", packet.number_of_objects,packet.data_size);
        for (uint8_t i = 0; i < packet.number_of_objects; ++i)
        {
            if (offset + 11 > data.size())
                return false;

            ObjectData obj;
            obj.class_id = data[offset];
            obj.x = static_cast<int16_t>((data[offset + 2] << 8) | data[offset + 1]);
            obj.y = static_cast<int16_t>((data[offset + 4] << 8) | data[offset + 3]);
            obj.theta = static_cast<int16_t>((data[offset + 6] << 8) | data[offset + 5]);
            obj.width = static_cast<int16_t>((data[offset + 8] << 8) | data[offset + 7]);
            obj.height = static_cast<int16_t>((data[offset + 10] << 8) | data[offset + 9]);
            obj.distance = static_cast<uint16_t>((data[offset + 12] << 8) | data[offset + 11]);
            // RCLCPP_INFO(this->get_logger(),"obj.class_id %u, idx : %u",obj.class_id,i);

            packet.objects.push_back(obj);
            offset += 13;
        }

        if (offset >= data.size())
            return false;

        packet.checksum = data[offset];

        uint8_t computed_checksum = 0;
        for (size_t i = 3; i < offset; ++i)
        {
            computed_checksum += data[i];
        }

        return (computed_checksum == packet.checksum);
    }

    bool objectV2ParsePacket(const std::vector<uint8_t> &data, ObjectV2PacketData &packet)
    {
        if (data.size() < 7) // 최소 데이터 크기 확인 (Version 및 필수 필드 포함)
            return false;

        // 고정 필드 파싱
        packet.pre_amble_0 = data[0];
        packet.pre_amble_1 = data[1];
        packet.data_size = data[2];
        packet.header = data[3];
        packet.version_1 = data[4];
        packet.version_2 = data[5];
        packet.number_of_objects = data[6];

        size_t offset = 7; // Version 필드를 포함한 이후의 오프셋

        // 객체 데이터 파싱
        for (uint8_t i = 0; i < packet.number_of_objects; ++i)
        {
            // 객체의 필드 크기를 확인하여 남은 데이터가 충분한지 확인
            if (offset + 14 > data.size()) // ObjectDataV2 크기는 14바이트
                return false;

            ObjectDataV2 obj;
            obj.class_id = data[offset];
            obj.confidence = data[offset + 1];
            obj.x = static_cast<int16_t>((data[offset + 3] << 8) | data[offset + 2]);
            obj.y = static_cast<int16_t>((data[offset + 5] << 8) | data[offset + 4]);
            obj.theta = static_cast<int16_t>((data[offset + 7] << 8) | data[offset + 6]);
            obj.width = static_cast<int16_t>((data[offset + 9] << 8) | data[offset + 8]);
            obj.height = static_cast<int16_t>((data[offset + 11] << 8) | data[offset + 10]);
            obj.distance = static_cast<uint16_t>((data[offset + 13] << 8) | data[offset + 12]);

            packet.objects.push_back(obj);
            offset += 14; // 객체 크기만큼 오프셋 증가
        }

        // 마지막 남은 바이트가 체크섬인지 확인
        if (offset >= data.size())
            return false;

        // 체크섬 파싱
        packet.checksum = data[offset];

        // 체크섬 계산 및 검증
        uint8_t computed_checksum = 0;
        for (size_t i = 3; i < offset; ++i) // 헤더와 그 이후 필드 포함
        {
            computed_checksum += data[i];
        }

        return (computed_checksum == packet.checksum);
    }
    
    #if USE_LINELASER_SENSOR > 0
    bool LLParsePacket(const std::vector<uint8_t> &data, LLPacketData &packet)
    {
        packet.pre_amble_0 = data[0];
        packet.pre_amble_1 = data[1];
        packet.data_size = data[2];
        packet.header = data[3];
        packet.number_of_2LL = data[4];

        size_t offset = 5; 

        for (uint8_t i = 0; i < packet.number_of_2LL; ++i)
        {
            if (offset + 12 > data.size())
                return false;

            LLData L2;
            L2.x = static_cast<int16_t>((data[offset + 1] << 8) | data[offset]);
            L2.y = static_cast<int16_t>((data[offset + 3] << 8) | data[offset + 2]);
            L2.theta = static_cast<int16_t>((data[offset + 5] << 8) | data[offset + 4]);
            L2.width = static_cast<int16_t>((data[offset + 7] << 8) | data[offset + 6]);
            L2.height = static_cast<int16_t>((data[offset + 9] << 8) | data[offset + 8]);
            L2.distance = static_cast<uint16_t>((data[offset + 11] << 8) | data[offset + 10]);

            packet.LL.push_back(L2);
            offset += 12;
        }

        if (offset >= data.size())
            return false;

        packet.checksum = data[offset];

        uint8_t computed_checksum = 0;
        for (size_t i = 3; i < offset; ++i)
        {
            computed_checksum += data[i];
        }

        return (computed_checksum == packet.checksum);
    }

    bool LLV2ParsePacket(const std::vector<uint8_t> &data, LLV2PacketData &packet)
    {
        // Minimum size check: preambles, data_size, header, versions, number_of_2LL
        if (data.size() < 7) 
            return false;

        packet.pre_amble_0 = data[0];
        packet.pre_amble_1 = data[1];
        packet.data_size = data[2];
        packet.header = data[3];
        packet.version_1 = data[4];     // First version field
        packet.version_2 = data[5];     // Second version field
        packet.number_of_2LL = data[6]; // Number of LLData

        size_t offset = 7; // Updated offset after version fields

        for (uint8_t i = 0; i < packet.number_of_2LL; ++i)
        {
            // Updated size check to match the new structure
            if (offset + 11 > data.size())
                return false;

            LLDataV2 L2;
            L2.x = static_cast<int16_t>((data[offset + 1] << 8) | data[offset]);
            L2.y = static_cast<int16_t>((data[offset + 3] << 8) | data[offset + 2]);
            L2.theta = static_cast<int16_t>((data[offset + 5] << 8) | data[offset + 4]);
            L2.direction = static_cast<int8_t>(data[offset + 6]); // 0: Left, 1: Right
            L2.reserved = static_cast<int8_t>(data[offset + 7]);
            L2.height = static_cast<int16_t>((data[offset + 9] << 8) | data[offset + 8]);
            L2.distance = static_cast<uint16_t>((data[offset + 11] << 8) | data[offset + 10]);

            packet.LL.push_back(L2);
            offset += 12; // Update offset based on the new structure size
        }

        if (offset >= data.size())
            return false;

        packet.checksum = data[offset];

        uint8_t computed_checksum = 0;
        for (size_t i = 3; i < offset; ++i)
        {
            computed_checksum += data[i];
        }

        return (computed_checksum == packet.checksum);
    }
    #endif

    bool parseErrorDataPacket(const std::vector<uint8_t> &data, ErrorDataPacket &packet) {
        // Ensure the packet has the minimum size
        if (data.size() < 6) { // Minimum size includes fixed fields and at least 1 error section
            return false;
        }

        // Parse fixed fields
        packet.PRE_AMBLE_0 = data[0];
        packet.PRE_AMBLE_1 = data[1];
        packet.Data_Size = data[2];
        packet.Header_Command = data[3];
        packet.Number_of_Objects = data[4];

        // Validate packet size
        size_t expected_size = 5 + packet.Number_of_Objects * sizeof(ErrorSection) + 1; // Fixed fields + errors + checksum
        if (data.size() != expected_size) {
            return false;
        }

        size_t offset = 5; // Start of the Errors section

        // Parse ErrorSections
        packet.Errors.clear();
        for (uint8_t i = 0; i < packet.Number_of_Objects; ++i) {
            if (offset + sizeof(ErrorSection) > data.size()) {
                return false; // Prevent out-of-bounds access
            }

            ErrorSection error;
            error.occurred = data[offset++];
            error.rank = data[offset++];
            for (int j = 0; j < 3; ++j) {
                error.errorCode[j] = data[offset++];
            }

            packet.Errors.push_back(error);
        }

        // Parse CHECKSUM
        packet.CHECKSUM = data[offset++];

        // Verify CHECKSUM
        uint8_t computed_checksum = 0;
        for (size_t i = 3; i < offset - 1; ++i) { // Start from Header_Command to the byte before CHECKSUM
            computed_checksum += data[i];
        }

        return (computed_checksum == packet.CHECKSUM);
    }



    void logParsedPacket(const ErrorDataPacket &packet)
    {
        std::cout << "Packet parsed successfully!" << std::endl;
        std::cout << "PRE_AMBLE_0: " << std::hex << static_cast<int>(packet.PRE_AMBLE_0) << std::endl;
        std::cout << "PRE_AMBLE_1: " << std::hex << static_cast<int>(packet.PRE_AMBLE_1) << std::endl;
        std::cout << "Data_Size: " << std::dec << static_cast<int>(packet.Data_Size) << std::endl;
        std::cout << "Header_Command: " << std::hex << static_cast<int>(packet.Header_Command) << std::endl;
        std::cout << "Number of Objects: " << std::dec << static_cast<int>(packet.Number_of_Objects) << std::endl;

        auto logErrorSection = [](const ErrorSection &error, size_t index)
        {
            std::cout << "Error" << index + 1 << " occurred: " << std::dec << static_cast<int>(error.occurred) << std::endl;
            std::cout << "Error" << index + 1 << " rank: " << static_cast<int>(error.rank) << std::endl;
            std::cout << "Error" << index + 1 << " errorCode: ";
            for (int i = 0; i < 3; ++i)
            {
                std::cout << std::hex << static_cast<int>(error.errorCode[i]) << " ";
            }
            std::cout << std::endl;
        };

        // Iterate over all ErrorSections
        for (size_t i = 0; i < packet.Errors.size(); ++i)
        {
            logErrorSection(packet.Errors[i], i);
        }

        std::cout << "CHECKSUM: " << std::hex << static_cast<int>(packet.CHECKSUM) << std::endl;
    }

    bool parseVersionResponsePacket(const std::vector<uint8_t> &data, VersionResponsePacket &packet)
    {
        // Validate minimum size of the packet
        if (data.size() < 7)
        {
            return false; // Packet too small
        }

        // Parse fixed fields
        packet.PRE_AMBLE_0 = data[0];
        packet.PRE_AMBLE_1 = data[1];
        packet.Data_Size = data[2];
        packet.Header_Command = data[3];
        packet.Version1 = data[4];
        packet.Version2 = data[5];
        packet.CHECKSUM = data[6];

        // Compute checksum
        uint8_t computed_checksum = data[3] + data[4] + data[5];
        return computed_checksum == packet.CHECKSUM;
    }

    void logVersionResponsePacket(const VersionResponsePacket &packet)
    {
        std::cout << "Version Response Packet Parsed Successfully!" << std::endl;
        std::cout << "PRE_AMBLE_0: 0x" << std::hex << static_cast<int>(packet.PRE_AMBLE_0) << std::endl;
        std::cout << "PRE_AMBLE_1: 0x" << std::hex << static_cast<int>(packet.PRE_AMBLE_1) << std::endl;
        std::cout << "Data_Size: " << std::dec << static_cast<int>(packet.Data_Size) << std::endl;
        std::cout << "Header_Command: 0x" << std::hex << static_cast<int>(packet.Header_Command) << " (VersionResponse)" << std::endl;
        std::cout << "Version1: " << std::dec << static_cast<int>(packet.Version1) << std::endl;
        std::cout << "Version2: " << std::dec << static_cast<int>(packet.Version2) << std::endl;
        std::cout << "CHECKSUM: 0x" << std::hex << static_cast<int>(packet.CHECKSUM) << std::endl;
    }

    bool parseCalibrationResultPacket(const std::vector<uint8_t> &data, CalibrationResultPacket &packet)
    {
        // Validate minimum size of the packet
        if (data.size() < 6)
        {
            return false; // Packet too small
        }

        // Parse fixed fields
        packet.PRE_AMBLE_0 = data[0];
        packet.PRE_AMBLE_1 = data[1];
        packet.Data_Size = data[2];
        packet.Header_Command = data[3];
        packet.Result = data[4];
        packet.CHECKSUM = data[5];

        // Compute checksum
        uint8_t computed_checksum = data[3] + data[4]; // ADD(4,5) = Header_Command + Result
        return computed_checksum == packet.CHECKSUM;
    }

    bool parseSensorControlPacket(const std::vector<uint8_t> &data, SensorControPacket &packet)
    {
        // Validate minimum size of the packet
        if (data.size() < 6)
        {
            return false; // Packet too small
        }

        // Parse fixed fields
        packet.PRE_AMBLE_0 = data[0];
        packet.PRE_AMBLE_1 = data[1];
        packet.Data_Size = data[2];
        packet.Header_Command = data[3];
        packet.Result = data[4];
        packet.CHECKSUM = data[5];

        // Compute checksum
        uint8_t computed_checksum = data[3] + data[4]; // ADD(4,5) = Header_Command + Result
        return computed_checksum == packet.CHECKSUM;
    }

    void logCalibrationResultPacket(const CalibrationResultPacket &packet)
    {
        std::cout << "Calibration Result Packet Parsed Successfully!" << std::endl;
        std::cout << "PRE_AMBLE_0: 0x" << std::hex << static_cast<int>(packet.PRE_AMBLE_0) << std::endl;
        std::cout << "PRE_AMBLE_1: 0x" << std::hex << static_cast<int>(packet.PRE_AMBLE_1) << std::endl;
        std::cout << "Data_Size: " << std::dec << static_cast<int>(packet.Data_Size) << std::endl;
        std::cout << "Header_Command: 0x" << std::hex << static_cast<int>(packet.Header_Command) << " (CalibrationResult)" << std::endl;
        std::cout << "Result: " << std::dec << static_cast<int>(packet.Result) << std::endl;
        std::cout << "CHECKSUM: 0x" << std::hex << static_cast<int>(packet.CHECKSUM) << std::endl;
    }

    void logSensorControResultPacket(const SensorControPacket &packet)
    {
        std::cout << "SensorContorl Result Packet Parsed Successfully!" << std::endl;
        std::cout << "PRE_AMBLE_0: 0x" << std::hex << static_cast<int>(packet.PRE_AMBLE_0) << std::endl;
        std::cout << "PRE_AMBLE_1: 0x" << std::hex << static_cast<int>(packet.PRE_AMBLE_1) << std::endl;
        std::cout << "Data_Size: " << std::dec << static_cast<int>(packet.Data_Size) << std::endl;
        std::cout << "Header_Command: 0x" << std::hex << static_cast<int>(packet.Header_Command) << " (CalibrationResult)" << std::endl;
        std::cout << "Result: " << std::dec << static_cast<int>(packet.Result) << std::endl;
        std::cout << "CHECKSUM: 0x" << std::hex << static_cast<int>(packet.CHECKSUM) << std::endl;
    }

    void parseDataFields(const std::vector<uint8_t> &data)
    {
        if (data.size() < 6)
        {
            std::cerr << "Data size too small for parsing." << std::endl;
            return;
        }
        #if USE_DEBUG_LOG
        std::ostringstream ss;
        for (size_t i = 0; i < data.size(); ++i)
        {
            ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]) << ' ';
        }

        RCLCPP_INFO(this->get_logger(), "Received data (hex): %s", ss.str().c_str());
        #endif

		if (data[3] == static_cast<uint8_t>(CommandNumber::ProtocolV1_Objects)) // 카메라
        {
            ObjectPacketData packet;
            objectParsePacket(data, packet);
            //RCLCPP_INFO(this->get_logger(),"obsject Number : %u",packet.number_of_objects );
            robot_custom_msgs::msg::CameraData camera_data;
            robot_custom_msgs::msg::CameraDataArray camera_data_array;
            camera_data_array.timestamp = this->get_clock()->now();
            camera_data_array.num = packet.number_of_objects;
            for (const auto &obj : packet.objects)
            {
                camera_data.x = static_cast<double>(obj.x * 0.001);
                camera_data.y = static_cast<double>(obj.y * 0.001);
                camera_data.theta = -static_cast<double>(obj.theta * M_PI/180); //[rad] - cw(+) // 카메라 장애물 좌표계는 로봇과 y축 반전 ( 오른쪽 + , 왼쪽 - )
                camera_data.width = static_cast<double>(obj.width * 0.001);
                camera_data.height = static_cast<double>(obj.height * 0.001);
                camera_data.distance = static_cast<double>(obj.distance * 0.001);
                camera_data_array.data_array.push_back(camera_data);
            }
            camera_data_array.robot_x = amcl_pose_x;
            camera_data_array.robot_y = amcl_pose_y;
            camera_data_array.robot_angle = amcl_pose_angle;
            camera_data_publisher_->publish(camera_data_array);
        }
        #if USE_LINELASER_SENSOR > 0
        else if (data[3] == static_cast<uint8_t>(CommandNumber::ProtocolV1_2LL)) // 2LL
        {
            if(use_2LL_ == 0)
            {
                // RCLCPP_INFO(this->get_logger(), "set 2LL Packet parsed remove option");
                return;
            }

            LLPacketData packet;
            LLParsePacket(data, packet);

            robot_custom_msgs::msg::AIData LL_data;
            for (const auto &L2 : packet.LL)
            {
                // LL_data.timestamp = this->get_clock()->now();
                LL_data.x = static_cast<double>(L2.x * 0.001);
                LL_data.y = static_cast<double>(L2.y * 0.001);
                LL_data.theta = static_cast<double>(L2.theta * M_PI/180); // [rad]
                LL_data.width = static_cast<double>(L2.width * 0.001);
                LL_data.height = static_cast<double>(L2.height * 0.001);
                LL_data.distance = static_cast<double>(L2.distance * 0.001);
                line_laser_data_publisher_->publish(LL_data); // [m]
            }
        }
        #endif
        else if (data[3] == static_cast<uint8_t>(CommandNumber::ProtocolV2_Objects )) // 카메라
        {
            ObjectV2PacketData packet;
            objectV2ParsePacket(data, packet);
            //RCLCPP_INFO(this->get_logger(), "cameraData received");
            std::string tmp = std::to_string(packet.version_1)+'.'+std::to_string(packet.version_2);
            if( aiVersion != tmp){ 
                aiVersion = tmp;
                // RCLCPP_INFO(this->get_logger(), "packet.version_1 : %u", packet.version_1);
                // RCLCPP_INFO(this->get_logger(), "packet.version_2 : %u", packet.version_2);

                // UInt8 메시지 생성
                std_msgs::msg::String msg;
                msg.data = tmp;  // unsigned short -> uint8_t로 캐스팅

                ai_version_publisher_->publish(msg);
            }

            robot_custom_msgs::msg::CameraData camera_data;
            robot_custom_msgs::msg::CameraDataArray camera_data_array;
            camera_data_array.timestamp = this->get_clock()->now();
            camera_data_array.num = packet.number_of_objects;
            
            for (const auto &obj : packet.objects)
            {
                camera_data.id = obj.class_id;          // class id
                camera_data.score = obj.confidence;     // confidence score [0,100]
                camera_data.x = std::round(static_cast<double>(obj.x * 0.001) * 1000) / 1000;
                camera_data.y = std::round(static_cast<double>(obj.y * 0.001) * 1000) / 1000;
                camera_data.theta = -static_cast<double>(obj.theta * M_PI/180); //[rad] - cw(+) // 카메라 장애물 좌표계는 로봇과 y축 반전 ( 오른쪽 + , 왼쪽 - )
                camera_data.width = std::round(static_cast<double>(obj.width * 0.001) * 1000) / 1000;
                camera_data.height = std::round(static_cast<double>(obj.height * 0.001) * 1000) / 1000;
                camera_data.distance = std::round(static_cast<double>(obj.distance * 0.001) * 1000) / 1000;
                camera_data_array.data_array.push_back(camera_data);
            }
            camera_data_array.robot_x = amcl_pose_x;
            camera_data_array.robot_y = amcl_pose_y;
            camera_data_array.robot_angle = amcl_pose_angle;
            camera_data_publisher_->publish(camera_data_array);
        }
        #if USE_LINELASER_SENSOR > 0
        else if (data[3] == static_cast<uint8_t>(CommandNumber::ProtocolV2_2LL)) // 2LL
        {
            //RCLCPP_INFO(this->get_logger(), "linelaserData received");
            if (use_2LL_ == 0)
            {
                // RCLCPP_INFO(this->get_logger(), "set 2LL Packet parsed remove option");
                return;
            }
            LLV2PacketData packet;
            std::string tmp = std::to_string(packet.version_1)+'.'+std::to_string(packet.version_2);
            if( aiVersion != tmp){ 
                aiVersion = tmp;
                // RCLCPP_INFO(this->get_logger(), "packet.version_1 : %u", packet.version_1);
                // RCLCPP_INFO(this->get_logger(), "packet.version_2 : %u", packet.version_2);

                // UInt8 메시지 생성
                std_msgs::msg::String msg;
                msg.data = tmp;  // unsigned short -> uint8_t로 캐스팅

                ai_version_publisher_->publish(msg);
            }
            LLV2ParsePacket(data, packet);

            robot_custom_msgs::msg::LineLaserData LL_data;
            robot_custom_msgs::msg::LineLaserDataArray LL_data_array;
            LL_data_array.timestamp = this->get_clock()->now();
            LL_data_array.num = packet.number_of_2LL;

            for (const auto &L2 : packet.LL)
            {
                LL_data.x = std::round(static_cast<double>(L2.x * 0.001) * 1000) / 1000;
                LL_data.y = std::round(static_cast<double>(L2.y * 0.001) * 1000) / 1000;
                LL_data.theta = -static_cast<double>(L2.theta * M_PI/180); // [rad]
                LL_data.direction = static_cast<int8_t>(L2.direction);      // Publish direction as is
                // LL_data.reserved = static_cast<int8_t>(L2.reserved);        // Publish reserved as is
                LL_data.height = std::round(static_cast<double>(L2.height * 0.001) * 1000) / 1000;
                LL_data.distance = std::round(static_cast<double>(L2.distance * 0.001) * 1000) / 1000;
                LL_data_array.data_array.push_back(LL_data);
            }
            LL_data_array.robot_x = amcl_pose_x;
            LL_data_array.robot_y = amcl_pose_y;
            LL_data_array.robot_angle = amcl_pose_angle;
            line_laser_data_publisher_->publish(LL_data_array);
        }
        #endif
        else if (data[3] == static_cast<uint8_t>(CommandNumber::Error )) // Error
        {
            ErrorDataPacket packet;

            if (parseErrorDataPacket(data, packet)) {
                logParsedPacket(packet);
            } else {
                std::cerr << "Failed to parse packet." << std::endl;
            }
        }else if (data[3] == static_cast<uint8_t>(CommandNumber::VersionResponse)) {
            VersionResponsePacket packet;

            if (parseVersionResponsePacket(data, packet)) {
                logVersionResponsePacket(packet);
                std::string tmp = std::to_string(packet.Version1)+'.'+std::to_string(packet.Version2);
                RCLCPP_INFO(this->get_logger(), "resPonse AI VERSION : %s", tmp.c_str());
                
                if( aiVersion != tmp){ 
                    aiVersion = tmp;
                    // RCLCPP_INFO(this->get_logger(), "packet.Version1 : %u", packet.Version1);
                    // RCLCPP_INFO(this->get_logger(), "packet.Version2 : %u", packet.Version2);

                    // UInt8 메시지 생성
                    RCLCPP_INFO(this->get_logger(), "resPonse AI VERSION UPDATE: %s", tmp.c_str());
                    std_msgs::msg::String msg;
                    msg.data = tmp;  // unsigned short -> uint8_t로 캐스팅

                    ai_version_publisher_->publish(msg);
                }
            } else {
                std::cerr << "Failed to parse Version Response packet." << std::endl;
            }
        }else if (data[3] == static_cast<uint8_t>(CommandNumber::CalibrationResponse)) {
            CalibrationResultPacket packet;

            if (parseCalibrationResultPacket(data, packet)) {
                logCalibrationResultPacket(packet);
            } else {
                std::cerr << "Failed to parse Calibration Result Packet(." << std::endl;
            }
        }else if (data[3] == static_cast<uint8_t>(CommandNumber::SensorControlResponse)) {
            SensorControPacket packet;
            RCLCPP_INFO(this->get_logger(), "resPonse Camera ON-OFF ");
            if (parseSensorControlPacket(data, packet)) {
                logSensorControResultPacket(packet);
            } else {
                std::cerr << "Failed to parse Sensor Control Result Packet(." << std::endl;
            }
        }
        else
        {

            RCLCPP_WARN(this->get_logger(), "Unexpected Header(Command): 0x%02x", data[3]);
        }
    }

    float normalizeAngle(float angle) {
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2 * M_PI;
        }
        return angle;
    }

    // Example file path for sending data
    // std::string file_path = "/path/to/your/file.bin";  // Update this path
    // sendFileOverSerial(file_path);
    void sendFileOverSerial(const std::string &file_path)
    {
        std::ifstream file(file_path, std::ios::binary);
        
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file: %s", file_path.c_str());
            return;
        }

        const size_t chunk_size = 1024; // 1KB씩 읽어 전송
        std::vector<char> buffer(chunk_size);

        size_t total_bytes_sent = 0;
        while (file.good())
        {
            file.read(buffer.data(), chunk_size);
            std::streamsize bytes_read = file.gcount(); // 실제로 읽은 바이트 수

            if (bytes_read > 0)
            {
                //checksum이 필요하면 나중에.

                // 시리얼로 데이터 전송
                size_t bytes_written = serial_.write(std::string(buffer.data(), bytes_read));
                total_bytes_sent += bytes_written;

                RCLCPP_INFO(this->get_logger(), "Sent %zu bytes (total: %zu bytes)", bytes_written, total_bytes_sent);
            }
        }

        RCLCPP_INFO(this->get_logger(), "File transmission completed. Total bytes sent: %zu", total_bytes_sent);
    }

    void publishVersion()
    {
        std_msgs::msg::String msg;
        msg.data = aiVersion;  // unsigned short -> uint8_t로 캐스팅
        ai_version_publisher_->publish(msg);
    }

};

void signal_handler(int signal)
{
    switch(signal)
    {
        case SIGINT:
        case SIGABRT:
        case SIGSEGV:
        case SIGTERM:
        case SIGKILL:
        default:
            break;
    }

    rclcpp::shutdown(); // This ensures that the node shuts down properly on SIGINT
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::string port_;

    // Register a custom signal handler to ensure proper shutdown
    std::signal(SIGINT, signal_handler);

    // Create the AIInterface node
    auto node = std::make_shared<AIInterface>();
   
    // Spin the node
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
