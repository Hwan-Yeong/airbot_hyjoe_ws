#ifndef __PERCEPTION_NODE_HPP__
#define __PERCEPTION_NODE_HPP__

#include <any>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/empty.hpp"
#include "yaml-cpp/yaml.h"

#include "filter/filter.hpp"
#include "layer.hpp"
#include "motor_status.hpp"
#include "position.hpp"

namespace A1::perception
{

/**
 * @class PerceptionNode
 * @brief 로봇 인식 데이터 처리를 위한 ROS2 노드 클래스.
 *
 * 이 클래스는 ROS2의 rclcpp::Node를 상속받아, 다양한 센서 데이터를 처리하고 필터링하는 기능을 제공한다.
 * YAML 설정 파일을 기반으로 구독자, 발행자 및 필터를 초기화하며, 센서 레이어 맵 관리를 통해
 * 각 센서의 데이터를 효율적으로 처리할 수 있다.
 *
 * 주요 기능:
 * - YAML 설정 정보를 사용하여 구독자, 발행자, 필터 초기화 (initSubscribers, initPublishers, initFilters).
 * - 센서 레이어와 데이터 필터의 관리 (setSensorLayerMap, getSensorLayer).
 * - 주기적 타이머 콜백(timerCallback)을 통해 실시간 데이터 처리 수행.
 * - pcl::PointCloud를 ROS 메시지(sensor_msgs::msg::PointCloud2) 형식으로 변환.
 * - 로봇의 현재 위치 정보 제공 (getPosition).
 *
 * 사용법:
 * 1. YAML 설정 파일을 통해 PerceptionNode 인스턴스를 생성.
 * 2. init() 메서드를 호출하여 초기화 작업 수행.
 * 3. 내부 메서드와 콜백 함수를 통해 센서 데이터를 처리하고, 필요한 작업을 수행.
 */
class PerceptionNode : public rclcpp::Node
{
   public:
    PerceptionNode();
    ~PerceptionNode() override = default;

    void init();

    void setSensorLayerMap(const std::string& name, const Layer& layer);
    Layer getSensorLayer(const std::string& name) const;
    Position getPosition();
    MotorStatus getMotorStatus();
    Position getIMUdata();
    Position setPosition();

    void resetLayers();
    void clearDropOffLayerMap();
    void sendActionStop(int data);
    std::unordered_map<std::string, Layer>& getDropOffLayerMap();

    void setClimb(bool is_climb);
    bool isClimb();

   private:
    void initSubscribers(const YAML::Node& config);
    void initPublishers(const YAML::Node& config);
    void initFilters(const YAML::Node& config);
    void initController();
    void climbCheck();
    double compute_exponential_weight_moving_average(double prev, double curr);
    void timerCallback();

    sensor_msgs::msg::PointCloud2 convertToPointCloud2(const pcl::PointCloud<pcl::PointXYZ>& cloud);

    YAML::Node config{};

    Position robot_position{};
    Position imu_data{};
    MotorStatus motor_status{};

    rclcpp::TimerBase::SharedPtr timer{};

    std::unordered_map<std::string, std::any> controller_subscribers{};
    std::unordered_map<std::string, std::any> subscribers{};
    std::unordered_map<std::string, std::any> publishers{};

    std::unordered_map<std::string, BaseFilterPtr> filters{};
    std::unordered_map<std::string, Layer> drop_off_layer_map{};
    std::unordered_map<std::string, Layer> sensor_layer_map{};
    std::unordered_map<std::string, LayerVector> layers{};

    float climbing_pitch_alpha{};
    float enable_climbing_threshold{};
    float disable_climbing_threshold{};
    float estimated_bias{};
    int climbing_timeout;
    rclcpp::Time climbing_time{};
    bool climb_flag{false};
};
}  // namespace A1::perception

#endif  // __PERCEPTION_NODE_HPP__