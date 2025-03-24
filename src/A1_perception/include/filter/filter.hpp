#ifndef __FILTER_HPP__
#define __FILTER_HPP__

#include <memory>
#include <string>
#include <vector>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include "yaml-cpp/yaml.h"

#include "layer.hpp"

namespace A1::perception
{
// Forward declaration
class PerceptionNode;

template <typename T>
T getYamlValue(const std::string func, const YAML::Node& config, const std ::string& key, const T& default_value)
{
    if (config[key])
    {
        return config[key].as<T>();
    }
    else
    {
        std::cout << " [" << func << "] - [" << key << "] yaml key is empty" << std::endl;
        return default_value;
    }
}

/**
 * @brief BaseFilter 클래스는 모든 필터 클래스의 기본 인터페이스를 제공하는 추상 클래스입니다.
 * 각 필터는 이 클래스를 상속받아 updateImpl 메소드를 구현함으로써 데이터 필터링 기능을 제공합니다.
 */
class BaseFilter
{
   public:
    BaseFilter(std::shared_ptr<PerceptionNode> node_ptr_);

    virtual ~BaseFilter() = default;

    LayerVector update(LayerVector layer_vector);

    /**
     * @brief 필터가 참조하는 PerceptionNode 스마트 포인터를 반환합니다.
     *
     * @return std::shared_ptr<PerceptionNode> PerceptionNode의 스마트 포인터
     */
    std::shared_ptr<PerceptionNode> getNodePtr() const;

   protected:
    /**
     * @brief 필터를 적용하여 데이터 레이어를 업데이트합니다.
     *
     * @param layer_vector 필터링 대상 데이터 (예: 포인트 클라우드)
     * @return LayerVector 필터 적용 후의 결과 데이터
     */
    virtual LayerVector updateImpl(LayerVector layer_vector) = 0;

    std::shared_ptr<PerceptionNode> node_ptr{};
};
using BaseFilterPtr = std::shared_ptr<BaseFilter>;

/**
 * @brief TimeoutFilter 클래스는 지정된 시간(밀리초) 경과 후 데이터를 무효화하여 필터링합니다.
 *
 * [YAML 설정 예시]
 * --------------------------------------------------
 * timeout:
 *   timeout_milliseconds: 100   # 데이터를 무효화할 시간 (밀리초 단위)
 * --------------------------------------------------
 *
 * @param timeout_milliseconds : 타임아웃 시간 (밀리초, 예: 100)
 */
class TimeoutFilter : public BaseFilter
{
   public:
    TimeoutFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    LayerVector updateImpl(LayerVector layer_vector) override;

    rclcpp::Duration timeout{0, 0};
};

/**
 * @brief DensityFilter 클래스는 지정된 반경 내에서 최대 점 개수를 유지하도록 데이터를 필터링합니다.
 *
 * [YAML 설정 예시]
 * --------------------------------------------------
 * density:
 *   max_count: 3        # 허용되는 최대 점 개수 (예: 3)
 *   radius: 0.1         # 점 밀도를 평가할 반경 (예: 0.1)
 * --------------------------------------------------
 *
 * @param max_count : 필터링 후 유지할 최대 점 개수
 * @param radius    : 점 밀도 평가를 위한 반경
 */
class DensityFilter : public BaseFilter
{
   public:
    DensityFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    LayerVector updateImpl(LayerVector layer_vector) override;

    int32_t max_count{};
    float radius{};
};

/**
 * @brief RoIFilter 클래스는 관심 영역(Region Of Interest)을 지정하여 데이터를 필터링하는 추상 클래스입니다.
 * 구체 여타 클래스(PolygonRoIFilter, SectorRoIFilter)에서 영역 선택 방식에 따라 데이터 필터링을 구현합니다.
 *
 * [YAML 설정 예시 - 다각형]
 * --------------------------------------------------
 * polygon_roi:
 *   use_inside: true     # true: 다각형 내부의 포인트 선택, false: 외부 선택
 *   polygon:            # 다각형 영역을 구성하는 [x, y] 좌표 리스트
 *     - [ 0.0, -0.4 ]
 *     - [ 0.6, -0.4 ]
 *     - [ 0.6,  0.4 ]
 *     - [ 0.0,  0.4 ]
 * --------------------------------------------------
 *
 * [YAML 설정 예시 - 섹터]
 * --------------------------------------------------
 * sector_roi:
 *   min_range: 0.5      # 필터링 시작 최소 거리 (예: 0.5)
 *   max_range: 5.0      # 필터링에 사용할 최대 거리 (예: 5.0)
 *   min_angle: -0.785   # 필터링 최소 각도 (라디안, 예: -0.785)
 *   max_angle: 0.785    # 필터링 최대 각도 (라디안, 예: 0.785)
 * --------------------------------------------------
 */
class RoIFilter : public BaseFilter
{
   public:
    RoIFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   protected:
    virtual bool isInside(pcl::PointXY point) = 0;
    bool use_inside{true};
    bool publish_stop{true};

   private:
    LayerVector updateImpl(LayerVector layer_vector) override;
};

/**
 * @brief PolygonRoIFilter 클래스는 다각형 영역 내 또는 외부의 포인트를 필터링합니다.
 *
 * [YAML 설정 예시]
 * --------------------------------------------------
 * polygon_roi:
 *   use_inside: true     # 내부 포인트 선택: true, 외부: false
 *   polygon:            # 관심 영역을 구성할 다각형 좌표 (각 좌표는 [x, y] 형태)
 *     - [ 0.0, -0.4 ]
 *     - [ 0.6, -0.4 ]
 *     - [ 0.6,  0.4 ]
 *     - [ 0.0,  0.4 ]
 * --------------------------------------------------
 */
class PolygonRoIFilter : public RoIFilter
{
   public:
    PolygonRoIFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    bool isInside(pcl::PointXY point) override;
    // 다각형 영역 좌표를 저장한 포인트 클라우드 (각 점은 [x, y] 좌표)
    pcl::PointCloud<pcl::PointXY> polygon{};
};

/**
 * @brief SectorRoIFilter 클래스는 섹터(원뿔 모양의 영역) 기준으로 포인트를 필터링합니다.
 *
 * [YAML 설정 예시]
 * --------------------------------------------------
 * sector_roi:
 *   min_range: 0.5      # 필터링에 사용할 최소 거리 (예: 0.5)
 *   max_range: 5.0      # 필터링에 사용할 최대 거리 (예: 5.0)
 *   min_angle: -0.785   # 필터링 최소 각도 (라디안 단위, 예: -0.785)
 *   max_angle: 0.785    # 필터링 최대 각도 (라디안 단위, 예: 0.785)
 * --------------------------------------------------
 *
 * @param min_range : 섹터 필터링 최소 거리
 * @param max_range : 섹터 필터링 최대 거리
 * @param min_angle : 섹터 필터링 최소 각도 (라디안)
 * @param max_angle : 섹터 필터링 최대 각도 (라디안)
 */
class SectorRoIFilter : public RoIFilter
{
   public:
    SectorRoIFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    bool isInside(pcl::PointXY point) override;
    float min_range{};
    float max_range{};
    float min_angle{};
    float max_angle{};
};

/**
 * @brief ComposeFilter 클래스는 여러 개의 필터를 순차적으로 적용하는 복합 필터입니다.
 *
 * [YAML 설정 예시]
 * --------------------------------------------------
 * compose:
 *   filters:
 *     timeout:
 *       timeout_milliseconds: 100   # TimeoutFilter: 데이터를 무효화할 시간 (밀리초)
 *     polygon_roi:
 *       use_inside: true            # PolygonRoIFilter: 내부 여부 (true이면 내부 포인트 선택)
 *       polygon:                    # PolygonRoIFilter: 다각형 좌표 목록
 *         - [ 0.0, -0.4 ]
 *         - [ 0.6, -0.4 ]
 *         - [ 0.6,  0.4 ]
 *         - [ 0.0,  0.4 ]
 *     density:
 *       max_count: 3                # DensityFilter: 유지할 최대 점 수
 *       radius: 0.1                 # DensityFilter: 밀도 평가 반경
 * --------------------------------------------------
 */
class ComposeFilter : public BaseFilter
{
   public:
    ComposeFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    LayerVector updateImpl(LayerVector layer_vector) override;

    std::vector<std::shared_ptr<BaseFilter>> filters{};
};

/**
 * @brief DropOffFilter 단차 검증을 위해 만든 필터입니다.
 *
 * [YAML 설정 예시]
 * --------------------------------------------------
 * compose:
 *   filters:
 *     timeout:
 *       timeout_milliseconds: 100   # TimeoutFilter: 데이터를 무효화할 시간 (밀리초)
 *     drop_off:
 *       min_range: 0.58             # DropOffFilter: 최소 거리값
 *       max_range: 0.7              # DropOffFilter: 최대 거리값
 *       line_length: 0.2            # DropOffFilter: 생성할 직선 길이/2(m)
 *       resolution: 0.05            # DropOffFilter: 생성할 직선의 레졸루션(m)
 *     density:
 *       max_count: 3                # DensityFilter: 유지할 최대 점 수
 *       radius: 0.1                 # DensityFilter: 밀도 평가 반경
 * --------------------------------------------------
 */
class DropOffFilter : public BaseFilter
{
   public:
    DropOffFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    LayerVector updateImpl(LayerVector layer_vector) override;
    float min_range{};
    float line_length{};
    float resolution{};
};

/**
 * @brief OneDTofFilter 1D ToF 검증을 위해 만든 필터입니다.
 */
class OneDTofFilter : public BaseFilter
{
   public:
    OneDTofFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    LayerVector updateImpl(LayerVector layer_vector) override;
    int layer_vector_size{};
    float line_length{};
    float resolution{};
    float lidar_wall_check_threshold_sqr{};
    bool use_stop{false};
};

/**
 * @brief DistCheckFilter 벽인지 검사하기 위해 체크하기 위한 필터.
 *
 * [YAML 설정 예시]
 * --------------------------------------------------
 * compose:
 *   filters:
 *     timeout:
 *       timeout_milliseconds: 100   # TimeoutFilter: 데이터를 무효화할 시간 (밀리초)
 *     dist_check:
 *       min_range: 0.0              # DistCheckFilter: 거리 검사를 위한 최소 거리값
 *       max_range: 0.30             # DistCheckFilter: 거리 검사를 위한 최대 거리값
 *       inputs:                     # DistCheckFilter: 거리 검사를 input layers
 *       - multi_tof_right_row1
 *       - multi_tof_right_row2
 *     density:
 *       max_count: 3                # DensityFilter: 유지할 최대 점 수
 *       radius: 0.1                 # DensityFilter: 밀도 평가 반경
 * --------------------------------------------------
 */
class DistCheckFilter : public BaseFilter
{
   public:
    DistCheckFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    LayerVector updateImpl(LayerVector layer_vector) override;
    float min_range{};
    float max_range{};
    std::vector<std::string> inputs{};
};

/**
 * @brief OneDTofFilter 1D ToF 검증을 위해 만든 필터입니다.
 */
class CollisionFilter : public BaseFilter
{
   public:
    CollisionFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    LayerVector updateImpl(LayerVector layer_vector) override;
    float line_length{};
    float resolution{};
};
}  // namespace A1::perception

#endif  // __FILTER_HPP__
