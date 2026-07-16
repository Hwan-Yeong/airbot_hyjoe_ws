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

    rclcpp::Time last_log_time{};
    /**
     * @brief 로그 데이터를 1초에 한번씩만 표현하기 위함입니다.
     *
     * @return 로그 표시 가능 여부
     */

    bool logIntervalPassed();

   private:
    /**
     * @brief 로그 데이터를 1초에 한번씩만 표현하기 위함입니다.
     *
     * @return 로그 표시 가능 여부
     */
    rclcpp::Time getLastLogTime() const;

    void setLastLogTime(rclcpp::Time time);
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
    bool use_when_climb{true};
    bool delete_out_of_range{true};
    bool delete_all_when_climb{false};

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
 *     lidar_dist_check:             # LidarDistCheckFilter: 라이다와 거리 계산을 해서 데이터를 지우는 필터
 *       min_range: 0.0              # LidarDistCheckFilter: 최소 거리값
 *       max_range: 0.3              # LidarDistCheckFilter: 최대 거리값
 *     drop_off:
 *       min_range: 0.58             # DropOffFilter: 최소 거리값
 *       line_length: 0.2            # DropOffFilter: 생성할 직선 길이/2(m)
 *       resolution: 0.05            # DropOffFilter: 생성할 직선의 레졸루션(m)
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
 * @brief DropOffDiffFilter 단차 검증을 위해 만든 필터입니다.
 *
 * [YAML 설정 예시]
 * --------------------------------------------------
 * compose:
 *   filters:
 *     lidar_dist_check:             # LidarDistCheckFilter: 라이다와 거리 계산을 해서 데이터를 지우는 필터
 *       min_range: 0.0              # LidarDistCheckFilter: 최소 거리값
 *       max_range: 0.3              # LidarDistCheckFilter: 최대 거리값
 *     drop_off_diff:
 *       dist_diff: 0.08             # DropOffDiffFilter: 이전 샘플과의 거리 차이
 *       line_length: 0.2            # DropOffDiffFilter: 생성할 직선 길이/2(m)
 *       resolution: 0.05            # DropOffDiffFilter: 생성할 직선의 레졸루션(m)
 * --------------------------------------------------
 */
class DropOffDiffFilter : public BaseFilter
{
   public:
    DropOffDiffFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    LayerVector updateImpl(LayerVector layer_vector) override;
    float standard_dist{};
    std::chrono::steady_clock::time_point prev_time;
    float dist_diff{};
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
    float stop_and_back_dist{};
    bool use_stop{false};
    rclcpp::Time move_back_log_time{};
};

/**
 * @brief LidarDistCheckFilter 벽인지 검사하기 위해 체크하기 위한 필터.
 *
 * [YAML 설정 예시]
 * --------------------------------------------------
 * compose:
 *   filters:
 *     timeout:
 *       timeout_milliseconds: 100   # TimeoutFilter: 데이터를 무효화할 시간 (밀리초)
 *     lidar_dist_check:
 *       min_range: 0.0              # LidarDistCheckFilter: 거리 검사를 위한 최소 거리값
 *       max_range: 0.30             # LidarDistCheckFilter: 거리 검사를 위한 최대 거리값
 *     density:
 *       max_count: 3                # DensityFilter: 유지할 최대 점 수
 *       radius: 0.1                 # DensityFilter: 밀도 평가 반경
 * --------------------------------------------------
 */
class LidarDistCheckFilter : public BaseFilter
{
   public:
    LidarDistCheckFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    LayerVector updateImpl(LayerVector layer_vector) override;
    float min_range{};
    float max_range{};
};

/**
 * @brief CollisionFilter 충돌 감지를 위해 만든 필터입니다.
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

/**
 * @brief CollisionTofFilter 충돌 감지를 위해 만든 필터입니다.
 */
class CollisionTofFilter : public BaseFilter
{
   public:
    CollisionTofFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    LayerVector updateImpl(LayerVector layer_vector) override;
    bool use_data{};
};

/**
 * @brief LowObstacleFilter multi tof 3/4 row를 이용한 장애물 검출 필터.
 *
 * [YAML 설정 예시]
 * --------------------------------------------------
 * compose:
 *   filters:
       lidar_dist_check:            # lidar 거리 검사  필터
         min_range: 0.0             # 비교할 최소 거리
         max_range: 0.10            # 비교할 최대 거리
 *     low_obstacle:
 *       dist_max: 0.30             # LowObstacleFilter: 거리 차가 적혀진 수치 이하면 장애물 생성
 *       dist_min: 0.05             # LowObstacleFilter: 거리 차가 적혀진 수치 이상이면 장애물 생성
 *       input:                     # LowObstacleFilter: 데이터가 나오는지 확인할 인풋(8행)
 *       - multi_tof_left_idx_63
 * --------------------------------------------------
 */
class LowObstacleFilter : public BaseFilter
{
   public:
    LowObstacleFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    LayerVector updateImpl(LayerVector layer_vector) override;
    std::string input{};
    float dist_max{};
    float dist_min{};
    float dist_diff_threshold{};
};

/**
 * @brief DepthCameraLowObstacleFilter
 *        depth 카메라 PointCloud2 에서 낮은 장애물(3~20cm)을 감지/확정하는 필터.
 *
 * 기존 multi tof low_obstacle 과 동일한 생성/제거 정책을 따른다:
 *  - 생성: 검증(높이 band + 전방 코리도 + 스냅샷 내 이웃 수 + lidar 미가시
 *          + 연속 프레임 관측)을 통과한 점을 isDeletable=false 레이어에
 *          "확정(lock)" 하여 영구 유지.
 *  - 제거: timeout 없음. costmap clear(goal/initialpose/UDP) 로만 제거된다.
 *          lidar 검사는 확정 "전" 1회만 수행한다 (lidar 가 보이는 곳이면
 *          벽/일반 장애물이므로 애초에 찍지 않음). 확정된 점을 매 tick
 *          lidar 근접으로 지우면 벽 근처 낮은 장애물이 삭제/재확정을
 *          반복하며 costmap 이 깜박이므로, 체인에 lidar_dist_check 를
 *          두지 말 것.
 *  - 승월(climb) 중에는 신규 감지를 중단한다 (locked 점은 유지).
 *
 * timerCallback 이 매 tick 같은 스냅샷을 중복 push 하므로, 아직 처리하지 않은
 * 최신 스냅샷 1개만 검사하고 나머지 미확정 레이어는 모두 소모(clear)한다.
 * 확정 점은 단일 locked 레이어 하나에 누적되며, radius 내 중복 확정을 막아
 * 점 밀도를 자체 제한한다. (별도 density/polygon_roi/timeout 필터 불필요)
 *
 * [YAML 설정 예시]
 * --------------------------------------------------
 * compose:
 *  filters:
 *    depth_camera_low_obstacle:
 *      min_height: 0.03          # 감지 최소 높이 (map z, m)
 *      max_height: 0.18          # 감지 최대 높이 (map z, m)
 *      min_neighbors: 4          # 스냅샷 내 radius 이내 최소 이웃 수 (공간 확인)
 *      radius: 0.05              # 이웃/연속프레임 매칭/중복 확정 방지 반경 (m)
 *      x_min: 0.10               # 전방 코리도 최소 x (base_link, m)
 *      x_max: 2.0                # 전방 코리도 최대 x (base_link, m)
 *      half_width: 0.20          # 전방 코리도 좌우 반폭 (base_link, m)
 *      confirm_frames: 2         # 확정에 필요한 연속 스냅샷 관측 횟수 (시간 확인)
 *      lidar_check_radius: 0.10  # 후보 이내에 lidar 리턴 있으면 확정 제외 (m, <=0 비활성)
 *    obstacle_logger: { ... }
 * --------------------------------------------------
 */
class DepthCameraLowObstacleFilter : public BaseFilter
{
   public:
    DepthCameraLowObstacleFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    LayerVector updateImpl(LayerVector layer_vector) override;

    /**
     * @brief 스냅샷(map frame)에서 낮은 장애물 후보 점 추출.
     *        높이 band + 전방 코리도 크롭 후, radius 내 이웃 min_neighbors
     *        이상인 점만 남기고 (고립 노이즈 제거), lidar 가 보고 있는
     *        위치(lidar_check_radius 이내 리턴 존재)는 벽/일반 장애물로
     *        판단하여 후보에서 제외한다.
     */
    pcl::PointCloud<pcl::PointXYZ> extractCandidates(const pcl::PointCloud<pcl::PointXYZ>& snapshot_cloud);

    float min_height{};
    float max_height{};
    int32_t min_neighbors{};
    float radius{};
    float x_min{};
    float x_max{};
    float half_width{};
    int32_t confirm_frames{};
    float lidar_check_radius{};

    // ---- 프레임 간 유지되는 상태 ----
    struct PendingPoint
    {
        pcl::PointXYZ point;  // map frame
        int32_t hits;         // 연속 스냅샷 관측 횟수
    };
    std::vector<PendingPoint> pending{};  // 확정 대기 후보
    rclcpp::Time last_snapshot_stamp{};   // 마지막으로 처리한 스냅샷 timestamp
};

/**
 * @brief ObstacleLoggerFilter
 *        체인 "마지막"에 두는 관측 전용 필터. 레이어를 변경하지 않고,
 *        최종 생존(=실제 발행될) 점들의 요약을 1Hz 로 로깅한다.
 *        rviz2 디스플레이와 동기화된 디버깅용. (grep tag 는 yaml 의 tag 로 지정)
 *
 * [YAML 설정 예시]
 * --------------------------------------------------
 * obstacle_logger:
 *   tag: "DEPTH_OBS"    # 로그 grep 태그
 * --------------------------------------------------
 */
class ObstacleLoggerFilter : public BaseFilter
{
   public:
    ObstacleLoggerFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    LayerVector updateImpl(LayerVector layer_vector) override;

    std::string tag{};
};

/**
 * @brief OneDRoIFilter 클래스는 다각형 영역 내 또는 외부의 포인트를 필터링합니다.
 *
 * [YAML 설정 예시]
 * --------------------------------------------------
 * one_d_roi:
 *   isInside: true      # 관심 영역 내부 사용 여부
 *   polygon:            # 내부 관심 영역을 구성할 다각형 좌표 (각 좌표는 [x, y] 형태)
 *     [ [0.41, -0.5], [0.41, 0.5], [0.18,0.5], [0.18,-0.50]]
 * --------------------------------------------------
 */
class OneDRoIFilter : public RoIFilter
{
   public:
    OneDRoIFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);

   private:
    LayerVector updateImpl(LayerVector layer_vector) override;
    bool isInside(pcl::PointXY point);
    pcl::PointCloud<pcl::PointXY> polygon{};
};

}  // namespace A1::perception

#endif  // __FILTER_HPP__
