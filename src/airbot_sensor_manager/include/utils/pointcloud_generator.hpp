#ifndef __POINTCLOUD_GENERATOR__
#define __POINTCLOUD_GENERATOR__

#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "utils/common_struct.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace sensor_manager {

class PointCloudGenerator
{
public:
    PointCloudGenerator();
    ~PointCloudGenerator();

    /**
     * @brief PointCloud2 데이터 여러개를 (vector<PC2>) 하나의 메시지로 통합하는 함수
     * 
     * @param[in] pc_msgs PointCloud2 메시지의 벡터
     * @param[in] frame target_frame <string>
     * @return 합쳐진 결과 sensor_msgs::msg::PointCloud2 메시지
     */
    sensor_msgs::msg::PointCloud2 mergePointCloud2Vector(const std::vector<sensor_msgs::msg::PointCloud2>& pc_msgs, std::string frame);

    /**
     * @brief "일반적인 센서" 위치 데이터로부터 PointCloud2 데이터를 생성하는 함수
     * 
     * @param[in] points PointCloud2로 변환하고자 하는 위치 데이터의 벡터
     * @param[in] frame target_frame <string>
     * @return 결과 sensor_msgs::msg::PointCloud2 메시지
     */
    sensor_msgs::msg::PointCloud2 generatePointCloud2Message(const std::vector<tPoint> &points, std::string frame);
    sensor_msgs::msg::PointCloud2 generatePointCloud2Message(const tPoint &point, std::string frame);

    /**
     * @brief "카메라 센서" 위치 데이터로부터 PointCloud2 데이터를 생성하는 함수
     * 
     * @param[in] input_bbox_array PointCloud2로 변환하고자 하는 bounding box의 array 집합
     * @param[in] resolution PointCloud2 데이터의 분해능
     * @param[in] frame target_frame <string>
     * @return 결과 sensor_msgs::msg::PointCloud2 메시지
     */
    sensor_msgs::msg::PointCloud2 generateCameraPointCloud2Message(const vision_msgs::msg::BoundingBox2DArray input_bbox_array, float resolution, std::string frame);

    /**
     * @brief "비어있는" PointCloud2 데이터를 생성하는 함수 (초기화 및 clear 용도)
     * 
     * @param[in] frame: target_frame <string>
     * @return 비어있는 sensor_msgs::msg::PointCloud2 메시지
     */
    sensor_msgs::msg::PointCloud2 generateEmptyPointCloud2Message(const std::string &frame);
private:
};

} // namespace sensor_manager

#endif // POINTCLOUD_GENERATOR