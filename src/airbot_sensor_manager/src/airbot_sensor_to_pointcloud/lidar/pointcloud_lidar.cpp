#include "airbot_sensor_to_pointcloud/lidar/pointcloud_lidar.hpp"

PointCloudLidar::PointCloudLidar()
{
}

PointCloudLidar::~PointCloudLidar()
{
}

/**
 * @brief ### 노드 초기화 시점에, 파라미터 업데이트
 */
void PointCloudLidar::updateParams(tLidarParam front, tLidarParam back)
{
    front_offset_x_ = front.offset.x;
    front_offset_y_ = front.offset.y;
    front_offset_z_ = front.offset.z;
    front_angle_max_ = front.angle_max;
    front_angle_min_ = front.angle_min;
    front_alpha_ = front.alpha;

    back_offset_x_ = back.offset.x;
    back_offset_y_ = back.offset.y;
    back_offset_z_ = back.offset.z;
    back_angle_max_ = back.angle_max;
    back_angle_min_ = back.angle_min;
    back_alpha_ = back.alpha;
}

sensor_msgs::msg::PointCloud2 PointCloudLidar::updateLidarPointCloudMsg(sensor_msgs::msg::LaserScan front_lidar_msg, sensor_msgs::msg::LaserScan back_lidar_msg)
{
    sensor_msgs::msg::PointCloud2 msg;

    // refresh_params();
    pcl::PointCloud<pcl::PointXYZ> cloud_; 
    std::vector<std::array<float, 2>> scan_data;
    float min_theta = std::numeric_limits<float>::max();
    float max_theta = std::numeric_limits<float>::lowest();

    auto process_laser = [&](const sensor_msgs::msg::LaserScan &laser, 
                            float x_off, float y_off, float z_off, float alpha, 
                            float angle_min, float angle_max) {
        if (laser.ranges.empty()) return;

        float temp_min = std::min(laser.angle_min, laser.angle_max);
        float temp_max = std::max(laser.angle_min, laser.angle_max);
        float alpha_rad = alpha * M_PI / 180.0;
        float cos_alpha = std::cos(alpha_rad);
        float sin_alpha = std::sin(alpha_rad);
        float angle_min_rad = angle_min * M_PI / 180.0;
        float angle_max_rad = angle_max * M_PI / 180.0;

        for (size_t i = 0; i < laser.ranges.size(); ++i) {
            float angle = temp_min + i * laser.angle_increment;
            if (angle > temp_max) break;

            float range = laser.ranges[i];

            if (std::isnan(range) || range < laser.range_min || range > laser.range_max) continue;

            bool is_in_range = (angle >= angle_min_rad && angle <= angle_max_rad);
            if (is_in_range == false) continue;

            pcl::PointXYZ pt; 
            float x = range * std::cos(angle);
            float y = range * std::sin(angle);

            pt.x = x * cos_alpha - y * sin_alpha + x_off;
            pt.y = x * sin_alpha + y * cos_alpha + y_off;
            pt.z = z_off;

            cloud_.points.push_back(pt);

            float r_ = std::hypot(pt.x, pt.y);
            float theta_ = std::atan2(pt.y, pt.x);
            scan_data.push_back({theta_, r_});

            min_theta = std::min(min_theta, theta_);
            max_theta = std::max(max_theta, theta_);
        }
    };

    // Process both lasers
    process_laser(front_lidar_msg, front_offset_x_, front_offset_y_, front_offset_z_, front_alpha_, 
                front_angle_min_, front_angle_max_);

    process_laser(back_lidar_msg, back_offset_x_, back_offset_y_, back_offset_z_, back_alpha_, 
                back_angle_min_, back_angle_max_);

    // Create and publish PointCloud2 message
    removePointsWithinRadius(cloud_, 0.2); //radius 0.19
    pcl::toROSMsg(cloud_, msg);
    msg.header.frame_id = "base_scan";

    rclcpp::Time time1(front_lidar_msg.header.stamp);
    rclcpp::Time time2(back_lidar_msg.header.stamp);
    msg.header.stamp = (time1 < time2) ? back_lidar_msg.header.stamp : front_lidar_msg.header.stamp;
    msg.is_dense = false;

    return msg;
}

void PointCloudLidar::removePointsWithinRadius(pcl::PointCloud<pcl::PointXYZ>& cloud, float radius, float center_x, float center_y)
{
    pcl::PointCloud<pcl::PointXYZ> filteredCloud;

    for (const auto& point : cloud.points) {
        float distance = std::sqrt((point.x - center_x) * (point.x - center_x) + 
                                    (point.y - center_y) * (point.y - center_y));
        if (distance >= radius) {
            filteredCloud.points.push_back(point);
        }
    }

    cloud.points.swap(filteredCloud.points);
}