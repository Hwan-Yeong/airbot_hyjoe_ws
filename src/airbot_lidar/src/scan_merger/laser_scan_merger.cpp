#include "scan_merger/laser_scan_merger.hpp"

scanMerger::scanMerger() : Node("airbot_scan_merger")
{
    initialize_params();
    refresh_params();

    laser1_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, topic1_);
    laser2_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, topic2_);
  
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *laser1_sub_, *laser2_sub_);
    sync_->registerCallback(std::bind(&scanMerger::synchronized_callback, this, std::placeholders::_1, std::placeholders::_2));

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloudTopic_, rclcpp::SensorDataQoS().best_effort());
}

scanMerger::~scanMerger()
{
}

void scanMerger::initialize_params()
{
    this->declare_parameter("pointCloudTopic", "/scan_merged_pointcloud");
    this->declare_parameter("pointCloutFrameId", "base_scan");

    this->declare_parameter("scan_front.topic", "/scan_front");
    this->declare_parameter("scan_front.range.angle_min", 60.0);
    this->declare_parameter("scan_front.range.angle_max", 300.0);
    this->declare_parameter("scan_front.geometry.flip", false);
    this->declare_parameter("scan_front.geometry.offset.x", 0.15);
    this->declare_parameter("scan_front.geometry.offset.y", 0.0);
    this->declare_parameter("scan_front.geometry.offset.z", 0.0);
    this->declare_parameter("scan_front.geometry.offset.alpha", 180.0);

    this->declare_parameter("scan_back.topic", "/scan_back");
    this->declare_parameter("scan_back.range.angle_min", 60.0);
    this->declare_parameter("scan_back.range.angle_max", 300.0);
    this->declare_parameter("scan_back.geometry.flip", false);
    this->declare_parameter("scan_back.geometry.offset.x", -0.15);
    this->declare_parameter("scan_back.geometry.offset.y", 0.0);
    this->declare_parameter("scan_back.geometry.offset.z", 0.0);
    this->declare_parameter("scan_back.geometry.offset.alpha", 0.0);
}

void scanMerger::refresh_params()
{
    this->get_parameter_or<std::string>("pointCloudTopic", cloudTopic_, "/scan_merged_pointcloud");
    this->get_parameter_or<std::string>("pointCloutFrameId", cloudFrameId_, "base_scan");

    this->get_parameter_or<std::string>("scan_front.topic", topic1_, "/scan_front");
    this->get_parameter_or<float>("scan_front.range.angle_min", laser1AngleMin_, 60.0);
    this->get_parameter_or<float>("scan_front.range.angle_max", laser1AngleMax_, 300.0);
    this->get_parameter_or<bool>("scan_front.geometry.flip", flip1_, false);
    this->get_parameter_or<float>("scan_front.geometry.offset.x", laser1XOff_, 0.15);
    this->get_parameter_or<float>("scan_front.geometry.offset.y", laser1YOff_, 0.0);
    this->get_parameter_or<float>("s", laser1ZOff_, 0.0);
    this->get_parameter_or<float>("scan_front.geometry.offset.alpha", laser1Alpha_, 180.0);

    this->get_parameter_or<std::string>("scan_back.topic", topic2_, "/scan_back");
    this->get_parameter_or<float>("scan_back.range.angle_min", laser2AngleMin_, 60.0);
    this->get_parameter_or<float>("scan_back.range.angle_max", laser2AngleMax_, 300.0);
    this->get_parameter_or<bool>("scan_back.geometry.flip", flip2_, false);
    this->get_parameter_or<float>("scan_back.geometry.offset.x", laser2XOff_, -0.15);
    this->get_parameter_or<float>("scan_back.geometry.offset.y", laser2YOff_, 0.0);
    this->get_parameter_or<float>("scan_back.geometry.offset.z", laser2ZOff_, 0.0);
    this->get_parameter_or<float>("scan_back.geometry.offset.alpha", laser2Alpha_, 0.0);
}

void scanMerger::synchronized_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser1_msg, const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser2_msg)
{
    laser1_ = *laser1_msg;
    laser2_ = *laser2_msg;
    update_point_cloud_rgb();
}

void scanMerger::update_point_cloud_rgb()
{
    pcl::PointCloud<pcl::PointXYZ> cloud_; 
    std::vector<std::array<float, 2>> scan_data;
    float min_theta = std::numeric_limits<float>::max();
    float max_theta = std::numeric_limits<float>::lowest();

    auto process_laser = [&](const sensor_msgs::msg::LaserScan &laser, 
                            float x_off, float y_off, float z_off, float alpha, 
                            float angle_min, float angle_max, bool flip) {
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

            size_t idx = flip ? laser.ranges.size() - 1 - i : i;
            float range = laser.ranges[idx];

            if (std::isnan(range) || range < laser.range_min || range > laser.range_max) continue;

            bool is_in_range = (angle >= angle_min_rad && angle <= angle_max_rad);
            if (!is_in_range) continue;

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

    process_laser(laser1_, laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, 
                laser1AngleMin_, laser1AngleMax_, flip1_);

    process_laser(laser2_, laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, 
                laser2AngleMin_, laser2AngleMax_, flip2_);
    
    removePointsWithinRadius(cloud_, 0.2); //radius 0.19
    auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud_, *pc2_msg);
    pc2_msg->header.frame_id = cloudFrameId_;
    pc2_msg->is_dense = false;
    rclcpp::Time time1(laser1_.header.stamp);
    rclcpp::Time time2(laser2_.header.stamp);
    pc2_msg->header.stamp = (time1 < time2) ? laser2_.header.stamp : laser1_.header.stamp;

    point_cloud_pub_->publish(*pc2_msg);
}

void scanMerger::removePointsWithinRadius(pcl::PointCloud<pcl::PointXYZ>& cloud, float radius)
{
    pcl::PointCloud<pcl::PointXYZ> filteredCloud;

    for (const auto& point : cloud.points) {
        float distance = std::sqrt(point.x*point.x + point.y*point.y);
        if (distance >= radius) {
            filteredCloud.points.push_back(point);
        }
    }
    cloud.points.swap(filteredCloud.points);
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scanMerger>());
  rclcpp::shutdown();
  return 0;
}