#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/time.hpp"

#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include <array>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;
// using SyncPolicy = sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>;
using SyncPolicy = sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>;
// typedef sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> SyncPolicy;


class scanMerger : public rclcpp::Node
{
public:
  scanMerger() : Node("airbot_lidar_merger")
    {
        initialize_params();
        refresh_params();

        laser1_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, topic1_);
        laser2_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, topic2_);
        
       
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *laser1_sub_, *laser2_sub_);
        sync_->registerCallback(std::bind(&scanMerger::synchronized_callback, this, std::placeholders::_1, std::placeholders::_2));

        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloudTopic_, rclcpp::SensorDataQoS().best_effort());

        // Multithreaded Executor for better performance
        // executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        // executor_->add_node(this->shared_from_this());
        // executor_thread_ = std::thread([&]() { executor_->spin(); });
    }

    ~scanMerger()
    {
    }

private:
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser1_sub_, laser2_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;
  // typedef sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> SyncPolicy;

  void synchronized_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser1_msg, const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser2_msg)
  {
      laser1_ = *laser1_msg;
      laser2_ = *laser2_msg;
      update_point_cloud_rgb();
  }

  void update_point_cloud_rgb()
  {
      // refresh_params();
      pcl::PointCloud<pcl::PointXYZ> cloud_; 
      std::vector<std::array<float, 2>> scan_data;
      float min_theta = std::numeric_limits<float>::max();
      float max_theta = std::numeric_limits<float>::lowest();

      auto process_laser = [&](const sensor_msgs::msg::LaserScan &laser, 
                                float x_off, float y_off, float z_off, float alpha, 
                                float angle_min, float angle_max, 
                                bool flip, bool inverse, bool show) {
          if (!show || laser.ranges.empty()) return;

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
              if (inverse == is_in_range) continue;

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
      process_laser(laser1_, laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, 
                    laser1AngleMin_, laser1AngleMax_, flip1_, inverse1_, show1_);

      process_laser(laser2_, laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, 
                    laser2AngleMin_, laser2AngleMax_, flip2_, inverse2_, show2_);

      // Create and publish PointCloud2 message
//      RCLCPP_INFO(this->get_logger(),"No significant movement detected forseconds. Stopping exploration.");
      removePointsWithinRadius(cloud_, 0.2); //radius 0.19
      auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(cloud_, *pc2_msg);
      pc2_msg->header.frame_id = cloudFrameId_;

      rclcpp::Time time1(laser1_.header.stamp);
      rclcpp::Time time2(laser2_.header.stamp);
      pc2_msg->header.stamp = (time1 < time2) ? laser2_.header.stamp : laser1_.header.stamp;
      pc2_msg->is_dense = false;

      point_cloud_pub_->publish(*pc2_msg);
  }

  float GET_R(float x, float y)
  {
    return sqrt(x * x + y * y);
  }
  float GET_THETA(float x, float y)
  {
    float temp_res;
    if ((x != 0))
    {
      temp_res = atan(y / x);
    }
    else
    {
      if (y >= 0)
      {
        temp_res = M_PI / 2;
      }
      else
      {
        temp_res = -M_PI / 2;
      }
    }
    if (temp_res > 0)
    {
      if (y < 0)
      {
        temp_res -= M_PI;
      }
    }
    else if (temp_res < 0)
    {
      if (x < 0)
      {
        temp_res += M_PI;
      }
    }
    // RCLCPP_INFO(this->get_logger(), "x: '%f', y: '%f', a: '%f'", x, y, temp_res);

    return temp_res;
  }
  float interpolate(float angle_1, float angle_2, float magnitude_1, float magnitude_2, float current_angle)
  {
    return (magnitude_1 + current_angle * ((magnitude_2 - magnitude_1) / (angle_2 - angle_1)));
  }
  void initialize_params()
  {
    this->declare_parameter("pointCloudTopic", "cloud_in");
    this->declare_parameter("pointCloutFrameId", "base_scan");

    this->declare_parameter("scanTopic1", "/scan_front");
    this->declare_parameter("laser1XOff", 0.15);
    this->declare_parameter("laser1YOff", 0.0);
    this->declare_parameter("laser1ZOff", 0.0);
    this->declare_parameter("laser1Alpha", 180.0);
    this->declare_parameter("laser1AngleMin", 90.0);
    this->declare_parameter("laser1AngleMax", 270.0);
    this->declare_parameter("laser1R", 255);
    this->declare_parameter("laser1G", 0);
    this->declare_parameter("laser1B", 0);
    this->declare_parameter("show1", true);
    this->declare_parameter("flip1", false);
    this->declare_parameter("inverse1", false);

    this->declare_parameter("scanTopic2", "/scan_back");
    this->declare_parameter("laser2XOff", -0.15);
    this->declare_parameter("laser2YOff", 0.0);
    this->declare_parameter("laser2ZOff", 0.0);
    this->declare_parameter("laser2Alpha", 0.0);
    this->declare_parameter("laser2AngleMin", 90.0);
    this->declare_parameter("laser2AngleMax", 270.0);
    this->declare_parameter("laser2R", 0);
    this->declare_parameter("laser2G", 0);
    this->declare_parameter("laser2B", 255);
    this->declare_parameter("show2", true);
    this->declare_parameter("flip2", false);
    this->declare_parameter("inverse2", false);
  }
  void refresh_params()
  {
    this->get_parameter_or<std::string>("pointCloudTopic", cloudTopic_, "cloud_in");
    this->get_parameter_or<std::string>("pointCloutFrameId", cloudFrameId_, "base_scan");
    this->get_parameter_or<std::string>("scanTopic1", topic1_, "/scan_front");
    this->get_parameter_or<float>("laser1XOff", laser1XOff_, 0.15);
    this->get_parameter_or<float>("laser1YOff", laser1YOff_, 0.0);
    this->get_parameter_or<float>("laser1ZOff", laser1ZOff_, 0.0);
    this->get_parameter_or<float>("laser1Alpha", laser1Alpha_, 180.0);
    this->get_parameter_or<float>("laser1AngleMin", laser1AngleMin_, 90.0);
    this->get_parameter_or<float>("laser1AngleMax", laser1AngleMax_, 270.0);
    this->get_parameter_or<uint8_t>("laser1R", laser1R_, 255);
    this->get_parameter_or<uint8_t>("laser1G", laser1G_, 0);
    this->get_parameter_or<uint8_t>("laser1B", laser1B_, 0);
    this->get_parameter_or<bool>("show1", show1_, true);
    this->get_parameter_or<bool>("flip1", flip1_, false);
    this->get_parameter_or<bool>("inverse1", inverse1_, false);
    this->get_parameter_or<std::string>("scanTopic2", topic2_, "/scan_back");
    this->get_parameter_or<float>("laser2XOff", laser2XOff_, -0.15);
    this->get_parameter_or<float>("laser2YOff", laser2YOff_, 0.0);
    this->get_parameter_or<float>("laser2ZOff", laser2ZOff_, 0.0);
    this->get_parameter_or<float>("laser2Alpha", laser2Alpha_, 0.0);
    this->get_parameter_or<float>("laser2AngleMin", laser2AngleMin_, 90.0);
    this->get_parameter_or<float>("laser2AngleMax", laser2AngleMax_, 270.0);
    this->get_parameter_or<uint8_t>("laser2R", laser2R_, 0);
    this->get_parameter_or<uint8_t>("laser2G", laser2G_, 0);
    this->get_parameter_or<uint8_t>("laser2B", laser2B_, 255);
    this->get_parameter_or<bool>("show2", show2_, true);
    this->get_parameter_or<bool>("flip2", flip2_, false);
    this->get_parameter_or<bool>("inverse2", inverse2_, false);
  }

  void removePointsWithinRadius(pcl::PointCloud<pcl::PointXYZ>& cloud, float radius, float center_x = 0.0f, float center_y = 0.0f) {
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

  std::string topic1_, topic2_, cloudTopic_, cloudFrameId_;
  bool show1_, show2_, flip1_, flip2_, inverse1_, inverse2_;
  float laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, laser1AngleMin_, laser1AngleMax_;
  uint8_t laser1R_, laser1G_, laser1B_;

  float laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, laser2AngleMin_, laser2AngleMax_;
  uint8_t laser2R_, laser2G_, laser2B_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  sensor_msgs::msg::LaserScan laser1_;
  sensor_msgs::msg::LaserScan laser2_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scanMerger>());
  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(std::make_shared<scanMerger>());
  // executor.spin();
  // auto node = std::make_shared<scanMerger>();
  rclcpp::shutdown();
  return 0;
}
