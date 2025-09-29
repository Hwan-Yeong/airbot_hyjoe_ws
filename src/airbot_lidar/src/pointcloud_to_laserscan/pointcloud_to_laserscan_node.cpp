/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode()
: rclcpp::Node("airbot_pointcloud_to_laserscan")
{
  target_frame_ = this->declare_parameter("target_frame", "");
  pointcloud_topic_ = this->declare_parameter("pointcloud_topic", "");
  laserscan_topic_ = this->declare_parameter("laserscan_topic", "");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);

  // ####################################################################################
  active_forced_escape_ = false;

  float roi_robot_radius;
  float roi_heigh_size;
  float roi_sector_front_min_deg;
  float roi_sector_front_max_deg;
  float roi_sector_back_min_deg;
  float roi_sector_back_max_deg;
  float roi_sector_resolution_deg;

  // 각도 min, max 기준은 반시계방향
  this->declare_parameter("forced_escape_scan_roi.robot_radius", 0.19);
  this->declare_parameter("forced_escape_scan_roi.roi.height", 3.0);
  this->declare_parameter("forced_escape_scan_roi.roi.sector_front_min_deg", -45.0);
  this->declare_parameter("forced_escape_scan_roi.roi.sector_front_max_deg", 45.0);
  this->declare_parameter("forced_escape_scan_roi.roi.sector_back_min_deg", 135.0);
  this->declare_parameter("forced_escape_scan_roi.roi.sector_back_max_deg", -135.0);
  this->declare_parameter("forced_escape_scan_roi.roi.sector_resolution", 10.0);
  this->declare_parameter("forced_escape_scan_roi.score_weight.angle", 1.0);
  this->declare_parameter("forced_escape_scan_roi.score_weight.distance", 1.0);

  this->get_parameter("forced_escape_scan_roi.robot_radius", roi_robot_radius);
  this->get_parameter("forced_escape_scan_roi.roi.height",roi_heigh_size);
  this->get_parameter("forced_escape_scan_roi.roi.sector_front_min_deg",roi_sector_front_min_deg);
  this->get_parameter("forced_escape_scan_roi.roi.sector_front_max_deg",roi_sector_front_max_deg);
  this->get_parameter("forced_escape_scan_roi.roi.sector_back_min_deg",roi_sector_back_min_deg);
  this->get_parameter("forced_escape_scan_roi.roi.sector_back_max_deg",roi_sector_back_max_deg);
  this->get_parameter("forced_escape_scan_roi.roi.sector_resolution",roi_sector_resolution_deg);
  this->get_parameter("forced_escape_scan_roi.score_weight.angle",forced_escape_scan_roi.angle_weight);
  this->get_parameter("forced_escape_scan_roi.score_weight.distance",forced_escape_scan_roi.distance_weight);

  RCLCPP_INFO(this->get_logger(), "=== Forced Escape ROI Parameters ===");
  RCLCPP_INFO(this->get_logger(), "  robot_radius: %f", roi_robot_radius);
  RCLCPP_INFO(this->get_logger(), "  roi_height: %f", roi_heigh_size);
  RCLCPP_INFO(this->get_logger(), "  sector_front_min_deg: %f", roi_sector_front_min_deg);
  RCLCPP_INFO(this->get_logger(), "  sector_front_max_deg: %f", roi_sector_front_max_deg);
  RCLCPP_INFO(this->get_logger(), "  sector_back_min_deg: %f", roi_sector_back_min_deg);
  RCLCPP_INFO(this->get_logger(), "  sector_back_max_deg: %f", roi_sector_back_max_deg);
  RCLCPP_INFO(this->get_logger(), "  sector_resolution: %f", roi_sector_resolution_deg);
  RCLCPP_INFO(this->get_logger(), "  angle_weight: %f", forced_escape_scan_roi.angle_weight);
  RCLCPP_INFO(this->get_logger(), "  distance_weight: %f", forced_escape_scan_roi.distance_weight);

  forced_escape_scan_roi.roi_x_min = roi_robot_radius;
  forced_escape_scan_roi.roi_x_max = roi_heigh_size + forced_escape_scan_roi.roi_x_min;
  forced_escape_scan_roi.roi_y_min = -roi_robot_radius;
  forced_escape_scan_roi.roi_y_max = roi_robot_radius;
  forced_escape_scan_roi.sector_front_min_rad = roi_sector_front_min_deg * M_PI / 180.0;
  forced_escape_scan_roi.sector_front_max_rad = roi_sector_front_max_deg * M_PI / 180.0;
  forced_escape_scan_roi.sector_back_min_rad = roi_sector_back_min_deg * M_PI / 180.0;
  forced_escape_scan_roi.sector_back_max_rad = roi_sector_back_max_deg * M_PI / 180.0;
  forced_escape_scan_roi.sector_resolution = roi_sector_resolution_deg * M_PI / 180.0;

  roi_sector_front_count = static_cast<int>(std::round((roi_sector_front_max_deg - roi_sector_front_min_deg) / roi_sector_resolution_deg));
  float sector_range_deg = roi_sector_back_max_deg - roi_sector_back_min_deg;
  if (sector_range_deg < 0) sector_range_deg += 360.0;
  roi_sector_back_count = static_cast<int>(sector_range_deg / roi_sector_resolution_deg);

  forced_escape_heading_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("forced_escape_heading", 10);
#if DEBUG_MODE
  // debug
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("debug/scan_roi_result", 10);
  // ####################################################################################
#endif

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(laserscan_topic_, rclcpp::SensorDataQoS());

  using std::placeholders::_1;
  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  } else {  // otherwise setup direct subscription
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  }

  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));

  forced_escape_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/maneuver/request/escape", 10,
    [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data == true) active_forced_escape_ = true;
        else active_forced_escape_ = false;
    }
  );

  RCLCPP_INFO(this->get_logger(), "node initialized");
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);
  subscription_listener_thread_.join();
  RCLCPP_INFO(this->get_logger(), "node terminated");
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count = pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count();
    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Got a subscriber to laserscan, starting pointcloud subscriber");
        rclcpp::SensorDataQoS qos;
        qos.keep_last(input_queue_size_);
        sub_.subscribe(this, pointcloud_topic_, qos.get_rmw_qos_profile());
      }
    } else if (sub_.getSubscriber()) {
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to laserscan, shutting down pointcloud subscriber");
      sub_.unsubscribe();
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // Transform cloud if necessary
  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (*iter_z > max_height_ || *iter_z < min_height_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for height %f not in range (%f, %f)\n",
        *iter_z, min_height_, max_height_);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for angle %f not in range (%f, %f)\n",
        angle, scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
    }
  }
  if (active_forced_escape_) {
    findForcedEscapeTargetPoint(scan_msg);
  }

  pub_->publish(std::move(scan_msg));
}

void PointCloudToLaserScanNode::findForcedEscapeTargetPoint(const std::unique_ptr<sensor_msgs::msg::LaserScan> &scan_msg)
{
  // front ROI (-45 ~ 45)
  std::vector<std::pair<float, float>> front_sector_selected_points(roi_sector_front_count, {0.0f, 0.0f});
  // back ROI (135 ~ -135)
  std::vector<std::pair<float, float>> back_sector_selected_points(roi_sector_back_count, {0.0f, 0.0f});

  float angle_rad = scan_msg->angle_min;

  for (size_t i = 0; i < scan_msg->ranges.size(); i++, angle_rad += scan_msg->angle_increment) {
    float dist = scan_msg->ranges[i];
    if (std::isnan(dist)) continue;
    if (std::isinf(dist)) continue/*dist = forced_escape_scan_roi.roi_x_max*/;

    // --------------------- front ROI ---------------------
    if (angle_rad >= forced_escape_scan_roi.sector_front_min_rad &&
        angle_rad <= forced_escape_scan_roi.sector_front_max_rad) {

      float x = dist * std::cos(angle_rad);
      float y = dist * std::sin(angle_rad);

      if (x >= forced_escape_scan_roi.roi_x_min && x <= forced_escape_scan_roi.roi_x_max &&
          y >= forced_escape_scan_roi.roi_y_min && y <= forced_escape_scan_roi.roi_y_max) {

        int idx = static_cast<int>(
          (angle_rad - forced_escape_scan_roi.sector_front_min_rad) /
          forced_escape_scan_roi.sector_resolution);

        if (idx >= 0 && idx < roi_sector_front_count) {
          if (dist > front_sector_selected_points[idx].first) {
            front_sector_selected_points[idx] = {dist, angle_rad};
          }
        }
      }
    }

    // --------------------- back ROI ---------------------
    if (angle_rad >= forced_escape_scan_roi.sector_back_min_rad ||
        angle_rad <= forced_escape_scan_roi.sector_back_max_rad) {

      float x = dist * std::cos(angle_rad);
      float y = dist * std::sin(angle_rad);

      if (x <= -forced_escape_scan_roi.roi_x_min && x >= -forced_escape_scan_roi.roi_x_max &&
          y >= forced_escape_scan_roi.roi_y_min && y <= forced_escape_scan_roi.roi_y_max) {
        int idx = static_cast<int>(
          (angle_rad - forced_escape_scan_roi.sector_back_min_rad) /
          forced_escape_scan_roi.sector_resolution);

        if (idx >= 0 && idx < roi_sector_back_count) {
          if (dist > back_sector_selected_points[idx].first) {
            back_sector_selected_points[idx] = {dist, angle_rad};
          }
        }
      }
    }
  }

  // ============== best selection for front ROI =============
  float front_best_score = 10;
  float front_best_range = 0.001; // default: max range of roi
  float front_best_angle = 0.0f; // default: robot heading forward

  for (auto &point : front_sector_selected_points) {
    float selected_dist = point.first;
    float selected_angle = point.second;
    if (selected_dist <= 0.0f) continue;

    // Note: if the ROI angular range changes, this constant must also be updated.
    float angle_norm = std::fabs(selected_angle) * 1.2732395447351628; // 1/(M_PI/4) = 4/π
    float distance_norm = selected_dist / forced_escape_scan_roi.roi_x_max;
    float score = forced_escape_scan_roi.angle_weight * angle_norm + forced_escape_scan_roi.distance_weight * (1.0f - distance_norm);

    if (score < front_best_score) {
      front_best_score = score;
      front_best_range = selected_dist;
      front_best_angle = selected_angle;
    }
  }

  // ============== best selection for back ROI ==============
  float back_best_score = 10;
  float back_best_range = 0.001;
  float back_best_angle = M_PI; // default: robot heading backward

  for (auto &point : back_sector_selected_points) {
    float selected_dist = point.first;
    float selected_angle = point.second;
    if (selected_dist <= 0.0f) continue;

    // Note: if the ROI angular range changes, this constant must also be updated.
    float angle_norm = std::fabs(selected_angle - M_PI) * 1.2732395447351628; // 1/(M_PI/4) = 4/π
    float distance_norm = selected_dist / forced_escape_scan_roi.roi_x_max;
    float score = forced_escape_scan_roi.angle_weight * angle_norm +
                  forced_escape_scan_roi.distance_weight * (1.0f - distance_norm);

    if (score < back_best_score) {
      back_best_score = score;
      back_best_range = selected_dist;
      back_best_angle = selected_angle;
    }
  }

  // publish result
  publishForcedEscapeHeadingMsg(front_best_angle, front_best_range, back_best_angle, back_best_range);
#if DEBUG_MODE
  // #######  publish result (debug)  #######
  float f_x = front_best_range * std::cos(front_best_angle);
  float f_y = front_best_range * std::sin(front_best_angle);
  float b_x = back_best_range * std::cos(back_best_angle);
  float b_y = back_best_range * std::sin(back_best_angle);
  publishDebugPoint(f_x, f_y, 1);
  publishDebugPoint(b_x, b_y, -1);
  // ########################################
#endif
}

void PointCloudToLaserScanNode::publishForcedEscapeHeadingMsg(float f_angle, float f_dist, float b_angle, float b_dist)
{
  std_msgs::msg::Float32MultiArray feheading_msg;
  feheading_msg.data = {f_angle, f_dist, b_angle, b_dist};
  forced_escape_heading_pub_->publish(feheading_msg);
}
#if DEBUG_MODE
// debug
void PointCloudToLaserScanNode::publishDebugPoint(float x, float y, float z)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_scan";
  marker.header.stamp = this->now();
  marker.ns = "debug";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.15;

  if (z > 0) {
    marker.color.a = 0.9;
    marker.color.r = 0.8;
    marker.color.g = 0.2;
    marker.color.b = 1.0;
  } else if (z < 0) {
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }

  marker_pub_->publish(marker);
}
#endif
}  // namespace pointcloud_to_laserscan

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pointcloud_to_laserscan::PointCloudToLaserScanNode>());
  rclcpp::shutdown();
  return 0;
}
