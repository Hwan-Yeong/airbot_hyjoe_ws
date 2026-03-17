#include "cloud_converter/sensors/tof_multi.hpp"
#include "sensor_manager_node.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"

namespace sensor_manager {

TofMultiLeftCloudConverter::TofMultiLeftCloudConverter(
    SensorManagerNode* node_ptr, const YAML::Node& config)
    : CloudConverterStrategy(node_ptr) {
  if (!config.IsMap()) {
    auto s = YAML::Dump(config);
    throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
  }

  // TODO: Set Default Config
  //       => Hard Coding for default value (e.g. yaml file is broken)

  // Load Config
  LoadCommonConfig(config);
  this->tof_multi_left_fov_ = DEG2RAD(config["extrinsics"]["fov"].as<double>());
  for (auto idx : config["sub_cell_idx_array"]) {
    this->tof_multi_left_sub_cell_idx_array_.push_back(idx.as<int>());
  }
  if (config["extrinsics"] && config["extrinsics"]["fov"]) {
    this->tof_multi_left_fov_ =
        DEG2RAD(config["extrinsics"]["fov"].as<double>());
  }
  if (config["sub_cell_idx_array"]) {
    this->tof_multi_left_sub_cell_idx_array_.clear();
    for (auto idx : config["sub_cell_idx_array"]) {
      this->tof_multi_left_sub_cell_idx_array_.push_back(idx.as<int>());
    }
  }

  // Print Config
  std::ostringstream oss_mtofLeft;
  oss_mtofLeft << GetCommonConfigInfo("MULTI TOF (Left)");
  oss_mtofLeft << "  fov [deg]            : "
               << RAD2DEG(this->tof_multi_left_fov_) << "\n";
  oss_mtofLeft << "  sub_cell_idx_array   : ";
  for (auto idx : this->tof_multi_left_sub_cell_idx_array_) {
    oss_mtofLeft << idx << " ";
  }
  oss_mtofLeft << "\n";
  oss_mtofLeft << "----------------------------------------------------";
  RCLCPP_INFO(this->node_ptr_->get_logger(), "%s",
              oss_mtofLeft.str().c_str());

  // Init Calculation
  tof_utils_.UpdateSubCellIndexArray(
      this->tof_multi_left_sub_cell_idx_array_, this->tof_multi_left_fov_,
      this->tof_multi_left_y_tan_array_,  // output
      this->tof_multi_left_z_tan_array_,  // output
      this->node_ptr_->get_logger());
}

ConverterOutput TofMultiLeftCloudConverter::PcConvertImpl(
    const void* sensor_msg) {
  ConverterOutput output;

  if (!this->use_converter_) return output;

  auto msg = static_cast<const robot_custom_msgs::msg::TofData*>(sensor_msg);

  std::vector<double> left_dists(msg->bot_left.begin(), msg->bot_left.end());

  // Create points on sensor frame
  std::vector<Point> points_on_sensor_frame =
      this->frame_converter_.TfMultiTofDistance2SensorFrame(
          left_dists, this->tof_multi_left_y_tan_array_,
          this->tof_multi_left_z_tan_array_);

  if (points_on_sensor_frame.empty()) return output;

  if (this->enable_sensor_tf_cloud_) {
    for (const auto& point : points_on_sensor_frame) {
      output.local_frame_clouds.push_back(
          this->pointcloud_generator_.GeneratePointCloud2Message(
              point, this->child_frame_));
    }
    output.local_topic_suffix = "/local";
  }

  // Create points on target frame
  if (this->enable_target_frame_cloud_) {
    Pose robot_pose;
    robot_pose.position.x = msg->robot_x;
    robot_pose.position.y = msg->robot_y;
    robot_pose.orientation.yaw = msg->robot_angle;

    std::vector<Point> points_on_robot_frame =
        this->frame_converter_.TfSensorFrame2RobotFrame(
            points_on_sensor_frame, this->sensor_extrinsic_);

    std::vector<Point> point_on_target_frame;
    if (this->target_frame_ == "base_link") {
      point_on_target_frame = points_on_robot_frame;
    } else if (this->target_frame_ == "map") {
      point_on_target_frame = this->frame_converter_.TfRobot2GlobalFrame(
          points_on_robot_frame, robot_pose);
    } else {
      RCLCPP_INFO(this->node_ptr_->get_logger(),
                  "Select Wrong Target Frame: %s", this->target_frame_.c_str());
    }

    for (const auto& point : point_on_target_frame) {
      output.target_frame_clouds.push_back(
          this->pointcloud_generator_.GeneratePointCloud2Message(
              point, this->target_frame_));
    }
  }

  return output;
}

std_msgs::msg::Float32MultiArray TofMultiLeftCloudConverter::CalibrationConvert(
    const void* sensor_msg) {
  std_msgs::msg::Float32MultiArray ret;

  auto msg = static_cast<const robot_custom_msgs::msg::TofData*>(sensor_msg);

  const size_t INDEX_SIZE = 16;

  std::vector<double> left_dists(msg->bot_left.begin(), msg->bot_left.end());
  std::vector<Point> robot_pts =
      this->frame_converter_.TfMultiTofSensor2RobotFrame(
          left_dists, this->tof_multi_left_y_tan_array_,
          this->tof_multi_left_z_tan_array_, this->sensor_extrinsic_);

  if (robot_pts.size() != INDEX_SIZE) {
    RCLCPP_WARN(this->node_ptr_->get_logger(),
                "Expected %zu robot points, but got %zu.", INDEX_SIZE,
                robot_pts.size());
  }

  size_t idx_num =
      3;  // Left 3 points ([13], [14], [15]) are calibrated, so set to 3
  if (robot_pts.size() >= idx_num) {
    const size_t n = robot_pts.size();
    ret.data.reserve(idx_num);
    for (size_t i = n - idx_num; i < n; ++i) {  // Access last 3 points
      ret.data.push_back(static_cast<float>(
          sqrt(robot_pts[i].x * robot_pts[i].x +
               robot_pts[i].y * robot_pts[i].y)));
    }
  } else {
    RCLCPP_WARN(
        this->node_ptr_->get_logger(),
        "robot_pts has fewer than 3 points (size=%zu). Returning empty array.",
        robot_pts.size());
  }

  return ret;
}

TofMultiRightCloudConverter::TofMultiRightCloudConverter(
    SensorManagerNode* node_ptr, const YAML::Node& config)
    : CloudConverterStrategy(node_ptr) {
  if (!config.IsMap()) {
    auto s = YAML::Dump(config);
    throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
  }

  // TODO: Set Default Config
  //       => Hard Coding for default value (e.g. yaml file is broken)

  // Load Config
  LoadCommonConfig(config);
  this->tof_multi_right_fov_ =
      DEG2RAD(config["extrinsics"]["fov"].as<double>());
  for (auto idx : config["sub_cell_idx_array"]) {
    this->tof_multi_right_sub_cell_idx_array_.push_back(idx.as<int>());
  }
  if (config["extrinsics"] && config["extrinsics"]["fov"]) {
    this->tof_multi_right_fov_ =
        DEG2RAD(config["extrinsics"]["fov"].as<double>());
  }
  if (config["sub_cell_idx_array"]) {
    this->tof_multi_right_sub_cell_idx_array_.clear();
    for (auto idx : config["sub_cell_idx_array"]) {
      this->tof_multi_right_sub_cell_idx_array_.push_back(idx.as<int>());
    }
  }

  // Print Config
  std::ostringstream oss_mtofRight;
  oss_mtofRight << GetCommonConfigInfo("MULTI TOF (Right)");
  oss_mtofRight << "  fov [deg]            : "
                << RAD2DEG(this->tof_multi_right_fov_) << "\n";
  oss_mtofRight << "  sub_cell_idx_array   : ";
  for (auto idx : this->tof_multi_right_sub_cell_idx_array_) {
    oss_mtofRight << idx << " ";
  }
  oss_mtofRight << "\n";
  oss_mtofRight << "----------------------------------------------------";
  RCLCPP_INFO(this->node_ptr_->get_logger(), "%s",
              oss_mtofRight.str().c_str());

  // Init Calculation
  tof_utils_.UpdateSubCellIndexArray(
      this->tof_multi_right_sub_cell_idx_array_, this->tof_multi_right_fov_,
      this->tof_multi_right_y_tan_array_,  // output
      this->tof_multi_right_z_tan_array_,  // output
      this->node_ptr_->get_logger());
}

ConverterOutput TofMultiRightCloudConverter::PcConvertImpl(
    const void* sensor_msg) {
  ConverterOutput output;

  if (!this->use_converter_) return output;

  auto msg = static_cast<const robot_custom_msgs::msg::TofData*>(sensor_msg);

  std::vector<double> right_dists(msg->bot_right.begin(), msg->bot_right.end());

  // Create points on sensor frame
  std::vector<Point> points_on_sensor_frame =
      this->frame_converter_.TfMultiTofDistance2SensorFrame(
          right_dists, this->tof_multi_right_y_tan_array_,
          this->tof_multi_right_z_tan_array_);

  if (points_on_sensor_frame.empty()) return output;

  if (this->enable_sensor_tf_cloud_) {
    for (const auto& point : points_on_sensor_frame) {
      output.local_frame_clouds.push_back(
          this->pointcloud_generator_.GeneratePointCloud2Message(
              point, this->child_frame_));
    }
    output.local_topic_suffix = "/local";
  }

  // Create points on target frame
  if (this->enable_target_frame_cloud_) {
    Pose robot_pose;
    robot_pose.position.x = msg->robot_x;
    robot_pose.position.y = msg->robot_y;
    robot_pose.orientation.yaw = msg->robot_angle;

    std::vector<Point> points_on_robot_frame =
        this->frame_converter_.TfSensorFrame2RobotFrame(
            points_on_sensor_frame, this->sensor_extrinsic_);

    std::vector<Point> point_on_target_frame;
    if (this->target_frame_ == "base_link") {
      point_on_target_frame = points_on_robot_frame;
    } else if (this->target_frame_ == "map") {
      point_on_target_frame = this->frame_converter_.TfRobot2GlobalFrame(
          points_on_robot_frame, robot_pose);
    } else {
      RCLCPP_INFO(this->node_ptr_->get_logger(),
                  "Select Wrong Target Frame: %s", this->target_frame_.c_str());
    }

    for (const auto& point : point_on_target_frame) {
      output.target_frame_clouds.push_back(
          this->pointcloud_generator_.GeneratePointCloud2Message(
              point, this->target_frame_));
    }
  }

  return output;
}

std_msgs::msg::Float32MultiArray
TofMultiRightCloudConverter::CalibrationConvert(const void* sensor_msg) {
  std_msgs::msg::Float32MultiArray ret;

  auto msg = static_cast<const robot_custom_msgs::msg::TofData*>(sensor_msg);

  const size_t INDEX_SIZE = 16;

  std::vector<double> right_dists(msg->bot_right.begin(), msg->bot_right.end());
  std::vector<Point> robot_pts =
      this->frame_converter_.TfMultiTofSensor2RobotFrame(
          right_dists, this->tof_multi_right_y_tan_array_,
          this->tof_multi_right_z_tan_array_, this->sensor_extrinsic_);

  if (robot_pts.size() != INDEX_SIZE) {
    RCLCPP_WARN(this->node_ptr_->get_logger(),
                "Expected %zu robot points, but got %zu.", INDEX_SIZE,
                robot_pts.size());
  }

  size_t idx_num =
      3;  // Right 3 points ([13], [14], [15]) are calibrated, so set to 3
  if (robot_pts.size() >= idx_num) {
    const size_t n = robot_pts.size();
    ret.data.reserve(idx_num);
    for (size_t i = n - idx_num; i < n; ++i) {  // Access last 3 points
      ret.data.push_back(static_cast<float>(
          sqrt(robot_pts[i].x * robot_pts[i].x +
               robot_pts[i].y * robot_pts[i].y)));
    }
  } else {
    RCLCPP_WARN(
        this->node_ptr_->get_logger(),
        "robot_pts has fewer than 3 points (size=%zu). Returning empty array.",
        robot_pts.size());
  }

  return ret;
}

}  // namespace sensor_manager