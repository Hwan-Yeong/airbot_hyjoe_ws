#include "cloud_converter/sensors/bottom_ir.hpp"
#include "sensor_manager_node.hpp"
#include "robot_custom_msgs/msg/bottom_ir_data.hpp"

namespace sensor_manager {

BottomIrCloudConverter::BottomIrCloudConverter(
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
  if (config["extrinsics"]) {
    this->ir_dist_center_to_sensor_ =
        config["extrinsics"]["distance"]
            ? config["extrinsics"]["distance"].as<double>()
            : 0.15;
    this->ir_angle_sensor_to_next_sensor_ =
        config["extrinsics"]["angle"]
            ? config["extrinsics"]["angle"].as<double>()
            : 50.0;
  }

  // Print Config
  std::ostringstream oss_bottomIr;
  oss_bottomIr << GetCommonConfigInfo("BOTTOM IR");
  oss_bottomIr << "  ir_dist_center_to_sensor_ [m]          : "
               << this->ir_dist_center_to_sensor_ << "\n";
  oss_bottomIr << "  ir_angle_sensor_to_next_sensor_ [deg]  : "
               << this->ir_angle_sensor_to_next_sensor_ << " \n";
  oss_bottomIr << "----------------------------------------------------";
  RCLCPP_INFO(this->node_ptr_->get_logger(), "%s", oss_bottomIr.str().c_str());
}

ConverterOutput BottomIrCloudConverter::PcConvertImpl(const void* sensor_msg) {
  ConverterOutput output;

  if (!this->use_converter_) return output;

  auto msg =
      static_cast<const robot_custom_msgs::msg::BottomIrData*>(sensor_msg);

  // Create points on sensor frame
  std::vector<Point> points_on_sensor_frame =
      this->frame_converter_.TfBottomIrSensor2SensorFrame(
          msg, this->ir_dist_center_to_sensor_,
          this->ir_angle_sensor_to_next_sensor_);

  if (points_on_sensor_frame.empty()) return output;

  if (this->enable_sensor_tf_cloud_) {
    output.local_frame_clouds.push_back(
        this->pointcloud_generator_.GeneratePointCloud2Message(
            points_on_sensor_frame, this->child_frame_));
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

    std::vector<Point> points_on_target_frame;
    if (this->target_frame_ == "map") {
      points_on_target_frame = this->frame_converter_.TfRobot2GlobalFrame(
          points_on_robot_frame, robot_pose);
    } else {
      points_on_target_frame = points_on_robot_frame;
    }

    output.target_frame_clouds.push_back(
        this->pointcloud_generator_.GeneratePointCloud2Message(
            points_on_target_frame, this->target_frame_));
  }

  return output;
}

}  // namespace sensor_manager