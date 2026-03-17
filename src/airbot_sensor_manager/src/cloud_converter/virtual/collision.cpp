#include "cloud_converter/virtual/collision.hpp"
#include "sensor_manager_node.hpp"
#include "robot_custom_msgs/msg/abnormal_event_data.hpp"

namespace sensor_manager {

CollisionCloudConverter::CollisionCloudConverter(
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
    this->sensor_extrinsic_.position =
        Point(config["extrinsics"]["distance"].as<double>(), 0.0, 0.0);
  }

  // Print Config
  std::ostringstream oss_collision;
  oss_collision << GetCommonConfigInfo("COLLISION");
  oss_collision << "----------------------------------------------------";
  RCLCPP_INFO(this->node_ptr_->get_logger(), "%s", oss_collision.str().c_str());
}

ConverterOutput CollisionCloudConverter::PcConvertImpl(const void* sensor_msg) {
  ConverterOutput output;

  if (!this->use_converter_) return output;

  auto msg =
      static_cast<const robot_custom_msgs::msg::AbnormalEventData*>(sensor_msg);

  // Create points on sensor frame
  Point point_on_sensor_frame =
      this->frame_converter_.TfCollisionData2SensorFrame(
          msg, this->sensor_extrinsic_.position.x);

  if (this->enable_sensor_tf_cloud_) {
    output.local_frame_clouds.push_back(
        this->pointcloud_generator_.GeneratePointCloud2Message(
            point_on_sensor_frame, this->child_frame_));
    output.local_topic_suffix = "/local";
  }

  // Create points on target frame
  if (this->enable_target_frame_cloud_) {
    Pose robot_pose;
    robot_pose.position.x = msg->robot_x;
    robot_pose.position.y = msg->robot_y;
    robot_pose.orientation.yaw = msg->robot_angle;

    Point point_on_robot_frame =
        this->frame_converter_.TfSensorFrame2RobotFrame(
            point_on_sensor_frame, this->sensor_extrinsic_);

    std::vector<Point> points_on_target_frame;
    if (this->target_frame_ == "map") {
      points_on_target_frame = this->frame_converter_.TfRobot2GlobalFrame(
          point_on_robot_frame, robot_pose);
    } else {
      points_on_target_frame = {point_on_robot_frame};
    }

    output.target_frame_clouds.push_back(
        this->pointcloud_generator_.GeneratePointCloud2Message(
            points_on_target_frame, this->target_frame_));
  }

  return output;
}

}  // namespace sensor_manager