#include "cloud_converter/sensors/tof_mono.hpp"
#include "sensor_manager_node.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"

namespace sensor_manager {

TofMonoCloudConverter::TofMonoCloudConverter(
    std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node& config)
    : CloudConverterStrategy(node_ptr) {
  if (!config.IsMap()) {
    auto s = YAML::Dump(config);
    throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
  }

  // TODO: Set Default Config
  //       => Hard Coding for default value (e.g. yaml file is broken)

  // Load Config
  LoadCommonConfig(config);

  // Print Config
  std::ostringstream oss_1dtof;
  oss_1dtof << GetCommonConfigInfo("1D ToF");
  oss_1dtof << "----------------------------------------------------";
  RCLCPP_INFO(this->node_ptr_->get_logger(), "%s", oss_1dtof.str().c_str());
}

ConverterOutput TofMonoCloudConverter::PcConvertImpl(const void* sensor_msg) {
  ConverterOutput output;

  if (!this->use_converter_) return output;

  auto msg = static_cast<const robot_custom_msgs::msg::TofData*>(sensor_msg);

  // Create point on sensor frame
  Point point_on_sensor_frame;
  point_on_sensor_frame.x = msg->top;
  point_on_sensor_frame.y = 0.0;
  point_on_sensor_frame.z = 0.0;

  if (this->enable_sensor_tf_cloud_) {
    output.local_frame_clouds.push_back(
        this->pointcloud_generator_.GeneratePointCloud2Message(
            {point_on_sensor_frame}, this->child_frame_));
    output.local_topic_suffix = "/local";
  }

  // Create point on target frame
  if (this->enable_target_frame_cloud_) {
    Pose robot_pose;
    robot_pose.position.x = msg->robot_x;
    robot_pose.position.y = msg->robot_y;
    robot_pose.orientation.yaw = msg->robot_angle;

    Point point_on_robot_frame =
        this->frame_converter_.TfMonoTofSensor2RobotFrame(
            msg->top, this->sensor_extrinsic_);

    std::vector<Point> point_on_target_frame;
    if (this->target_frame_ == "base_link") {
      point_on_target_frame = {point_on_robot_frame};
    } else if (this->target_frame_ == "map") {
      point_on_target_frame = this->frame_converter_.TfRobot2GlobalFrame(
          point_on_robot_frame, robot_pose);
    } else {
      RCLCPP_INFO(this->node_ptr_->get_logger(),
                  "Select Wrong Target Frame: %s", this->target_frame_.c_str());
    }

    if (!point_on_target_frame.empty()) {
      output.target_frame_clouds.push_back(
          this->pointcloud_generator_.GeneratePointCloud2Message(
              point_on_target_frame, this->target_frame_));
    }
  }

  return output;
}

}  // namespace sensor_manager