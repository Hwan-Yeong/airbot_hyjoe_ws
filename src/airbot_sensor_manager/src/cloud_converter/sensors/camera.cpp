#include "cloud_converter/sensors/camera.hpp"
#include "sensor_manager_node.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"

namespace sensor_manager {

CameraCloudConverter::CameraCloudConverter(
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
  this->object_direction_ = config["object_direction"].as<bool>();
  this->pointcloud_resolution_ = config["pointcloud_resolution"].as<double>();
  this->object_max_dist_ = config["object_max_distance_m"].as<double>();
  this->object_ignore_pitch_th_ =
      DEG2RAD(config["object_ignore_pitch_th_deg"].as<double>());
  for (const auto& class_id : config["class_id_confidence_th"]) {
    auto item = class_id.as<std::string>();
    std::istringstream ss(item);
    std::string key, value;
    if (std::getline(ss, key, ':') && std::getline(ss, value)) {
      this->camera_class_id_confidence_th_[std::stoi(key)] = std::stoi(value);
    }
  }
  this->object_direction_ = config["object_direction"]
                                ? config["object_direction"].as<bool>()
                                : true;
  this->pointcloud_resolution_ = config["pointcloud_resolution"]
                                     ? config["pointcloud_resolution"].as<double>()
                                     : 0.05;
  this->object_max_dist_ = config["object_max_distance_m"]
                               ? config["object_max_distance_m"].as<double>()
                               : 1.5;
  this->object_ignore_pitch_th_ =
      DEG2RAD(config["object_ignore_pitch_th_deg"]
                  ? config["object_ignore_pitch_th_deg"].as<double>()
                  : 3.0);
  if (config["class_id_confidence_th"]) {
    for (const auto& class_id : config["class_id_confidence_th"]) {
      auto item = class_id.as<std::string>();
      std::istringstream ss(item);
      std::string key, value;
      if (std::getline(ss, key, ':') && std::getline(ss, value)) {
        this->camera_class_id_confidence_th_[std::stoi(key)] = std::stoi(value);
      }
    }
  }
  this->use_object_logger_ = config["logger"]["use"].as<bool>();
  this->object_logger_margin_distance_diff_m_ =
      config["logger"]["margin"]["distance_diff_m"].as<double>();
  if (config["logger"]) {
    this->use_object_logger_ =
        config["logger"]["use"] ? config["logger"]["use"].as<bool>() : true;
    if (config["logger"]["margin"]) {
      this->object_logger_margin_distance_diff_m_ =
          config["logger"]["margin"]["distance_diff_m"]
              ? config["logger"]["margin"]["distance_diff_m"].as<double>()
              : 1.0;
    }
  }

  // Print Config
  std::ostringstream oss_camera;
  oss_camera << GetCommonConfigInfo("CAMERA");
  oss_camera << "  object_direction_       : " << this->object_direction_
             << "\n";
  oss_camera << "  pointcloud_resolution_  : " << std::fixed
             << std::setprecision(2) << this->pointcloud_resolution_ << "\n";
  oss_camera << "  object_max_dist_        : " << std::fixed
             << std::setprecision(2) << this->object_max_dist_ << " m\n";
  oss_camera << "  object_ignore_pitch_th_ : " << std::fixed
             << std::setprecision(2) << RAD2DEG(this->object_ignore_pitch_th_)
             << " deg\n";
  oss_camera << "  class_id_confidence_th  :\n";
  for (const auto& [class_id, confidence_th] :
       this->camera_class_id_confidence_th_) {
    oss_camera << "    - { id: " << std::setw(2) << std::setfill('0')
               << class_id << ", th: " << confidence_th << " }\n";
  }
  oss_camera << "  use_logger_             : " << this->use_object_logger_
             << "\n";
  oss_camera << "  log_margin_dist_diff_m_ : " << std::fixed
             << std::setprecision(2)
             << this->object_logger_margin_distance_diff_m_ << "\n";
  oss_camera << "----------------------------------------------------";
  RCLCPP_INFO(this->node_ptr_->get_logger(), "%s", oss_camera.str().c_str());

  // IMU Msg Subscriber
  SetupImuSubscription();
}

void CameraCloudConverter::SetupImuSubscription() {
  imu_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::Imu>(
      "/imu_data", rclcpp::QoS(10).reliable(),
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        double roll, pitch, yaw;
        tf2::Quaternion quaternion(msg->orientation.x, msg->orientation.y,
                                   msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        if (abs(roll) >= this->object_ignore_pitch_th_ ||
            abs(pitch) >= this->object_ignore_pitch_th_) {
          is_ramp_detection_ = true;
          ramp_release_cnt = 0;
        } else {
          if (is_ramp_detection_) {
            ramp_release_cnt++;
            if (ramp_release_cnt > 10) {  // 1 sec
              is_ramp_detection_ = false;
              ramp_release_cnt = 0;
            }
          }
        }
      });
}

ConverterOutput CameraCloudConverter::PcConvertImpl(const void* sensor_msg) {
  ConverterOutput output;
  if (!this->use_converter_) return output;
  if (this->is_ramp_detection_) {
    RCLCPP_WARN(this->node_ptr_->get_logger(),
                "Ramp detected, skipping point cloud conversion");
    return output;
  }

  auto msg =
      static_cast<const robot_custom_msgs::msg::CameraDataArray*>(sensor_msg);
  auto now = this->node_ptr_->get_clock()->now();

  // Create points on sensor frame
  std::vector<CameraObject> objects_on_sensor_frame =
      this->frame_converter_.TfCameraSensor2SensorFrame(
          msg, this->object_direction_, this->object_max_dist_);

  if (objects_on_sensor_frame.empty()) return output;

  if (this->enable_sensor_tf_cloud_) {
    output.local_frame_clouds.push_back(
        this->pointcloud_generator_.GenerateCameraPointCloud2Message(
            this->frame_converter_.ToBBoxArray(objects_on_sensor_frame),
            this->pointcloud_resolution_, this->child_frame_, this->sensor_extrinsic_));
    output.local_topic_suffix = "/local";
  }

  // Create points on target frame
  if (this->enable_target_frame_cloud_) {
    auto objects_on_robot_frame =
        this->frame_converter_.TfCameraObjects2RobotFrame(
            objects_on_sensor_frame, this->sensor_extrinsic_);

    std::vector<CameraObject> objects_on_target_frame;
    if (this->target_frame_ == "map") {
      Pose robot_pose;
      robot_pose.position.x = msg->robot_x;
      robot_pose.position.y = msg->robot_y;
      robot_pose.orientation.yaw = msg->robot_angle;
      objects_on_target_frame =
          this->frame_converter_.TfCameraObjects2GlobalFrame(
              objects_on_robot_frame, robot_pose);
    } else {
      objects_on_target_frame = objects_on_robot_frame;
    }

    if (this->use_object_logger_) {
      this->LogNewObjects(objects_on_target_frame);
    }

    output.target_frame_clouds.push_back(
        this->pointcloud_generator_.GenerateCameraPointCloud2Message(
            this->frame_converter_.ToBBoxArray(objects_on_target_frame),
            this->pointcloud_resolution_, this->target_frame_, this->sensor_extrinsic_));
  }

  return output;
}

void CameraCloudConverter::LogNewObjects(
    const std::vector<CameraObject>& objects) {
  for (const auto& obj : objects) {
    auto it = this->camera_class_id_confidence_th_.find(obj.id);
    if (it == this->camera_class_id_confidence_th_.end()) continue;

    bool is_new_object = true;
    if (logged_objects_.find(obj.id) != logged_objects_.end()) {
      for (const auto& old_obj : logged_objects_[obj.id]) {
        double dist = std::sqrt(
            std::pow(obj.bbox.center.position.x - old_obj.center.position.x,
                     2) +
            std::pow(obj.bbox.center.position.y - old_obj.center.position.y,
                     2));
        if (dist <= this->object_logger_margin_distance_diff_m_) {
          is_new_object = false;
          break;
        }
      }
    }

    if (is_new_object) {
      logged_objects_[obj.id].push_back(obj.bbox);
      RCLCPP_INFO(this->node_ptr_->get_logger(),
                  "================ [UPDATE] ================");
      RCLCPP_INFO(this->node_ptr_->get_logger(),
                  "[ID]: %u, [Position (X, Y): (%.3f, %.3f)], [Size (W, H): "
                  "(%.3f, %.3f)]",
                  obj.id, obj.bbox.center.position.x,
                  obj.bbox.center.position.y, obj.bbox.size_y, obj.bbox.size_x);
    }
  }
}

}  // namespace sensor_manager