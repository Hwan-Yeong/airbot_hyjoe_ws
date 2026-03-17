#include "cloud_converter/cloud_converter_strategy.hpp"

#include "cloud_converter/cloud_converter_factory.hpp"
#include "sensor_manager_node.hpp"

namespace sensor_manager {

CloudConverterStrategy::CloudConverterStrategy(SensorManagerNode* node_ptr)
    : node_ptr_(node_ptr) {
  last_call_time_ = std::chrono::steady_clock::now();
  this->target_frame_ = node_ptr_->GetTargetFrame();
  this->frame_converter_.SetTfBuffer(node_ptr_->GetTfBuffer());
}

ConverterOutput CloudConverterStrategy::PcConvert(const void* sensor_msg) {
  auto now = std::chrono::steady_clock::now();

  if (timeout_limit_sec_ > 0.0) {
    auto duration =
        std::chrono::duration_cast<std::chrono::seconds>(now - last_call_time_);
    double duration_sec = duration.count();

    /**
     * @brief Converter Internal Variable auto Initialization
     *
     * @details Internal Variable auto Initialization happens in two cases:
     * 1) sensor_manager node "on -> off -(over timeout)-> on"
     * 2) sensor data "receive -> not receive -(over timeout)-> receive"
     */
    if ((duration_sec >= timeout_limit_sec_)) {
      if (!is_already_reset_) {
        RCLCPP_WARN(this->node_ptr_->get_logger(),
                    "[%s] Data gap (%.1fs) exceeded reset timeout (%.1fs). "
                    "Resetting...",
                    typeid(*this).name(), duration_sec, timeout_limit_sec_);
        ResetInternalVariables();
        is_already_reset_ = true;
      }
    } else {
      is_already_reset_ = false;
    }
  }

  last_call_time_ = now;
  return PcConvertImpl(sensor_msg);
}

ConverterOutput CloudConverterStrategy::PcConvertEmpty(
  const std::string& frame_id) {
  ConverterOutput output;
  output.target_frame_clouds.push_back(
      this->pointcloud_generator_.GenerateEmptyPointCloud2Message(frame_id));
  return output;
}

void CloudConverterStrategy::LoadCommonConfig(const YAML::Node& config) {
  if (!config.IsMap()) return;

  this->use_converter_ = config["use"] ? config["use"].as<bool>() : true;
  this->enable_target_frame_cloud_ =
      config["enable_target_frame_cloud"]
          ? config["enable_target_frame_cloud"].as<bool>()
          : false;
  this->enable_sensor_tf_cloud_ =
      config["enable_sensor_tf_cloud"]
          ? config["enable_sensor_tf_cloud"].as<bool>()
          : false;
  this->timeout_limit_sec_ = config["reset_timeout_sec"]
                                 ? config["reset_timeout_sec"].as<double>()
                                 : -1.0;

  try {
    if (config["extrinsics"]) {
      const auto& ext = config["extrinsics"];

      if (ext["tf_frame"]) {
        this->parent_frame_ = ext["tf_frame"]["parent"]
                                  ? ext["tf_frame"]["parent"].as<std::string>()
                                  : "base_link";
        this->child_frame_ = ext["tf_frame"]["child"]
                                 ? ext["tf_frame"]["child"].as<std::string>()
                                 : "";
      }

      if (ext["translation"]) {
        this->sensor_extrinsic_.position.x =
            ext["translation"]["x"] ? ext["translation"]["x"].as<double>()
                                    : 0.0;
        this->sensor_extrinsic_.position.y =
            ext["translation"]["y"] ? ext["translation"]["y"].as<double>()
                                    : 0.0;
        this->sensor_extrinsic_.position.z =
            ext["translation"]["z"] ? ext["translation"]["z"].as<double>()
                                    : 0.0;
      }

      if (ext["rotation"]) {
        this->sensor_extrinsic_.orientation.roll = DEG2RAD(
            ext["rotation"]["roll"] ? ext["rotation"]["roll"].as<double>()
                                    : 0.0);
        this->sensor_extrinsic_.orientation.pitch = DEG2RAD(
            ext["rotation"]["pitch"] ? ext["rotation"]["pitch"].as<double>()
                                     : 0.0);
        this->sensor_extrinsic_.orientation.yaw = DEG2RAD(
            ext["rotation"]["yaw"] ? ext["rotation"]["yaw"].as<double>() : 0.0);
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Extrinsics parsing failed: %s",
                 e.what());
  }
}

std::string CloudConverterStrategy::GetCommonConfigInfo(
    const std::string& sensor_type) {
  std::ostringstream oss;
  oss << "\n[" << sensor_type << " CONVERTER PARAMETERS]\n"
      << std::boolalpha << "  use_converter_            : "
      << this->use_converter_ << "\n"
      << "  enable_target_cloud       : " << this->enable_target_frame_cloud_
      << "\n"
      << "  enable_sensor_tf          : " << this->enable_sensor_tf_cloud_
      << "\n"
      << "  reset_timeout_sec         : " << this->timeout_limit_sec_ << "\n"
      << "  tf frame (Parent/Child)   : " << this->parent_frame_ << " / "
      << this->child_frame_ << "\n"
      << "  sensor_extrinsic_pose     : translation [m] x/y/z = ("
      << this->sensor_extrinsic_.position.x << ", "
      << this->sensor_extrinsic_.position.y << ", "
      << this->sensor_extrinsic_.position.z << ")\n"
      << "   ->(from base_link)       : rotation [deg] r/p/y = ("
      << RAD2DEG(this->sensor_extrinsic_.orientation.roll) << ", "
      << RAD2DEG(this->sensor_extrinsic_.orientation.pitch) << ", "
      << RAD2DEG(this->sensor_extrinsic_.orientation.yaw) << ")\n";

  return oss.str();
}

std::optional<geometry_msgs::msg::TransformStamped>
CloudConverterStrategy::GetStaticTf() {
  if (!enable_sensor_tf_cloud_ || parent_frame_.empty() || child_frame_.empty())
    return std::nullopt;

  geometry_msgs::msg::TransformStamped tfs;
  tfs.header.stamp = node_ptr_->get_clock()->now();
  tfs.header.frame_id = this->parent_frame_;
  tfs.child_frame_id = this->child_frame_;

  tfs.transform.translation.x = this->sensor_extrinsic_.position.x;
  tfs.transform.translation.y = this->sensor_extrinsic_.position.y;
  tfs.transform.translation.z = this->sensor_extrinsic_.position.z;

  tf2::Quaternion q;
  q.setRPY(this->sensor_extrinsic_.orientation.roll,
           this->sensor_extrinsic_.orientation.pitch,
           this->sensor_extrinsic_.orientation.yaw);
  tfs.transform.rotation = tf2::toMsg(q);

  return tfs;
}

SensorManagerNode* CloudConverterStrategy::GetNodePtr() const {
  return node_ptr_;
}

}  // namespace sensor_manager
