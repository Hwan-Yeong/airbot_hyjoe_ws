#include "cloud_converter/cloud_converter.hpp"

#include "cloud_converter/cloud_converter_factory.hpp"
#include "sensor_to_pointcloud_node.hpp"

namespace sensor_to_pointcloud {

CloudConverterStrategy::CloudConverterStrategy(std::shared_ptr<SensorToPointcloudNode> node_ptr_) : node_ptr(node_ptr_)
{
    this->target_frame_ = node_ptr->getTargetFrame();
}

std::shared_ptr<SensorToPointcloudNode> CloudConverterStrategy::getNodePtr() const
{
    return node_ptr;
}

TofMonoCloudConverter::TofMonoCloudConverter(std::shared_ptr<SensorToPointcloudNode> node_ptr_, const YAML::Node &config)
    : CloudConverterStrategy(node_ptr_)
{
    // Load Config
    if (!config.IsMap())
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
    }

    this->use_tof_mono_ = config["use"].as<bool>();
    this->sensor_frame_pose_ = tPose(
        tPoint(
            config["extrinsics"]["translation"]["x"].as<double>(),
            config["extrinsics"]["translation"]["y"].as<double>(),
            config["extrinsics"]["translation"]["z"].as<double>()
        ),
        tOrientation(
            DEG2RAD(config["extrinsics"]["rotation"]["roll"].as<double>()),
            DEG2RAD(config["extrinsics"]["rotation"]["pitch"].as<double>()),
            DEG2RAD(config["extrinsics"]["rotation"]["yaw"].as<double>())
        )
    );

    // Print Config
    std::ostringstream oss;
    oss << "\n[1D TOF POINTCLOUD CONVERTER PARAMETERS]\n";
    oss << std::boolalpha;
    oss << "  use_tof_mono_          : " << this->use_tof_mono_ << "\n";
    oss << "  sensor_frame_pose      : position [m] = ("
        << std::fixed << std::setprecision(5) << this->sensor_frame_pose_.position.x << ", "
        << std::fixed << std::setprecision(5) << this->sensor_frame_pose_.position.y << ", "
        << std::fixed << std::setprecision(5) << this->sensor_frame_pose_.position.z << "), "
        << "orientation [deg] = ("
        << std::fixed << std::setprecision(1) << RAD2DEG(this->sensor_frame_pose_.orientation.roll)  << ", "
        << std::fixed << std::setprecision(1) << RAD2DEG(this->sensor_frame_pose_.orientation.pitch) << ", "
        << std::fixed << std::setprecision(1) << RAD2DEG(this->sensor_frame_pose_.orientation.yaw)   << ")\n";
    oss << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr->get_logger(), "%s", oss.str().c_str());
}

sensor_msgs::msg::PointCloud2 TofMonoCloudConverter::pc_convert(const void *sensor_msg)
{
    sensor_msgs::msg::PointCloud2 ret;

    if (this->use_tof_mono_)
    {
        auto msg = static_cast<const robot_custom_msgs::msg::TofData*>(sensor_msg);

        tPose robot_pose;
        robot_pose.position.x = msg->robot_x;
        robot_pose.position.y = msg->robot_y;
        robot_pose.orientation.yaw = msg->robot_angle;

        tPoint point_on_robot_frame = this->frame_converter_.tfMonoTofSensor2RobotFrame(msg->top, this->sensor_frame_pose_);

        if (this->target_frame_ == "map") {
            std::vector<tPoint> point_on_map_frame = this->frame_converter_.tfRobot2GlobalFrame({point_on_robot_frame}, robot_pose);
            ret = this->pointcloud_generator_.generatePointCloud2Message(point_on_map_frame, this->target_frame_);
        } else if (this->target_frame_ == "base_link") {
            ret = this->pointcloud_generator_.generatePointCloud2Message({point_on_robot_frame}, this->target_frame_);
        } else {
            RCLCPP_INFO(this->node_ptr->get_logger(), "Select Wrong Target Frame: %s", this->target_frame_.c_str());
        }
    }

    return ret;
}

CameraCloudConverter::CameraCloudConverter(std::shared_ptr<SensorToPointcloudNode> node_ptr_, const YAML::Node &config)
    : CloudConverterStrategy(node_ptr_)
{
    // Load Config
    if (!config.IsMap())
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
    }

    this->use_camera_ = config["use"].as<bool>();
    this->object_direction_ = config["object_direction"].as<bool>();
    this->pointcloud_resolution_ = config["pointcloud_resolution"].as<double>();
    this->object_max_dist_ = config["object_max_distance_m"].as<double>();
    for (const auto& class_id: config["class_id_confidence_th"]) {
        auto item = class_id.as<std::string>();
        std::istringstream ss(item);
        std::string key, value;
        if (std::getline(ss, key, ':') && std::getline(ss, value)) {
            this->camera_class_id_confidence_th_[std::stoi(key)] = std::stoi(value);
        }
    }
    this->sensor_frame_pose_.position = tPoint(
        config["extrinsics"]["translation"]["x"].as<double>(),
        config["extrinsics"]["translation"]["y"].as<double>(),
        config["extrinsics"]["translation"]["z"].as<double>()
    );

    // Print Config
    std::ostringstream oss;
    oss << "\n[CAMERA POINTCLOUD CONVERTER PARAMETERS]\n";
    oss << std::boolalpha;
    oss << "  use_camera_            : " << this->use_camera_ << "\n";
    oss << "  object_direction_      : " << this->object_direction_ << "\n";
    oss << "  pointcloud_resolution_ : " << std::fixed << std::setprecision(2) << this->pointcloud_resolution_ << "\n";
    oss << "  object_max_dist_       : " << std::fixed << std::setprecision(2) << this->object_max_dist_ << " m\n";
    oss << "  sensor_frame_pose      : position [m] = ("
        << std::fixed << std::setprecision(5) << this->sensor_frame_pose_.position.x << ", "
        << std::fixed << std::setprecision(5) << this->sensor_frame_pose_.position.y << ", "
        << std::fixed << std::setprecision(5) << this->sensor_frame_pose_.position.z << ")\n";
    oss << "  class_id_confidence_th :\n";
    for (const auto& [class_id, confidence_th] : this->camera_class_id_confidence_th_) {
        oss << "    - { id: " << class_id << ", th: " << confidence_th << " }\n";
    }
    oss << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr->get_logger(), "%s", oss.str().c_str());
}

sensor_msgs::msg::PointCloud2 CameraCloudConverter::pc_convert(const void *sensor_msg)
{
    sensor_msgs::msg::PointCloud2 ret;

    if (this->use_camera_)
    {
        auto msg = static_cast<const robot_custom_msgs::msg::CameraDataArray*>(sensor_msg);

        tPose robot_pose;
        robot_pose.position.x = msg->robot_x;
        robot_pose.position.y = msg->robot_y;
        robot_pose.orientation.yaw = msg->robot_angle;

        vision_msgs::msg::BoundingBox2DArray bbox_array = frame_converter_.tfCameraSensor2RobotFrameBBoxArray(
            msg, robot_pose, this->camera_class_id_confidence_th_, this->object_direction_, this->object_max_dist_, this->target_frame_, this->sensor_frame_pose_);

        bbox_array.header.stamp = this->node_ptr->get_clock()->now();

        sensor_msgs::msg::PointCloud2 pc_msg = this->pointcloud_generator_.generateCameraPointCloud2Message(bbox_array, this->pointcloud_resolution_);

        ret = pc_msg;
    }

    return ret;
}

CollisionCloudConverter::CollisionCloudConverter(std::shared_ptr<SensorToPointcloudNode> node_ptr_, const YAML::Node &config)
    : CloudConverterStrategy(node_ptr_)
{
    // Load Config
    if (!config.IsMap())
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
    }

    this->use_collision_ = config["use"].as<bool>();
    this->sensor_frame_pose_.position = tPoint(
        config["extrinsics"]["distance"].as<double>(),
        0.0,
        0.0
    );

    // Print Config
    std::ostringstream oss;
    oss << "\n[COLLISION POINTCLOUD CONVERTER PARAMETERS]\n";
    oss << std::boolalpha;
    oss << "  use_collision_         : " << this->use_collision_ << "\n";
    oss << "  sensor_frame_pose      : position [m] = ("
        << std::fixed << std::setprecision(5) << this->sensor_frame_pose_.position.x << ", "
        << std::fixed << std::setprecision(5) << this->sensor_frame_pose_.position.y << ", "
        << std::fixed << std::setprecision(5) << this->sensor_frame_pose_.position.z << ")\n";
    oss << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr->get_logger(), "%s", oss.str().c_str());
}

sensor_msgs::msg::PointCloud2 CollisionCloudConverter::pc_convert(const void *sensor_msg)
{
    sensor_msgs::msg::PointCloud2 ret;

    if (this->use_collision_)
    {
        auto msg = static_cast<const robot_custom_msgs::msg::AbnormalEventData*>(sensor_msg);

        tPose robot_pose;
        robot_pose.position.x = msg->robot_x;
        robot_pose.position.y = msg->robot_y;
        robot_pose.orientation.yaw = msg->robot_angle;

        tPoint point_on_robot_frame = this->frame_converter_.tfCollisionData2RobotFrame(msg, this->sensor_frame_pose_.position.x);

        if (this->target_frame_ == "map") {
            std::vector<tPoint> point_on_map_frame = this->frame_converter_.tfRobot2GlobalFrame({point_on_robot_frame}, robot_pose);
            ret = this->pointcloud_generator_.generatePointCloud2Message(point_on_map_frame, this->target_frame_);
        } else if (this->target_frame_ == "base_link") {
            ret = this->pointcloud_generator_.generatePointCloud2Message({point_on_robot_frame}, this->target_frame_);
        } else {
            RCLCPP_INFO(this->node_ptr->get_logger(), "Select Wrong Target Frame: %s", this->target_frame_.c_str());
        }
    }

    return ret;
}

EmptyCloudConverter::EmptyCloudConverter(std::shared_ptr<SensorToPointcloudNode> node_ptr_, const YAML::Node& config)
    : CloudConverterStrategy(node_ptr_)
{
    // Load Config
    if (!config.IsMap())
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
    }

    this->use_empty_msg_ = config["use"].as<bool>();

    // Print Config
    std::ostringstream oss;
    oss << "\n[EMPTY POINTCLOUD PARAMETERS]\n";
    oss << std::boolalpha;
    oss << "  use_empty_: " << this->use_empty_msg_ << "\n";
    oss << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr->get_logger(), "%s", oss.str().c_str());
}

sensor_msgs::msg::PointCloud2 EmptyCloudConverter::pc_convert(const void *sensor_msg)
{
    sensor_msgs::msg::PointCloud2 ret;

    if (sensor_msg == nullptr && this->use_empty_msg_) {
        ret = this->pointcloud_generator_.generateEmptyPointCloud2Message(this->target_frame_);
    }

    return ret;
}

} // namespace sensor_to_pointcloud