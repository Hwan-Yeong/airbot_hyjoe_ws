#include "cloud_converter/cloud_converter.hpp"

#include "cloud_converter/cloud_converter_factory.hpp"
#include "sensor_manager_node.hpp"

namespace sensor_manager {

CloudConverterStrategy::CloudConverterStrategy(std::shared_ptr<SensorManagerNode> node_ptr_) : node_ptr(node_ptr_)
{
    this->target_frame_ = node_ptr->getTargetFrame();
}

std::shared_ptr<SensorManagerNode> CloudConverterStrategy::getNodePtr() const
{
    return node_ptr;
}

TofMonoCloudConverter::TofMonoCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr_, const YAML::Node &config)
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
    oss << "  use_tof_mono_      : " << this->use_tof_mono_ << "\n";
    oss << "  sensor_frame_pose  : position [m] = ("
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

PointCloudMsgVector TofMonoCloudConverter::pc_convert(const void *sensor_msg)
{
    PointCloudMsg cloud;

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
            cloud = this->pointcloud_generator_.generatePointCloud2Message(point_on_map_frame, this->target_frame_);
        } else if (this->target_frame_ == "base_link") {
            cloud = this->pointcloud_generator_.generatePointCloud2Message({point_on_robot_frame}, this->target_frame_);
        } else {
            RCLCPP_INFO(this->node_ptr->get_logger(), "Select Wrong Target Frame: %s", this->target_frame_.c_str());
        }
    }

    return {cloud};
}

CameraCloudConverter::CameraCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr_, const YAML::Node &config)
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
    oss << "  use_camera_             : " << this->use_camera_ << "\n";
    oss << "  object_direction_       : " << this->object_direction_ << "\n";
    oss << "  pointcloud_resolution_  : " << std::fixed << std::setprecision(2) << this->pointcloud_resolution_ << "\n";
    oss << "  object_max_dist_        : " << std::fixed << std::setprecision(2) << this->object_max_dist_ << " m\n";
    oss << "  sensor_frame_pose       : position [m] = ("
        << std::fixed << std::setprecision(5) << this->sensor_frame_pose_.position.x << ", "
        << std::fixed << std::setprecision(5) << this->sensor_frame_pose_.position.y << ", "
        << std::fixed << std::setprecision(5) << this->sensor_frame_pose_.position.z << ")\n";
    oss << "  class_id_confidence_th  :\n";
    for (const auto& [class_id, confidence_th] : this->camera_class_id_confidence_th_) {
        oss << "    - { id: " << class_id << ", th: " << confidence_th << " }\n";
    }
    oss << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr->get_logger(), "%s", oss.str().c_str());
}

PointCloudMsgVector CameraCloudConverter::pc_convert(const void *sensor_msg)
{
    PointCloudMsg cloud;

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

        cloud = this->pointcloud_generator_.generateCameraPointCloud2Message(bbox_array, this->pointcloud_resolution_);
    }

    return {cloud};
}

BottomIrCloudConverter::BottomIrCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr_, const YAML::Node &config)
    : CloudConverterStrategy(node_ptr_)
{
    // Load Config
    if (!config.IsMap())
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
    }

    this->use_bottom_ir_ = config["use"].as<bool>();
    this->ir_dist_center_to_sensor = config["extrinsics"]["distance"].as<double>();
    this->ir_angle_sensor_to_next_sensor = config["extrinsics"]["angle"].as<double>();

    // Print Config
    std::ostringstream oss;
    oss << "\n[BOTTOM IR POINTCLOUD CONVERTER PARAMETERS]\n";
    oss << std::boolalpha;
    oss << "  use_bottom_ir_                        : " << this->use_bottom_ir_ << "\n";
    oss << "  ir_dist_center_to_sensor [m]          : " << this->ir_dist_center_to_sensor << "\n";
    oss << "  ir_angle_sensor_to_next_sensor [deg]  : " << this->ir_angle_sensor_to_next_sensor <<" \n";
    oss << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr->get_logger(), "%s", oss.str().c_str());
}

PointCloudMsgVector BottomIrCloudConverter::pc_convert(const void *sensor_msg)
{
    PointCloudMsg cloud;

    if (this->use_bottom_ir_)
    {
        auto msg = static_cast<const robot_custom_msgs::msg::BottomIrData*>(sensor_msg);

        tPose robot_pose;
        robot_pose.position.x = msg->robot_x;
        robot_pose.position.y = msg->robot_y;
        robot_pose.orientation.yaw = msg->robot_angle;

        std::vector<tPoint> point_on_robot_frame = this->frame_converter_.tfBottomIrSensor2RobotFrame(msg, this->ir_dist_center_to_sensor, this->ir_angle_sensor_to_next_sensor);

        if (this->target_frame_ == "map") {
            std::vector<tPoint> point_on_map_frame = this->frame_converter_.tfRobot2GlobalFrame(point_on_robot_frame, robot_pose);
            cloud = this->pointcloud_generator_.generatePointCloud2Message(point_on_map_frame, this->target_frame_);
        } else if (this->target_frame_ == "base_link") {
            cloud = this->pointcloud_generator_.generatePointCloud2Message({point_on_robot_frame}, this->target_frame_);
        } else {
            RCLCPP_INFO(this->node_ptr->get_logger(), "Select Wrong Target Frame: %s", this->target_frame_.c_str());
        }
    }

    return {cloud};
}

CollisionCloudConverter::CollisionCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr_, const YAML::Node &config)
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
    oss << "  use_collision_     : " << this->use_collision_ << "\n";
    oss << "  sensor_frame_pose  : position [m] = ("
        << std::fixed << std::setprecision(5) << this->sensor_frame_pose_.position.x << ", "
        << std::fixed << std::setprecision(5) << this->sensor_frame_pose_.position.y << ", "
        << std::fixed << std::setprecision(5) << this->sensor_frame_pose_.position.z << ")\n";
    oss << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr->get_logger(), "%s", oss.str().c_str());
}

PointCloudMsgVector CollisionCloudConverter::pc_convert(const void *sensor_msg)
{
    PointCloudMsg cloud;

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
            cloud = this->pointcloud_generator_.generatePointCloud2Message(point_on_map_frame, this->target_frame_);
        } else if (this->target_frame_ == "base_link") {
            cloud = this->pointcloud_generator_.generatePointCloud2Message({point_on_robot_frame}, this->target_frame_);
        } else {
            RCLCPP_INFO(this->node_ptr->get_logger(), "Select Wrong Target Frame: %s", this->target_frame_.c_str());
        }
    }

    return {cloud};
}

EmptyCloudConverter::EmptyCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr_, const YAML::Node& config)
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
    oss << "  use_empty_  : " << this->use_empty_msg_ << "\n";
    oss << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr->get_logger(), "%s", oss.str().c_str());
}

PointCloudMsgVector EmptyCloudConverter::pc_convert(const void *sensor_msg)
{
    PointCloudMsg cloud;

    if (sensor_msg == nullptr && this->use_empty_msg_) {
        cloud = this->pointcloud_generator_.generateEmptyPointCloud2Message(this->target_frame_);
    }

    return {cloud};
}

} // namespace sensor_manager