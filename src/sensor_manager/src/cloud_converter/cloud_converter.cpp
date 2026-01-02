#include "cloud_converter/cloud_converter.hpp"

#include "cloud_converter/cloud_converter_factory.hpp"
#include "sensor_manager_node.hpp"

namespace sensor_manager {

CloudConverterStrategy::CloudConverterStrategy(std::shared_ptr<SensorManagerNode> node_ptr_) : node_ptr(node_ptr_)
{
    this->target_frame_ = node_ptr->getTargetFrame();
}

std::shared_ptr<SensorManagerNode> CloudConverterStrategy::get_node_ptr() const
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
    this->tof_mono_sensor_frame_pose_ = tPose(
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
        << std::fixed << std::setprecision(5) << this->tof_mono_sensor_frame_pose_.position.x << ", "
        << std::fixed << std::setprecision(5) << this->tof_mono_sensor_frame_pose_.position.y << ", "
        << std::fixed << std::setprecision(5) << this->tof_mono_sensor_frame_pose_.position.z << "), "
        << "orientation [deg] = ("
        << std::fixed << std::setprecision(1) << RAD2DEG(this->tof_mono_sensor_frame_pose_.orientation.roll)  << ", "
        << std::fixed << std::setprecision(1) << RAD2DEG(this->tof_mono_sensor_frame_pose_.orientation.pitch) << ", "
        << std::fixed << std::setprecision(1) << RAD2DEG(this->tof_mono_sensor_frame_pose_.orientation.yaw)   << ")\n";
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

        tPoint point_on_robot_frame = this->frame_converter_.tfMonoTofSensor2RobotFrame(msg->top, this->tof_mono_sensor_frame_pose_);

        if (this->target_frame_ == "map") {
            std::vector<tPoint> points_on_map_frame = this->frame_converter_.tfRobot2GlobalFrame(point_on_robot_frame, robot_pose);
            cloud = this->pointcloud_generator_.generatePointCloud2Message(points_on_map_frame, this->target_frame_);
        } else if (this->target_frame_ == "base_link") {
            cloud = this->pointcloud_generator_.generatePointCloud2Message({point_on_robot_frame}, this->target_frame_);
        } else {
            RCLCPP_INFO(this->node_ptr->get_logger(), "Select Wrong Target Frame: %s", this->target_frame_.c_str());
        }
    }

    return {cloud};
}

TofMultiLeftCloudConverter::TofMultiLeftCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr_, const YAML::Node &config)
    : CloudConverterStrategy(node_ptr_)
{
    // Load Config
    if (!config.IsMap())
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
    }

    this->use_tof_multi_left_ = config["use"].as<bool>();
    this->tof_multi_left_fov_ = DEG2RAD(config["extrinsics"]["fov"].as<double>());
    this->tof_multi_left_sensor_frame_pose_ = tPose(
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
    for (auto idx : config["sub_cell_idx_array"]) {
        this->tof_multi_left_sub_cell_idx_array_.push_back(idx.as<int>());
    }

    // Print Config
    std::ostringstream oss;
    oss << "\n[MULTI TOF (Left) POINTCLOUD CONVERTER PARAMETERS]\n";
    oss << std::boolalpha;
    oss << "  use_tof_multi_left_  : " << this->use_tof_multi_left_ << "\n";
    oss << "  fov [deg]            : " << RAD2DEG(this->tof_multi_left_fov_) << "\n";
    oss << "  sensor_frame_pose    : position [m] = ("
        << std::fixed << std::setprecision(5) << this->tof_multi_left_sensor_frame_pose_.position.x << ", "
        << std::fixed << std::setprecision(5) << this->tof_multi_left_sensor_frame_pose_.position.y << ", "
        << std::fixed << std::setprecision(5) << this->tof_multi_left_sensor_frame_pose_.position.z << "), "
        << "orientation [deg] = ("
        << std::fixed << std::setprecision(1) << RAD2DEG(this->tof_multi_left_sensor_frame_pose_.orientation.roll)  << ", "
        << std::fixed << std::setprecision(1) << RAD2DEG(this->tof_multi_left_sensor_frame_pose_.orientation.pitch) << ", "
        << std::fixed << std::setprecision(1) << RAD2DEG(this->tof_multi_left_sensor_frame_pose_.orientation.yaw)   << ")\n";
    oss << "  sub_cell_idx_array   : ";
    for (auto idx : this->tof_multi_left_sub_cell_idx_array_) {
        oss << idx << " ";
    }
    oss << "\n";
    oss << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr->get_logger(), "%s", oss.str().c_str());

    // Init Calculation
    tof_utils_.updateSubCellIndexArray(
        this->tof_multi_left_sub_cell_idx_array_,
        this->tof_multi_left_fov_,
        this->tof_multi_left_y_tan_array_,
        this->tof_multi_left_z_tan_array_,
        this->node_ptr->get_logger()
    );
}

PointCloudMsgVector TofMultiLeftCloudConverter::pc_convert(const void *sensor_msg)
{
    PointCloudMsgVector clouds;

    if (this->use_tof_multi_left_)
    {
        auto msg = static_cast<const robot_custom_msgs::msg::TofData*>(sensor_msg);

        tPose robot_pose;
        robot_pose.position.x = msg->robot_x;
        robot_pose.position.y = msg->robot_y;
        robot_pose.orientation.yaw = msg->robot_angle;

        std::vector<double> left_dists(msg->bot_left.begin(), msg->bot_left.end());
        std::vector<tPoint> points_on_robot_frame = this->frame_converter_.tfMultiTofSensor2RobotFrame(
            left_dists,
            this->tof_multi_left_y_tan_array_,
            this->tof_multi_left_z_tan_array_,
            true,
            this->tof_multi_left_sensor_frame_pose_);

        if (this->target_frame_ == "map") {
            std::vector<tPoint> points_on_map_frame = this->frame_converter_.tfRobot2GlobalFrame(points_on_robot_frame, robot_pose);
            for (const auto& point : points_on_map_frame) {
                clouds.push_back(this->pointcloud_generator_.generatePointCloud2Message(point, this->target_frame_));
            }
        } else if (this->target_frame_ == "base_link") {
            for (const auto& point : points_on_robot_frame) {
                clouds.push_back(this->pointcloud_generator_.generatePointCloud2Message(point, this->target_frame_));
            }
        } else {
            RCLCPP_INFO(this->node_ptr->get_logger(), "Select Wrong Target Frame: %s", this->target_frame_.c_str());
        }
    }

    return clouds;
}

std_msgs::msg::Float32MultiArray TofMultiLeftCloudConverter::calibration_convert(const void* sensor_msg)
{
    std_msgs::msg::Float32MultiArray ret;

    auto msg = static_cast<const robot_custom_msgs::msg::TofData*>(sensor_msg);

    const size_t INDEX_SIZE = 16;

    std::vector<double> left_dists(msg->bot_left.begin(), msg->bot_left.end());
    std::vector<tPoint> robot_pts = this->frame_converter_.tfMultiTofSensor2RobotFrame(
        left_dists,
        this->tof_multi_left_y_tan_array_,
        this->tof_multi_left_z_tan_array_,
        true,
        this->tof_multi_left_sensor_frame_pose_);


    if (robot_pts.size() != INDEX_SIZE) {
        RCLCPP_WARN(this->node_ptr->get_logger(),
            "Expected %zu robot points, but got %zu.",
            INDEX_SIZE, robot_pts.size()
        );
    }

    size_t idx_num = 3; // 왼쪽 3개([13], [14], [15]) 캘리브레이션 진행 하여 3으로 설정
    if (robot_pts.size() >= idx_num) {
        const size_t n = robot_pts.size();
        ret.data.reserve(idx_num);
        for (size_t i = n - idx_num; i < n; ++i) { // 뒤에서부터 3개 인덱스 접근
            ret.data.push_back(static_cast<float>(sqrt(robot_pts[i].x*robot_pts[i].x + robot_pts[i].y*robot_pts[i].y)));
        }
    } else {
        RCLCPP_WARN(this->node_ptr->get_logger(),
            "robot_pts has fewer than 3 points (size=%zu). Returning empty array.",
            robot_pts.size()
        );
    }

    return ret;
}

TofMultiRightCloudConverter::TofMultiRightCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr_, const YAML::Node &config)
    : CloudConverterStrategy(node_ptr_)
{
    // Load Config
    if (!config.IsMap())
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
    }

    this->use_tof_multi_right_ = config["use"].as<bool>();
    this->tof_multi_right_fov_ = DEG2RAD(config["extrinsics"]["fov"].as<double>());
    this->tof_multi_right_sensor_frame_pose_ = tPose(
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
    for (auto idx : config["sub_cell_idx_array"]) {
        this->tof_multi_right_sub_cell_idx_array_.push_back(idx.as<int>());
    }

    // Print Config
    std::ostringstream oss;
    oss << "\n[MULTI TOF (Right) POINTCLOUD CONVERTER PARAMETERS]\n";
    oss << std::boolalpha;
    oss << "  use_tof_multi_right_  : " << this->use_tof_multi_right_ << "\n";
    oss << "  fov [deg]            : " << RAD2DEG(this->tof_multi_right_fov_) << "\n";
    oss << "  sensor_frame_pose    : position [m] = ("
        << std::fixed << std::setprecision(5) << this->tof_multi_right_sensor_frame_pose_.position.x << ", "
        << std::fixed << std::setprecision(5) << this->tof_multi_right_sensor_frame_pose_.position.y << ", "
        << std::fixed << std::setprecision(5) << this->tof_multi_right_sensor_frame_pose_.position.z << "), "
        << "orientation [deg] = ("
        << std::fixed << std::setprecision(1) << RAD2DEG(this->tof_multi_right_sensor_frame_pose_.orientation.roll)  << ", "
        << std::fixed << std::setprecision(1) << RAD2DEG(this->tof_multi_right_sensor_frame_pose_.orientation.pitch) << ", "
        << std::fixed << std::setprecision(1) << RAD2DEG(this->tof_multi_right_sensor_frame_pose_.orientation.yaw)   << ")\n";
    oss << "  sub_cell_idx_array   : ";
    for (auto idx : this->tof_multi_right_sub_cell_idx_array_) {
        oss << idx << " ";
    }
    oss << "\n";
    oss << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr->get_logger(), "%s", oss.str().c_str());

    // Init Calculation
    tof_utils_.updateSubCellIndexArray(
        this->tof_multi_right_sub_cell_idx_array_,
        this->tof_multi_right_fov_,
        this->tof_multi_right_y_tan_array_,
        this->tof_multi_right_z_tan_array_,
        this->node_ptr->get_logger()
    );
}

PointCloudMsgVector TofMultiRightCloudConverter::pc_convert(const void *sensor_msg)
{
    PointCloudMsgVector clouds;

    if (this->use_tof_multi_right_)
    {
        auto msg = static_cast<const robot_custom_msgs::msg::TofData*>(sensor_msg);

        tPose robot_pose;
        robot_pose.position.x = msg->robot_x;
        robot_pose.position.y = msg->robot_y;
        robot_pose.orientation.yaw = msg->robot_angle;

        std::vector<double> right_dists(msg->bot_right.begin(), msg->bot_right.end());
        std::vector<tPoint> points_on_robot_frame = this->frame_converter_.tfMultiTofSensor2RobotFrame(
            right_dists,
            this->tof_multi_right_y_tan_array_,
            this->tof_multi_right_z_tan_array_,
            false,
            this->tof_multi_right_sensor_frame_pose_);

        if (this->target_frame_ == "map") {
            std::vector<tPoint> points_on_map_frame = this->frame_converter_.tfRobot2GlobalFrame(points_on_robot_frame, robot_pose);
            for (const auto& point : points_on_map_frame) {
                clouds.push_back(this->pointcloud_generator_.generatePointCloud2Message(point, this->target_frame_));
            }
        } else if (this->target_frame_ == "base_link") {
            for (const auto& point : points_on_robot_frame) {
                clouds.push_back(this->pointcloud_generator_.generatePointCloud2Message(point, this->target_frame_));
            }
        } else {
            RCLCPP_INFO(this->node_ptr->get_logger(), "Select Wrong Target Frame: %s", this->target_frame_.c_str());
        }
    }

    return clouds;
}

std_msgs::msg::Float32MultiArray TofMultiRightCloudConverter::calibration_convert(const void* sensor_msg)
{
    std_msgs::msg::Float32MultiArray ret;

    auto msg = static_cast<const robot_custom_msgs::msg::TofData*>(sensor_msg);

    const size_t INDEX_SIZE = 16;

    std::vector<double> right_dists(msg->bot_right.begin(), msg->bot_right.end());
    std::vector<tPoint> robot_pts = this->frame_converter_.tfMultiTofSensor2RobotFrame(
        right_dists,
        this->tof_multi_right_y_tan_array_,
        this->tof_multi_right_z_tan_array_,
        false,
        this->tof_multi_right_sensor_frame_pose_);


    if (robot_pts.size() != INDEX_SIZE) {
        RCLCPP_WARN(this->node_ptr->get_logger(),
            "Expected %zu robot points, but got %zu.",
            INDEX_SIZE, robot_pts.size()
        );
    }

    size_t idx_num = 3; // 오른쪽 3개([13], [14], [15]) 캘리브레이션 진행 하여 3으로 설정
    if (robot_pts.size() >= idx_num) {
        const size_t n = robot_pts.size();
        ret.data.reserve(idx_num);
        for (size_t i = n - idx_num; i < n; ++i) { // 뒤에서부터 3개 인덱스 접근
            ret.data.push_back(static_cast<float>(sqrt(robot_pts[i].x*robot_pts[i].x + robot_pts[i].y*robot_pts[i].y)));
        }
    } else {
        RCLCPP_WARN(this->node_ptr->get_logger(),
            "robot_pts has fewer than 3 points (size=%zu). Returning empty array.",
            robot_pts.size()
        );
    }

    return ret;
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
    this->camera_sensor_frame_pose_.position = tPoint(
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
        << std::fixed << std::setprecision(5) << this->camera_sensor_frame_pose_.position.x << ", "
        << std::fixed << std::setprecision(5) << this->camera_sensor_frame_pose_.position.y << ", "
        << std::fixed << std::setprecision(5) << this->camera_sensor_frame_pose_.position.z << ")\n";
    oss << "  class_id_confidence_th  :\n";
    for (const auto& [class_id, confidence_th] : this->camera_class_id_confidence_th_) {
        oss << "    - { id: " << std::setw(2) << std::setfill('0') << class_id << ", th: " << confidence_th << " }\n";
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
            msg,
            robot_pose,
            this->camera_class_id_confidence_th_,
            this->object_direction_,
            this->object_max_dist_,
            this->target_frame_,
            this->camera_sensor_frame_pose_);

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

        std::vector<tPoint> points_on_robot_frame = this->frame_converter_.tfBottomIrSensor2RobotFrame(msg, this->ir_dist_center_to_sensor, this->ir_angle_sensor_to_next_sensor);

        if (this->target_frame_ == "map") {
            std::vector<tPoint> points_on_map_frame = this->frame_converter_.tfRobot2GlobalFrame(points_on_robot_frame, robot_pose);
            cloud = this->pointcloud_generator_.generatePointCloud2Message(points_on_map_frame, this->target_frame_);
        } else if (this->target_frame_ == "base_link") {
            cloud = this->pointcloud_generator_.generatePointCloud2Message(points_on_robot_frame, this->target_frame_);
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
    this->collision_sensor_frame_pose_.position = tPoint(
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
        << std::fixed << std::setprecision(5) << this->collision_sensor_frame_pose_.position.x << ", "
        << std::fixed << std::setprecision(5) << this->collision_sensor_frame_pose_.position.y << ", "
        << std::fixed << std::setprecision(5) << this->collision_sensor_frame_pose_.position.z << ")\n";
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

        tPoint point_on_robot_frame = this->frame_converter_.tfCollisionData2RobotFrame(msg, this->collision_sensor_frame_pose_.position.x);

        if (this->target_frame_ == "map") {
            std::vector<tPoint> points_on_map_frame = this->frame_converter_.tfRobot2GlobalFrame(point_on_robot_frame, robot_pose);
            cloud = this->pointcloud_generator_.generatePointCloud2Message(points_on_map_frame, this->target_frame_);
        } else if (this->target_frame_ == "base_link") {
            cloud = this->pointcloud_generator_.generatePointCloud2Message(point_on_robot_frame, this->target_frame_);
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