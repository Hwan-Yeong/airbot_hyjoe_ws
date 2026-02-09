#include "cloud_converter/cloud_converter.hpp"

#include "cloud_converter/cloud_converter_factory.hpp"
#include "sensor_manager_node.hpp"

namespace sensor_manager {

CloudConverterStrategy::CloudConverterStrategy(std::shared_ptr<SensorManagerNode> node_ptr)
    : node_ptr_(node_ptr)
{
    last_call_time_ = std::chrono::steady_clock::now();
    this->target_frame_ = node_ptr_->getTargetFrame();
}

PointCloudMsgVector CloudConverterStrategy::pc_convert(const void* sensor_msg)
{
    auto now = std::chrono::steady_clock::now();

    if (timeout_limit_sec_ > 0.0) {
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_call_time_);
        double duration_sec = duration.count();

        /**
         * @brief Converter 내부 변수 자동 초기화 기능
         *
         * @details 변수 자동 초기화는 크게 2가지 경우에 발생
         * 1) sensor_manager 노드 "on -> off -(over timeout)-> on" 시
         * 2) 센서 데이터 "수신 -> 미수신 -(over timeout)-> 수신" 시
        */
        if ((duration_sec >= timeout_limit_sec_)) {
            if (!is_already_reset_) {
            RCLCPP_WARN(this->node_ptr_->get_logger(),
                        "[%s] Data gap (%.1fs) exceeded reset timeout (%.1fs). Resetting...",
                        typeid(*this).name(), duration_sec, timeout_limit_sec_);
            reset_internal_variables();
            is_already_reset_ = true;
            }
        } else {
            is_already_reset_ = false;
        }
    }

    last_call_time_ = now;
    return pc_convert_impl(sensor_msg);
}

void CloudConverterStrategy::load_common_config(const YAML::Node& config) {
    if (!config.IsMap()) return;

    this->use_converter_ = config["use"] ? config["use"].as<bool>() : true;
    this->enable_target_frame_cloud_ = config["enable_target_frame_cloud"] ? config["enable_target_frame_cloud"].as<bool>() : false;
    this->enable_sensor_tf_cloud_ = config["enable_sensor_tf_cloud"] ? config["enable_sensor_tf_cloud"].as<bool>() : false;
    this->timeout_limit_sec_ = config["reset_timeout_sec"] ? config["reset_timeout_sec"].as<double>() : -1.0;

    try {
        if (config["extrinsics"]) {
            const auto& ext = config["extrinsics"];

            if (ext["tf_frame"]) { // TF Frame 정보가 있을 때만 로드
                this->parent_frame_ = ext["tf_frame"]["parent"] ? ext["tf_frame"]["parent"].as<std::string>() : "base_link";
                this->child_frame_ = ext["tf_frame"]["child"] ? ext["tf_frame"]["child"].as<std::string>() : "";
            }

            if (ext["translation"]) {
                this->sensor_extrinsic_.position.x = ext["translation"]["x"] ? ext["translation"]["x"].as<double>() : 0.0;
                this->sensor_extrinsic_.position.y = ext["translation"]["y"] ? ext["translation"]["y"].as<double>() : 0.0;
                this->sensor_extrinsic_.position.z = ext["translation"]["z"] ? ext["translation"]["z"].as<double>() : 0.0;
            }

            if (ext["rotation"]) {
                this->sensor_extrinsic_.orientation.roll  = DEG2RAD(ext["rotation"]["roll"] ? ext["rotation"]["roll"].as<double>() : 0.0);
                this->sensor_extrinsic_.orientation.pitch = DEG2RAD(ext["rotation"]["pitch"] ? ext["rotation"]["pitch"].as<double>() : 0.0);
                this->sensor_extrinsic_.orientation.yaw   = DEG2RAD(ext["rotation"]["yaw"] ? ext["rotation"]["yaw"].as<double>() : 0.0);
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Extrinsics parsing failed: %s", e.what());
    }
}

std::string CloudConverterStrategy::get_common_config_info(const std::string& sensor_type) {
    std::ostringstream oss;
    oss << "\n[" << sensor_type << " CONVERTER PARAMETERS]\n" << std::boolalpha
        << "  use_converter_            : " << this->use_converter_ << "\n"
        << "  enable_target_cloud       : " << this->enable_target_frame_cloud_ << "\n"
        << "  enable_sensor_tf          : " << this->enable_sensor_tf_cloud_ << "\n"
        << "  reset_timeout_sec         : " << this->timeout_limit_sec_ << "\n"
        << "  tf frame (Parent/Child)   : " << this->parent_frame_ << " / " << this->child_frame_ << "\n"
        << "  sensor_extrinsic_pose     : translation [m] x/y/z = ("
                                            << this->sensor_extrinsic_.position.x << ", "
                                            << this->sensor_extrinsic_.position.y << ", "
                                            << this->sensor_extrinsic_.position.z << ")\n"
        << "   ->(from base_link)       : rotation [deg] r/p/y = ("
                                            << RAD2DEG(this->sensor_extrinsic_.orientation.roll)  << ", "
                                            << RAD2DEG(this->sensor_extrinsic_.orientation.pitch)  << ", "
                                            << RAD2DEG(this->sensor_extrinsic_.orientation.yaw)  << ")\n";

    return oss.str();
}

std::optional<geometry_msgs::msg::TransformStamped> CloudConverterStrategy::get_static_tf()
{
    if (!enable_sensor_tf_cloud_ || parent_frame_.empty() || child_frame_.empty()) return std::nullopt;

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

std::shared_ptr<SensorManagerNode> CloudConverterStrategy::get_node_ptr() const
{
    return node_ptr_;
}

TofMonoCloudConverter::TofMonoCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node &config)
    : CloudConverterStrategy(node_ptr)
{
    if (!config.IsMap())
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
    }

    // TODO: Set Default Config
    //       => Yaml 파일이 깨질 경우를 대비하여 양산 시 확정된 사양의 기본값을 하드 코딩으로 채워넣기.

    // Load Config
    load_common_config(config);

    // Print Config
    std::ostringstream oss_1dtof;
    oss_1dtof << get_common_config_info("1D ToF");
    oss_1dtof << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr_->get_logger(), "%s", oss_1dtof.str().c_str());
}

PointCloudMsgVector TofMonoCloudConverter::pc_convert_impl(const void *sensor_msg)
{
    PointCloudMsg cloud;

    if (this->use_converter_)
    {
        auto msg = static_cast<const robot_custom_msgs::msg::TofData*>(sensor_msg);

        tPose robot_pose;
        robot_pose.position.x = msg->robot_x;
        robot_pose.position.y = msg->robot_y;
        robot_pose.orientation.yaw = msg->robot_angle;

        tPoint point_on_robot_frame = this->frame_converter_.tfMonoTofSensor2RobotFrame(msg->top, this->sensor_extrinsic_);

        if (this->target_frame_ == "map") {
            std::vector<tPoint> points_on_map_frame = this->frame_converter_.tfRobot2GlobalFrame(point_on_robot_frame, robot_pose);
            cloud = this->pointcloud_generator_.generatePointCloud2Message(points_on_map_frame, this->target_frame_);
        } else if (this->target_frame_ == "base_link") {
            cloud = this->pointcloud_generator_.generatePointCloud2Message({point_on_robot_frame}, this->target_frame_);
        } else {
            RCLCPP_INFO(this->node_ptr_->get_logger(), "Select Wrong Target Frame: %s", this->target_frame_.c_str());
        }
    }

    return {cloud};
}

TofMultiLeftCloudConverter::TofMultiLeftCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node &config)
    : CloudConverterStrategy(node_ptr)
{
    if (!config.IsMap())
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
    }

    // TODO: Set Default Config
    //       => Yaml 파일이 깨질 경우를 대비하여 양산 시 확정된 사양의 기본값을 하드 코딩으로 채워넣기.

    // Load Config
    load_common_config(config);
    this->tof_multi_left_fov_ = DEG2RAD(config["extrinsics"]["fov"].as<double>());
    for (auto idx : config["sub_cell_idx_array"]) {
        this->tof_multi_left_sub_cell_idx_array_.push_back(idx.as<int>());
    }
    if (config["extrinsics"] && config["extrinsics"]["fov"]) {
        this->tof_multi_left_fov_ = DEG2RAD(config["extrinsics"]["fov"].as<double>());
    }
    if (config["sub_cell_idx_array"]) {
        this->tof_multi_left_sub_cell_idx_array_.clear();
        for (auto idx : config["sub_cell_idx_array"]) {
            this->tof_multi_left_sub_cell_idx_array_.push_back(idx.as<int>());
        }
    }

    // Print Config
    std::ostringstream oss_mtofLeft;
    oss_mtofLeft << get_common_config_info("MULTI TOF (Left)");
    oss_mtofLeft << "  fov [deg]            : " << RAD2DEG(this->tof_multi_left_fov_) << "\n";
    oss_mtofLeft << "  sub_cell_idx_array   : ";
    for (auto idx : this->tof_multi_left_sub_cell_idx_array_) { oss_mtofLeft << idx << " "; }
    oss_mtofLeft << "\n";
    oss_mtofLeft << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr_->get_logger(), "%s", oss_mtofLeft.str().c_str());

    // Init Calculation
    tof_utils_.updateSubCellIndexArray(
        this->tof_multi_left_sub_cell_idx_array_,
        this->tof_multi_left_fov_,
        this->tof_multi_left_y_tan_array_, //output
        this->tof_multi_left_z_tan_array_, //output
        this->node_ptr_->get_logger()
    );
}

PointCloudMsgVector TofMultiLeftCloudConverter::pc_convert_impl(const void *sensor_msg)
{
    PointCloudMsgVector clouds;

    if (this->use_converter_)
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
            this->sensor_extrinsic_);

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
            RCLCPP_INFO(this->node_ptr_->get_logger(), "Select Wrong Target Frame: %s", this->target_frame_.c_str());
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
        this->sensor_extrinsic_);

    if (robot_pts.size() != INDEX_SIZE) {
        RCLCPP_WARN(this->node_ptr_->get_logger(),
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
        RCLCPP_WARN(this->node_ptr_->get_logger(),
            "robot_pts has fewer than 3 points (size=%zu). Returning empty array.",
            robot_pts.size()
        );
    }

    return ret;
}

TofMultiRightCloudConverter::TofMultiRightCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node &config)
    : CloudConverterStrategy(node_ptr)
{
    if (!config.IsMap())
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
    }

    // TODO: Set Default Config
    //       => Yaml 파일이 깨질 경우를 대비하여 양산 시 확정된 사양의 기본값을 하드 코딩으로 채워넣기.

    // Load Config
    load_common_config(config);
    this->tof_multi_right_fov_ = DEG2RAD(config["extrinsics"]["fov"].as<double>());
    for (auto idx : config["sub_cell_idx_array"]) {
        this->tof_multi_right_sub_cell_idx_array_.push_back(idx.as<int>());
    }
    if (config["extrinsics"] && config["extrinsics"]["fov"]) {
        this->tof_multi_right_fov_ = DEG2RAD(config["extrinsics"]["fov"].as<double>());
    }
    if (config["sub_cell_idx_array"]) {
        this->tof_multi_right_sub_cell_idx_array_.clear();
        for (auto idx : config["sub_cell_idx_array"]) {
            this->tof_multi_right_sub_cell_idx_array_.push_back(idx.as<int>());
        }
    }

    // Print Config
    std::ostringstream oss_mtofRight;
    oss_mtofRight << get_common_config_info("MULTI TOF (Right)");
    oss_mtofRight << "  fov [deg]            : " << RAD2DEG(this->tof_multi_right_fov_) << "\n";
    oss_mtofRight << "  sub_cell_idx_array   : ";
    for (auto idx : this->tof_multi_right_sub_cell_idx_array_) { oss_mtofRight << idx << " "; }
    oss_mtofRight << "\n";
    oss_mtofRight << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr_->get_logger(), "%s", oss_mtofRight.str().c_str());

    // Init Calculation
    tof_utils_.updateSubCellIndexArray(
        this->tof_multi_right_sub_cell_idx_array_,
        this->tof_multi_right_fov_,
        this->tof_multi_right_y_tan_array_, //output
        this->tof_multi_right_z_tan_array_, //output
        this->node_ptr_->get_logger()
    );
}

PointCloudMsgVector TofMultiRightCloudConverter::pc_convert_impl(const void *sensor_msg)
{
    PointCloudMsgVector clouds;

    if (this->use_converter_)
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
            this->sensor_extrinsic_);

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
            RCLCPP_INFO(this->node_ptr_->get_logger(), "Select Wrong Target Frame: %s", this->target_frame_.c_str());
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
        this->sensor_extrinsic_);

    if (robot_pts.size() != INDEX_SIZE) {
        RCLCPP_WARN(this->node_ptr_->get_logger(),
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
        RCLCPP_WARN(this->node_ptr_->get_logger(),
            "robot_pts has fewer than 3 points (size=%zu). Returning empty array.",
            robot_pts.size()
        );
    }

    return ret;
}

CameraCloudConverter::CameraCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node &config)
    : CloudConverterStrategy(node_ptr)
{
    if (!config.IsMap())
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
    }

    // TODO: Set Default Config
    //       => Yaml 파일이 깨질 경우를 대비하여 양산 시 확정된 사양의 기본값을 하드 코딩으로 채워넣기.

    // Load Config
    load_common_config(config);
    this->object_direction_ = config["object_direction"].as<bool>();
    this->pointcloud_resolution_ = config["pointcloud_resolution"].as<double>();
    this->object_max_dist_ = config["object_max_distance_m"].as<double>();
    this->object_ignore_pitch_th_ = DEG2RAD(config["object_ignore_pitch_th_deg"].as<double>());
    for (const auto& class_id: config["class_id_confidence_th"]) {
        auto item = class_id.as<std::string>();
        std::istringstream ss(item);
        std::string key, value;
        if (std::getline(ss, key, ':') && std::getline(ss, value)) {
            this->camera_class_id_confidence_th_[std::stoi(key)] = std::stoi(value);
        }
    }
    this->object_direction_ = config["object_direction"] ? config["object_direction"].as<bool>() : true;
    this->pointcloud_resolution_ = config["pointcloud_resolution"] ? config["pointcloud_resolution"].as<double>() : 0.05;
    this->object_max_dist_ = config["object_max_distance_m"] ? config["object_max_distance_m"].as<double>() : 1.5;
    this->object_ignore_pitch_th_ = DEG2RAD(config["object_ignore_pitch_th_deg"] ? config["object_ignore_pitch_th_deg"].as<double>() : 3.0);
    if (config["class_id_confidence_th"]) {
        for (const auto& class_id: config["class_id_confidence_th"]) {
            auto item = class_id.as<std::string>();
            std::istringstream ss(item);
            std::string key, value;
            if (std::getline(ss, key, ':') && std::getline(ss, value)) {
                this->camera_class_id_confidence_th_[std::stoi(key)] = std::stoi(value);
            }
        }
    }
    this->use_object_logger_ = config["logger"]["use"].as<bool>();
    this->object_logger_margin_distance_diff_m_ = config["logger"]["margin"]["distance_diff_m"].as<double>();
    if (config["logger"]) {
        this->use_object_logger_ = config["logger"]["use"] ? config["logger"]["use"].as<bool>() : true;
        if (config["logger"]["margin"]) {
            this->object_logger_margin_distance_diff_m_ = config["logger"]["margin"]["distance_diff_m"] ? config["logger"]["margin"]["distance_diff_m"].as<double>() : 1.0;
        }
    }

    // Print Config
    std::ostringstream oss_camera;
    oss_camera << get_common_config_info("CAMERA");
    oss_camera << "  object_direction_       : " << this->object_direction_ << "\n";
    oss_camera << "  pointcloud_resolution_  : " << std::fixed << std::setprecision(2) << this->pointcloud_resolution_ << "\n";
    oss_camera << "  object_max_dist_        : " << std::fixed << std::setprecision(2) << this->object_max_dist_ << " m\n";
    oss_camera << "  object_ignore_pitch_th_ : " << std::fixed << std::setprecision(2) << RAD2DEG(this->object_ignore_pitch_th_) << " deg\n";
    oss_camera << "  class_id_confidence_th  :\n";
    for (const auto& [class_id, confidence_th] : this->camera_class_id_confidence_th_) {
        oss_camera << "    - { id: " << std::setw(2) << std::setfill('0') << class_id << ", th: " << confidence_th << " }\n";
    }
    oss_camera << "  use_logger_             : " << this->use_object_logger_ << "\n";
    oss_camera << "  log_margin_dist_diff_m_ : " << std::fixed << std::setprecision(2) << this->object_logger_margin_distance_diff_m_ << "\n";
    oss_camera << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr_->get_logger(), "%s", oss_camera.str().c_str());

    // IMU Msg Subscriber
    setup_imu_subscription();
}

void CameraCloudConverter::setup_imu_subscription()
{
    imu_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_data", rclcpp::QoS(10).reliable(),
        [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            double roll, pitch, yaw;
            tf2::Quaternion quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
            tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
            if (abs(roll) >= this->object_ignore_pitch_th_ || abs(pitch) >= this->object_ignore_pitch_th_) {
                is_ramp_detection_ = true;
                ramp_release_cnt = 0;
            } else {
                if (is_ramp_detection_) {
                    ramp_release_cnt++;
                    if (ramp_release_cnt > 10) { // 1 sec
                        is_ramp_detection_ = false;
                        ramp_release_cnt = 0;
                    }
                }
            }
        }
    );
}

PointCloudMsgVector CameraCloudConverter::pc_convert_impl(const void *sensor_msg)
{
    PointCloudMsg cloud;

    if (this->use_converter_ && !this->is_ramp_detection_)
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
            this->sensor_extrinsic_,
            this->use_object_logger_,
            this->object_logger_margin_distance_diff_m_,
            this->node_ptr_->get_logger());

        bbox_array.header.stamp = this->node_ptr_->get_clock()->now();

        cloud = this->pointcloud_generator_.generateCameraPointCloud2Message(bbox_array, this->pointcloud_resolution_);
    }

    return {cloud};
}

BottomIrCloudConverter::BottomIrCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node &config)
    : CloudConverterStrategy(node_ptr)
{
    if (!config.IsMap())
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
    }

    // TODO: Set Default Config
    //       => Yaml 파일이 깨질 경우를 대비하여 양산 시 확정된 사양의 기본값을 하드 코딩으로 채워넣기.

    // Load Config
    load_common_config(config);
    if (config["extrinsics"]) {
        this->ir_dist_center_to_sensor_ = config["extrinsics"]["distance"] ? config["extrinsics"]["distance"].as<double>() : 0.15;
        this->ir_angle_sensor_to_next_sensor_ = config["extrinsics"]["angle"] ? config["extrinsics"]["angle"].as<double>() : 50.0;
    }

    // Print Config
    std::ostringstream oss_bottomIr;
    oss_bottomIr << get_common_config_info("BOTTOM IR");
    oss_bottomIr << "  ir_dist_center_to_sensor_ [m]          : " << this->ir_dist_center_to_sensor_ << "\n";
    oss_bottomIr << "  ir_angle_sensor_to_next_sensor_ [deg]  : " << this->ir_angle_sensor_to_next_sensor_ <<" \n";
    oss_bottomIr << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr_->get_logger(), "%s", oss_bottomIr.str().c_str());
}

PointCloudMsgVector BottomIrCloudConverter::pc_convert_impl(const void *sensor_msg)
{
    PointCloudMsg cloud;

    if (this->use_converter_)
    {
        auto msg = static_cast<const robot_custom_msgs::msg::BottomIrData*>(sensor_msg);

        tPose robot_pose;
        robot_pose.position.x = msg->robot_x;
        robot_pose.position.y = msg->robot_y;
        robot_pose.orientation.yaw = msg->robot_angle;

        std::vector<tPoint> points_on_robot_frame = this->frame_converter_.tfBottomIrSensor2RobotFrame(msg, this->ir_dist_center_to_sensor_, this->ir_angle_sensor_to_next_sensor_);

        if (this->target_frame_ == "map") {
            std::vector<tPoint> points_on_map_frame = this->frame_converter_.tfRobot2GlobalFrame(points_on_robot_frame, robot_pose);
            cloud = this->pointcloud_generator_.generatePointCloud2Message(points_on_map_frame, this->target_frame_);
        } else if (this->target_frame_ == "base_link") {
            cloud = this->pointcloud_generator_.generatePointCloud2Message(points_on_robot_frame, this->target_frame_);
        } else {
            RCLCPP_INFO(this->node_ptr_->get_logger(), "Select Wrong Target Frame: %s", this->target_frame_.c_str());
        }
    }

    return {cloud};
}

CollisionCloudConverter::CollisionCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node &config)
    : CloudConverterStrategy(node_ptr)
{
    if (!config.IsMap())
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
    }

    // TODO: Set Default Config
    //       => Yaml 파일이 깨질 경우를 대비하여 양산 시 확정된 사양의 기본값을 하드 코딩으로 채워넣기.

    // Load Config
    load_common_config(config);
    if (config["extrinsics"]) {
        this->sensor_extrinsic_.position = tPoint(config["extrinsics"]["distance"].as<double>(), 0.0, 0.0);
    }

    // Print Config
    std::ostringstream oss_collision;
    oss_collision << get_common_config_info("COLLISION");
    oss_collision << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr_->get_logger(), "%s", oss_collision.str().c_str());
}

PointCloudMsgVector CollisionCloudConverter::pc_convert_impl(const void *sensor_msg)
{
    PointCloudMsg cloud;

    if (this->use_converter_)
    {
        auto msg = static_cast<const robot_custom_msgs::msg::AbnormalEventData*>(sensor_msg);

        tPose robot_pose;
        robot_pose.position.x = msg->robot_x;
        robot_pose.position.y = msg->robot_y;
        robot_pose.orientation.yaw = msg->robot_angle;

        tPoint point_on_robot_frame = this->frame_converter_.tfCollisionData2RobotFrame(msg, this->sensor_extrinsic_.position.x);

        if (this->target_frame_ == "map") {
            std::vector<tPoint> points_on_map_frame = this->frame_converter_.tfRobot2GlobalFrame(point_on_robot_frame, robot_pose);
            cloud = this->pointcloud_generator_.generatePointCloud2Message(points_on_map_frame, this->target_frame_);
        } else if (this->target_frame_ == "base_link") {
            cloud = this->pointcloud_generator_.generatePointCloud2Message(point_on_robot_frame, this->target_frame_);
        } else {
            RCLCPP_INFO(this->node_ptr_->get_logger(), "Select Wrong Target Frame: %s", this->target_frame_.c_str());
        }
    }

    return {cloud};
}

EmptyCloudConverter::EmptyCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node& config)
    : CloudConverterStrategy(node_ptr)
{
    if (!config.IsMap())
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
    }

    // Load Config
    load_common_config(config);

    // Print Config
    std::ostringstream oss_empty;
    oss_empty << get_common_config_info("EMPTY");
    oss_empty << "----------------------------------------------------";
    RCLCPP_INFO(this->node_ptr_->get_logger(), "%s", oss_empty.str().c_str());
}

PointCloudMsgVector EmptyCloudConverter::pc_convert_impl(const void *sensor_msg)
{
    PointCloudMsg cloud;

    if (sensor_msg == nullptr && this->use_converter_) {
        cloud = this->pointcloud_generator_.generateEmptyPointCloud2Message(this->target_frame_);
    }

    return {cloud};
}

} // namespace sensor_manager