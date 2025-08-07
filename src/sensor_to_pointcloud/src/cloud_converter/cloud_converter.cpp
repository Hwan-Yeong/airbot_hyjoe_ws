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
    this->sensor_frame_translation_ = tPoint(
                                                config["extrinsics"]["translation"]["x"].as<double>(),
                                                config["extrinsics"]["translation"]["y"].as<double>(),
                                                config["extrinsics"]["translation"]["z"].as<double>()
                                            );

    // Print Config
    std::ostringstream oss;
    oss << "\n==== CAMERA POINTCLOUD CONVERTER PARAMETERS ====\n";
    oss << std::boolalpha;
    oss << "  use_camera_: " << this->use_camera_ << "\n";
    oss << "  Pitch Angle: " << std::fixed << std::setprecision(2) << this->pointcloud_resolution_ << "\n";
    oss << "  Camera Class ID Confidence Threshold:\n";
    for (const auto& [class_id, confidence_th] : this->camera_class_id_confidence_th_) {
        oss << "    - ID: " << class_id << ", Threshold: " << confidence_th << "\n";
    }
    oss <<   "================================================";
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

        vision_msgs::msg::BoundingBox2DArray bbox_array = generateObjectBBoxArray(msg, robot_pose, this->camera_class_id_confidence_th_, this->object_direction_, this->object_max_dist_);

        sensor_msgs::msg::PointCloud2 pc_msg = generateCameraPointCloudMsg(bbox_array, this->pointcloud_resolution_);

        ret = pc_msg;
    }

    return ret;
}

vision_msgs::msg::BoundingBox2DArray CameraCloudConverter::generateObjectBBoxArray(const robot_custom_msgs::msg::CameraDataArray* msg, tPose &robot_pose, std::map<int, int> class_id_confidence_th, bool direction, double object_max_distance)
{
    auto bbox_array = vision_msgs::msg::BoundingBox2DArray();

    double robot_cos = std::cos(robot_pose.orientation.yaw);
    double robot_sin = std::sin(robot_pose.orientation.yaw);

    if (msg->data_array.empty() || msg->num == 0) {
        // RCLCPP_INFO(this->node_ptr->get_logger(), "Input data is empty!");
        return bbox_array;
    }

    bbox_array.header.stamp = rclcpp::Clock().now();
    bbox_array.header.frame_id = this->target_frame_;
    // bbox_array.header.frame_id = "map";

    std::vector<robot_custom_msgs::msg::CameraData> objects(msg->data_array.begin(), msg->data_array.end());
    for (const auto &obj : objects)
    {
        if (obj.distance > object_max_distance) continue; // 객체인식 장애물 최대 거리 제한
        auto it = class_id_confidence_th.find(obj.id);
        if (it != class_id_confidence_th.end() && static_cast<int>(obj.score) >= it->second) { // data filtering with "class id", "confidencd score"
            if (obj.height >= 0.0 && obj.width >= 0.0) {
                auto bbox = vision_msgs::msg::BoundingBox2D();

                tPoint point_on_sensor_frame, point_on_robot_frame;
                /*
                    객체의 너비(가로폭)가 30cm 이하인 경우, 높이를 너비와 동일한 -> 정사각형 객체로 가공
                    객체의 너비(가로폭)가 30cm 이상인 경우, 높이를 30cm로 고정
                */
                double height = std::min(static_cast<double>(obj.width), 0.3);

                if (direction) {
                    point_on_sensor_frame.x = obj.distance * std::cos(obj.theta) + height/2;
                    point_on_sensor_frame.y = obj.distance * std::sin(obj.theta);
                } else {
                    point_on_sensor_frame.x = obj.distance * std::cos(-obj.theta) + height/2;
                    point_on_sensor_frame.y = obj.distance * std::sin(-obj.theta);
                }
                // point_on_sensor_frame.z = -this->sensor_frame_translation_.z;

                point_on_robot_frame.x = point_on_sensor_frame.x + this->sensor_frame_translation_.x;
                point_on_robot_frame.y = point_on_sensor_frame.y + this->sensor_frame_translation_.y;
                // point_on_robot_frame.z = point_on_sensor_frame.z + this->sensor_frame_translation_.z;
                if (this->target_frame_ == "map") {
                    bbox.center.position.x = point_on_robot_frame.x*robot_cos - point_on_robot_frame.y*robot_sin + robot_pose.position.x;
                    bbox.center.position.y = point_on_robot_frame.x*robot_sin + point_on_robot_frame.y*robot_cos + robot_pose.position.y;
                } else if (this->target_frame_ == "base_link") {
                    bbox.center.position.x = point_on_robot_frame.x;
                    bbox.center.position.y = point_on_robot_frame.y;
                } else {
                    RCLCPP_INFO(this->node_ptr->get_logger(), "Select Wrong Target Frame: %s", target_frame_.c_str());
                }
                bbox.center.theta = 0.0;
                bbox.size_x = height;
                /*
                    pole (의자 다리) 객체의 경우,
                    객체의 너비(가로폭)가 50cm 이하인 경우, 너비를 50cm로 고정
                    객체의 너비(가로폭)가 50cm 이상인 경우, 인식된 너비 그대로 사용
                */
                // if (obj.id == 12) { // pole
                //     bbox.size_y = obj.width < 0.5 ? 0.5 : obj.width;
                // } else {
                //     bbox.size_y = obj.width;
                // }

                /*
                    bed (침대) 객체의 경우,
                    객체의 최대 너비(가로폭)를 55cm로 제한
                */
                if (obj.id == 17) { // bed
                    bbox.size_y = std::min(obj.width, 0.55);
                } else {
                    bbox.size_y = obj.width;
                }
                bbox_array.boxes.push_back(bbox);
            }
        } else {
            // RCLCPP_INFO(this->node_ptr->get_logger(),
            //     "[Camera Filtered Data] ID: %d, SCORE: %d",
            //     static_cast<int>(obj.id), static_cast<int>(obj.score)
            // );
        }
    }

    return bbox_array;
}

sensor_msgs::msg::PointCloud2 CameraCloudConverter::generateCameraPointCloudMsg(const vision_msgs::msg::BoundingBox2DArray input_bbox_array, float resolution)
{
    sensor_msgs::msg::PointCloud2 msg;

    if (input_bbox_array.boxes.empty()) {
        // RCLCPP_WARN(this->node_ptr->get_logger(), "Input data is empty!");
        return msg;
    }
    if (resolution <= 0) {
        // RCLCPP_ERROR(this->node_ptr->get_logger(), "Invalid resolution: %f", resolution);
        return msg;
    }

    size_t total_points = 0;
    int point_size_x, point_size_y;
    for (const auto& box : input_bbox_array.boxes) {
        if (box.size_x <= 0 || box.size_y <= 0 ) { // width, height 음수인 경우는 예외처리 (계산 망가짐)
            return msg;
        }
        point_size_x = static_cast<int>(box.size_x*1000)/static_cast<int>(resolution*1000) + 1;
        point_size_y = static_cast<int>(box.size_y*1000)/static_cast<int>(resolution*1000) + 1;
        if (point_size_x == 1 && point_size_y == 1) {
            total_points += 1;
        } else {
            total_points += 2*point_size_x + 2*(point_size_y-2);
        }
    }

    msg.header = input_bbox_array.header;

    msg.height = 1;
    msg.width = total_points;
    msg.is_dense = false;
    msg.is_bigendian = false;
    msg.point_step = 12;
    msg.row_step = msg.width * msg.point_step;

    sensor_msgs::msg::PointField field_x, field_y, field_z;
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;

    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;

    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;

    msg.fields = {field_x, field_y, field_z};

    msg.data.resize(msg.row_step);
    size_t max_size = msg.data.size();
    uint8_t* ptr = msg.data.data();
    for (const auto& box : input_bbox_array.boxes) {
        const double center_x = box.center.position.x;
        const double center_y = box.center.position.y;
        const double size_x = box.size_x;
        const double size_y = box.size_y;

        point_size_x = static_cast<int>(box.size_x*1000)/static_cast<int>(resolution*1000) + 1;
        point_size_y = static_cast<int>(box.size_y*1000)/static_cast<int>(resolution*1000) + 1;

        for (int i = 0; i < point_size_x; ++i) {
            for (int j = 0; j < point_size_y; ++j) {
                if (i == 0 || i == point_size_x - 1 || j == 0 || j == point_size_y - 1) { // 테두리 조건: x축 가장자리 or y축 가장자리일 때만 point 생성
                    size_t current_offset = static_cast<size_t>(ptr - msg.data.data());
                    if (current_offset + msg.point_step > max_size) {
                        return msg;
                    }

                    float x = (center_x - size_x/2) + i*resolution;
                    float y = (center_y - size_y/2) + j*resolution;
                    float z = 0.0f;
                    memcpy(ptr, &x, sizeof(float));
                    memcpy(ptr + 4, &y, sizeof(float));
                    memcpy(ptr + 8, &z, sizeof(float));
                    ptr += msg.point_step;
                }
            }
        }
    }

    return msg;
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
    oss << "\n==== EMPTY POINTCLOUD PARAMETERS ====\n";
    oss << std::boolalpha;
    oss << "  use_empty_: " << this->use_empty_msg_ << "\n";
    oss <<   "=====================================";
    RCLCPP_INFO(this->node_ptr->get_logger(), "%s", oss.str().c_str());
}

sensor_msgs::msg::PointCloud2 EmptyCloudConverter::pc_convert(const void *sensor_msg)
{
    sensor_msgs::msg::PointCloud2 ret;

    if (sensor_msg == nullptr && this->use_empty_msg_) {
        ret = generateEmptyPointCloudMsg();
    }

    return ret;
}

sensor_msgs::msg::PointCloud2 EmptyCloudConverter::generateEmptyPointCloudMsg()
{
    sensor_msgs::msg::PointCloud2 msg;

    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = this->target_frame_;

    msg.height = 1;                // 단일 행
    msg.width = 0;                 // 데이터 포인트 0개
    msg.is_dense = true;           // 빈 메시지이므로 dense라고 봐도 무방
    msg.is_bigendian = false;
    msg.point_step = 12;           // 3 floats * 4 bytes
    msg.row_step = msg.point_step * msg.width;  // 0

    // 필드 정의 (x, y, z)
    sensor_msgs::msg::PointField field_x;
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;

    sensor_msgs::msg::PointField field_y;
    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;

    sensor_msgs::msg::PointField field_z;
    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;

    msg.fields = {field_x, field_y, field_z};

    msg.data.clear();

    return msg;
}

} // namespace sensor_to_pointcloud