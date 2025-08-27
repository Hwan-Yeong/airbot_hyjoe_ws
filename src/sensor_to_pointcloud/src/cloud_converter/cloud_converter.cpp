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
    this->tof_mono_sensor_frame_pitch_cosine = std::cos(this->sensor_frame_pose_.orientation.pitch);
    this->tof_mono_sensor_frame_pitch_sine = std::sin(this->sensor_frame_pose_.orientation.pitch);

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

        sensor_msgs::msg::PointCloud2 pc_msg = generateTofMonoPointCloudMsg(msg, robot_pose);

        ret = pc_msg;
    }

    return ret;
}

sensor_msgs::msg::PointCloud2 TofMonoCloudConverter::generateTofMonoPointCloudMsg(const robot_custom_msgs::msg::TofData* input_msg, tPose &robot_pose)
{
    sensor_msgs::msg::PointCloud2 ret;

    tPoint point_on_robot_frame;
    point_on_robot_frame.x = this->sensor_frame_pose_.position.x + input_msg->top * this->tof_mono_sensor_frame_pitch_cosine;
    point_on_robot_frame.y = this->sensor_frame_pose_.position.y;
    point_on_robot_frame.z = this->sensor_frame_pose_.position.z - input_msg->top * this->tof_mono_sensor_frame_pitch_sine;

    if (this->target_frame_ == "map") {
        std::vector<tPoint> point_on_map_frame = this->frame_converter_.tfRobot2GlobalFrame({point_on_robot_frame}, robot_pose);
        ret = this->pointcloud_generator_.generatePointCloud2Message(point_on_map_frame, this->target_frame_);
    } else {
        ret = this->pointcloud_generator_.generatePointCloud2Message({point_on_robot_frame}, this->target_frame_);
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

        vision_msgs::msg::BoundingBox2DArray bbox_array = generateObjectBBoxArray(msg, robot_pose, this->camera_class_id_confidence_th_, this->object_direction_, this->object_max_dist_);

        sensor_msgs::msg::PointCloud2 pc_msg = this->pointcloud_generator_.generateCameraPointCloud2Message(bbox_array, this->pointcloud_resolution_);

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
                // point_on_sensor_frame.z = -this->sensor_frame_pose_.position.z;

                point_on_robot_frame.x = point_on_sensor_frame.x + this->sensor_frame_pose_.position.x;
                point_on_robot_frame.y = point_on_sensor_frame.y + this->sensor_frame_pose_.position.y;
                // point_on_robot_frame.z = point_on_sensor_frame.z + this->sensor_frame_pose_.position.z;
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