#include "utils/pointcloud_generator.hpp"


PointCloudGenerator::PointCloudGenerator()
{
}

PointCloudGenerator::~PointCloudGenerator()
{
}

/**
 * @input: std::vector<tPoint>
 * ### Convert ToF, Cliff(IR) Data
 */
sensor_msgs::msg::PointCloud2 PointCloudGenerator::generatePointCloud2Message(const std::vector<tPoint> &points, std::string frame)
{
    sensor_msgs::msg::PointCloud2 msg;

    if (points.empty()) {
        // RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Input data is empty!");
        return msg;
    }

    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = frame;

    msg.height = 1;
    msg.width = points.size();                  // 입력받은 points의 개수
    msg.is_dense = false;                       // True if there are no invalid points
    msg.is_bigendian = false;                   // Is this data bigendian?
    msg.point_step = 12;                        // 각 포인트가 차지하는 bytes (8 bytes * 3 (x, y, z))
    msg.row_step = msg.point_step * msg.width;  // Length of a row in bytes (전체 bytes 크기)

    // Define fields (x, y, z)
    sensor_msgs::msg::PointField field_x, field_y, field_z;
    field_x.name = "x";                                         // Name of field
    field_x.offset = 0;                                         // Offset from start of point struct
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;   // Datatype enumeration, see "PointField.msg"
    field_x.count = 1;                                          // 각 포인트에 저장할 값의 개수 (1이 정상적인 값임)

    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;

    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;

    msg.fields = {field_x, field_y, field_z};

    // Fill point data
    // 실제 Points들의 데이터(바이너리 값들)를 저장하는 곳
    msg.data.resize(msg.row_step);                   // 전체 bytes 크기로 데이터 버퍼 생성
    uint8_t* data_ptr = msg.data.data();
    for (const auto& point : points) {
        float x = static_cast<float>(point.x);
        float y = static_cast<float>(point.y);
        float z = static_cast<float>(point.z);
        std::memcpy(data_ptr, &x, sizeof(float));        // x
        std::memcpy(data_ptr + 4, &y, sizeof(float));    // y
        std::memcpy(data_ptr + 8, &z, sizeof(float));    // z
        data_ptr += msg.point_step;
    }
    // RCLCPP_INFO(rclcpp::get_logger("PointCloud"), "Msg Data Size: %zu", msg.data.size());

    return msg;
}

/**
 * @input: vision_msgs::msg::BoundingBox2DArray
 * ### Convert Camera Data
 */
sensor_msgs::msg::PointCloud2 PointCloudGenerator::generatePointCloud2Message(const vision_msgs::msg::BoundingBox2DArray input_bbox_array, float resolution)
{
    sensor_msgs::msg::PointCloud2 msg;

    if (input_bbox_array.boxes.empty()) {
        // RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Input data is empty!");
        return msg;
    }
    if (resolution <= 0) {
        // RCLCPP_ERROR(rclcpp::get_logger("PointCloud"), "Invalid resolution: %f", resolution);
        return msg;
    }

    size_t total_points = 0;
    int point_size_x, point_size_y;
    for (const auto& box : input_bbox_array.boxes) {
        if (box.size_x <= 0 || box.size_y <= 0 ) { // width, height 음수인 경우는 예외처리 (계산 망가짐)
            return msg;
        }
        point_size_x = static_cast<int>(box.size_x/resolution) + 1;
        point_size_y = static_cast<int>(box.size_y/resolution) + 1;
        total_points += point_size_x * point_size_y;
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

        for (int i = 0; i < point_size_x; ++i) {
            for (int j = 0; j < point_size_y; ++j) {
                size_t current_offset = static_cast<size_t>(ptr - msg.data.data());
                if (current_offset + msg.point_step > max_size) {
                    // RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "# IGNORE! # Memory overflow detected! #");
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

    return msg;
}