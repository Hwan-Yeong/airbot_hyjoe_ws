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

sensor_msgs::msg::PointCloud2 PointCloudGenerator::generatePointCloud2EmptyMessage(const std::string &frame)
{
    sensor_msgs::msg::PointCloud2 msg;

    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = frame;

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

/**
 * @input: vision_msgs::msg::BoundingBox2DArray
 * ### Convert Camera Data
 */
sensor_msgs::msg::PointCloud2 PointCloudGenerator::generatePointCloud2Message(const vision_msgs::msg::BoundingBox2DArray input_bbox_array, float resolution, const std::vector<int> &bbox_ai_ids, const geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg)
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

    if(input_bbox_array.boxes.size() != bbox_ai_ids.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("PointCloud"), "Mismatch between BoundingBox count and AI ID count![%ld|%ld]", input_bbox_array.boxes.size(), bbox_ai_ids.size());
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
    
    
    const float safety_min_x = init_pose_msg.pose.pose.position.x + -0.4f;
    const float safety_max_x = init_pose_msg.pose.pose.position.x + 0.4f;
    const float safety_min_y = init_pose_msg.pose.pose.position.y + -0.4f;
    const float safety_max_y = init_pose_msg.pose.pose.position.y + 0.4f;
    size_t filtered_points = 0;
    auto cam_object_it = bbox_ai_ids.cbegin();

    msg.data.resize(msg.row_step);
    size_t max_size = msg.data.size();
    uint8_t* ptr = msg.data.data();
    for (const auto& box : input_bbox_array.boxes) {
        const auto& cam_object_id = *cam_object_it;
        const double center_x = box.center.position.x;
        const double center_y = box.center.position.y;
        const double size_x = box.size_x;
        const double size_y = box.size_y;

        point_size_x = static_cast<int>(size_x*1000)/static_cast<int>(resolution*1000) + 1;
        point_size_y = static_cast<int>(size_y*1000)/static_cast<int>(resolution*1000) + 1;

        for (int i = 0; i < point_size_x; ++i) {
            for (int j = 0; j < point_size_y; ++j) {
                if (i == 0 || i == point_size_x - 1 || j == 0 || j == point_size_y - 1) { // 테두리 조건: x축 가장자리 or y축 가장자리일 때만 point 생성
                    
                    float x = (center_x - size_x/2) + i*resolution;
                    float y = (center_y - size_y/2) + j*resolution;
                    float z = 0.0f;

                    // min_x <= x <= max_x
                    // min_y <= y <= max_y, box포인트가 safetybox 안에 들어오면 skip
                    if (cam_object_id == 0 && (x >= safety_min_x && x <= safety_max_x && y >= safety_min_y && y <= safety_max_y)) {
                        continue;
                    }

                    size_t current_offset = static_cast<size_t>(ptr - msg.data.data());
                    if (current_offset + msg.point_step > max_size) {
                        return msg;
                    }

                    memcpy(ptr, &x, sizeof(float));
                    memcpy(ptr + 4, &y, sizeof(float));
                    memcpy(ptr + 8, &z, sizeof(float));
                    ptr += msg.point_step;
                    filtered_points++;        
                }
            }
        }
        ++cam_object_it;
    }
    // filter처리된 point 만큼 data resize.
    msg.width = filtered_points; 
    msg.row_step = msg.width * msg.point_step;
    msg.data.resize(msg.row_step);

    return msg;
}

sensor_msgs::msg::PointCloud2 PointCloudGenerator::mergePointCloud2Vector(const std::vector<sensor_msgs::msg::PointCloud2> &pc_msgs, std::string frame)
{
    pcl::PointCloud<pcl::PointXYZ> merged_cloud;

    for (const auto& pc : pc_msgs) {
        pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
        if (!pc.fields.empty()) {
            pcl::fromROSMsg(pc, tmp_cloud);
        }
        merged_cloud += tmp_cloud;
    }

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(merged_cloud, output_msg);
    output_msg.header.frame_id = frame;
    output_msg.header.stamp = rclcpp::Clock().now();

    return output_msg;
}