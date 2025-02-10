#include "airbot_sensor_to_pointcloud/pointcloud.hpp"
#include "rclcpp/rclcpp.hpp"

#define USE_Camera_LOG false

PointCloud::PointCloud(float robot_radius = 0.3,
                       float tof_top_sensor_frame_x_translate = 0.0942,
                       float tof_top_sensor_frame_y_translate = 0.0,
                       float tof_tof_sensor_frame_z_translate = 0.56513,
                       float tof_top_sensor_frame_pitch_ang = 33,
                       float tof_bot_sensor_frame_x_translate = 0.145,
                       float tof_bot_sensor_frame_y_translate = 0.076,
                       float tof_bot_sensor_frame_z_translate = 0.03,
                       float tof_bot_left_sensor_frame_yaw_ang = 15.0,
                       float tof_bot_right_sensor_frame_yaw_ang = -15.0,
                       float tof_bot_fov_ang = 45,
                       float camera_sensor_frame_x_translate = 0.15473,
                       float camera_sensor_frame_y_translate = 0.0,
                       float camera_sensor_frame_z_translate = 0.5331)
    : robot_radius_(robot_radius),
      tof_top_sensor_frame_x_translate_(tof_top_sensor_frame_x_translate),
      tof_top_sensor_frame_y_translate_(tof_top_sensor_frame_y_translate),
      tof_tof_sensor_frame_z_translate_(tof_tof_sensor_frame_z_translate),
      tof_top_sensor_frame_pitch_ang_(tof_top_sensor_frame_pitch_ang),
      tof_bot_sensor_frame_x_translate_(tof_bot_sensor_frame_x_translate),
      tof_bot_sensor_frame_y_translate_(tof_bot_sensor_frame_y_translate),
      tof_bot_sensor_frame_z_translate_(tof_bot_sensor_frame_z_translate),
      tof_bot_left_sensor_frame_yaw_ang_(tof_bot_left_sensor_frame_yaw_ang),
      tof_bot_right_sensor_frame_yaw_ang_(tof_bot_right_sensor_frame_yaw_ang),
      tof_bot_fov_ang_(tof_bot_fov_ang),
      camera_sensor_frame_x_translate_(camera_sensor_frame_x_translate),
      camera_sensor_frame_y_translate_(camera_sensor_frame_y_translate),
      camera_sensor_frame_z_translate_(camera_sensor_frame_z_translate)
{
    tof_top_sensor_frame_pitch_cosine_ = std::cos(tof_top_sensor_frame_pitch_ang_*M_PI/180);
    tof_top_sensor_frame_pitch_sine_ = std::sin(tof_top_sensor_frame_pitch_ang_*M_PI/180);

    tof_left_sensor_frame_yaw_cosine_ = std::cos(tof_bot_left_sensor_frame_yaw_ang_*M_PI/180);
    tof_left_sensor_frame_yaw_sine_ = std::sin(tof_bot_left_sensor_frame_yaw_ang_*M_PI/180);
    tof_right_sensor_frame_yaw_cosine_ = std::cos(tof_bot_right_sensor_frame_yaw_ang_*M_PI/180);
    tof_right_sensor_frame_yaw_sine_ = std::sin(tof_bot_right_sensor_frame_yaw_ang_*M_PI/180);

    tof_bot_row_1_z_tan_ = std::tan(tof_bot_fov_ang_*(3.0/8.0)*M_PI/180);
    tof_bot_row_2_z_tan_ = std::tan(tof_bot_fov_ang_*(1.0/8.0)*M_PI/180);
    tof_bot_row_3_z_tan_ = std::tan(-tof_bot_fov_ang_*(1.0/8.0)*M_PI/180);
    tof_bot_row_4_z_tan_ = std::tan(-tof_bot_fov_ang_*(3.0/8.0)*M_PI/180);

    tof_bot_col_1_xy_tan_ = std::tan(tof_bot_fov_ang_*(3.0/8.0)*M_PI/180);
    tof_bot_col_2_xy_tan_ = std::tan(tof_bot_fov_ang_*(1.0/8.0)*M_PI/180);
    tof_bot_col_3_xy_tan_ = std::tan(-tof_bot_fov_ang_*(1.0/8.0)*M_PI/180);
    tof_bot_col_4_xy_tan_ = std::tan(-tof_bot_fov_ang_*(3.0/8.0)*M_PI/180);

    camera_bbox_array = vision_msgs::msg::BoundingBox2DArray();
}

PointCloud::~PointCloud()
{
}

void PointCloud::updateTargetFrame(std::string &updated_frame)
{
    target_frame = updated_frame;
}

void PointCloud::updateRobotPose(tPose &pose)
{
    robotPose = pose;
#if 0
    RCLCPP_INFO(rclcpp::get_logger("PointCloud"),
        "AMCL UPDATE! [x,y]: [%f,%f]",
        robotPose.position.x, robotPose.position.y);
#endif
}

sensor_msgs::msg::PointCloud2 PointCloud::getConvertedTofTopToPointCloud(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    tPoint point_on_robot_frame, point_on_map_frame;
    point_on_robot_frame.x = tof_top_sensor_frame_x_translate_ + msg->top * tof_top_sensor_frame_pitch_cosine_;
    point_on_robot_frame.y = tof_top_sensor_frame_y_translate_;
    point_on_robot_frame.z = tof_tof_sensor_frame_z_translate_ + msg->top * tof_top_sensor_frame_pitch_sine_;

    if (target_frame == "map") {
        std::vector<tPoint> points_on_map_frame = transformRobot2GlobalFrame(point_on_robot_frame);
        point_on_map_frame = points_on_map_frame.front();
    }

    if (target_frame == "map") {
        return createTofTopPointCloud2Message(point_on_map_frame);
    } else if (target_frame == "base_link") {
        return createTofTopPointCloud2Message(point_on_robot_frame);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Select Wrong Target Frame: %s", target_frame.c_str());
        return sensor_msgs::msg::PointCloud2();
    }
}

sensor_msgs::msg::PointCloud2 PointCloud::getConvertedTofBotToPointCloud(const robot_custom_msgs::msg::TofData::SharedPtr msg,
                                                                         TOF_SIDE side, bool debug, ROW_NUMBER row)
{
    std::vector<tPoint> multi_tof_points_on_sensor_frame;
    std::vector<tPoint> multi_left_tof_points_on_sensor_frame;
    std::vector<tPoint> multi_right_tof_points_on_sensor_frame;
    std::vector<tPoint> multi_tof_points_on_robot_frame;
    std::vector<tPoint> multi_left_tof_points_on_robot_frame;
    std::vector<tPoint> multi_right_tof_points_on_robot_frame;
    std::vector<tPoint> multi_tof_points_on_map_frame;
    std::vector<double> multi_tof_bot_left(msg->bot_left.begin(), msg->bot_left.end());
    std::vector<double> multi_tof_bot_right(msg->bot_right.begin(), msg->bot_right.end());
    switch(side)
    {
    case TOF_SIDE::LEFT:
        zero_dist_index = std::vector<bool>(16,false);
        multi_tof_points_on_sensor_frame = transformTofMsg2PointsOnSensorFrame(multi_tof_bot_left, false);
        multi_tof_points_on_robot_frame = transformTofSensor2RobotFrame(multi_tof_points_on_sensor_frame, true);
        if (target_frame == "map") {
            multi_tof_points_on_map_frame = transformRobot2GlobalFrame(multi_tof_points_on_robot_frame);
        }
        break;
    case TOF_SIDE::RIGHT:
        zero_dist_index = std::vector<bool>(16,false);
        multi_tof_points_on_sensor_frame = transformTofMsg2PointsOnSensorFrame(multi_tof_bot_right, false);
        multi_tof_points_on_robot_frame = transformTofSensor2RobotFrame(multi_tof_points_on_sensor_frame, false);
        if (target_frame == "map") {
            multi_tof_points_on_map_frame = transformRobot2GlobalFrame(multi_tof_points_on_robot_frame);
        }
        break;
    case TOF_SIDE::BOTH:
        zero_dist_index = std::vector<bool>(32,false);
        multi_left_tof_points_on_sensor_frame = transformTofMsg2PointsOnSensorFrame(multi_tof_bot_left, false);
        multi_right_tof_points_on_sensor_frame = transformTofMsg2PointsOnSensorFrame(multi_tof_bot_right, true);
        multi_left_tof_points_on_robot_frame = transformTofSensor2RobotFrame(multi_left_tof_points_on_sensor_frame, true);
        multi_right_tof_points_on_robot_frame = transformTofSensor2RobotFrame(multi_right_tof_points_on_sensor_frame, false);
        multi_tof_points_on_robot_frame.insert(multi_tof_points_on_robot_frame.end(), 
                                          multi_left_tof_points_on_robot_frame.begin(), 
                                          multi_left_tof_points_on_robot_frame.end());
        multi_tof_points_on_robot_frame.insert(multi_tof_points_on_robot_frame.end(), 
                                          multi_right_tof_points_on_robot_frame.begin(), 
                                          multi_right_tof_points_on_robot_frame.end());
        if (target_frame == "map") {
            multi_tof_points_on_map_frame = transformRobot2GlobalFrame(multi_tof_points_on_robot_frame);
        }
        break;
    default:
        break;
    }

    if (debug) {
        int start_index = static_cast<int>(row) * 4;
        int end_index = start_index + 4;
        if (target_frame == "map") {
            multi_tof_points_on_map_frame.erase(multi_tof_points_on_map_frame.begin(), multi_tof_points_on_map_frame.begin() + start_index);
            zero_dist_index.erase(zero_dist_index.begin(), zero_dist_index.begin() + start_index);
            if ((end_index - start_index) < static_cast<int>(multi_tof_points_on_map_frame.size())) {  
                multi_tof_points_on_map_frame.erase(multi_tof_points_on_map_frame.begin() + (end_index - start_index), 
                                                    multi_tof_points_on_map_frame.end());
                zero_dist_index.erase(zero_dist_index.begin() + (end_index - start_index), 
                                      zero_dist_index.end());
            }
        } else if (target_frame == "base_link") {
            multi_tof_points_on_robot_frame.erase(multi_tof_points_on_robot_frame.begin(), multi_tof_points_on_robot_frame.begin() + start_index);
            zero_dist_index.erase(zero_dist_index.begin(), zero_dist_index.begin() + start_index);
            if ((end_index - start_index) < static_cast<int>(multi_tof_points_on_robot_frame.size())) {  
                multi_tof_points_on_robot_frame.erase(multi_tof_points_on_robot_frame.begin() + (end_index - start_index), 
                                                      multi_tof_points_on_robot_frame.end());
                zero_dist_index.erase(zero_dist_index.begin() + (end_index - start_index), 
                                      zero_dist_index.end());
            }
        }
    }

    if (target_frame == "map") {
        std::vector<tPoint> filtered_tof_points = filterPoints(multi_tof_points_on_map_frame);
        return createTofBotPointCloud2Message(filtered_tof_points);
    } else if (target_frame == "base_link") {
        std::vector<tPoint> filtered_tof_points = filterPoints(multi_tof_points_on_robot_frame);
        return createTofBotPointCloud2Message(filtered_tof_points);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Select Wrong Target Frame: %s", target_frame.c_str());
        return sensor_msgs::msg::PointCloud2();
    }
}

sensor_msgs::msg::PointCloud2 PointCloud::getConvertedCameraToPointCloud(const robot_custom_msgs::msg::AIDataArray::SharedPtr msg, float pc_resolution, int number_of_object)
{
    vision_msgs::msg::BoundingBox2DArray bbox_array = createBoundingBoxMessage(msg);
    setCameraBoundingBoxMessage(bbox_array);
    return createCameraPointCloud2Message(bbox_array, pc_resolution);
}

// TODO
sensor_msgs::msg::PointCloud2 PointCloud::getConvertedLineLaserToPointCloud(const robot_custom_msgs::msg::LineLaserData::SharedPtr msg)
{
    return sensor_msgs::msg::PointCloud2();
}

vision_msgs::msg::BoundingBox2DArray PointCloud::getCameraBoundingBoxMessage()
{
    return camera_bbox_array;
}

std::vector<tPoint> PointCloud::transformTofMsg2PointsOnSensorFrame(std::vector<double> input_tof_dist, bool isBothSide)
{
    std::vector<tPoint> points;

    constexpr int ROWS = 4;
    constexpr int COLS = 4;

    const double xy_sine[COLS] = {tof_bot_col_4_xy_tan_, tof_bot_col_3_xy_tan_,
                                  tof_bot_col_2_xy_tan_, tof_bot_col_1_xy_tan_};
    const double z_sine[ROWS] = {tof_bot_row_1_z_tan_, tof_bot_row_2_z_tan_,
                                 tof_bot_row_3_z_tan_, tof_bot_row_4_z_tan_};
    
    for (int row = 0; row < ROWS; ++row) {
        for (int col = 0; col < COLS; ++col) {
            int index = row * COLS + col;
            
            if (!isBothSide) {
                if (input_tof_dist[index] > 0.001) { // 0값 필터링을 위한 인덱스 저장
                    zero_dist_index[index] = false;
                } else {
                    zero_dist_index[index] = true;
                }
            } else {
                if (input_tof_dist[index] > 0.001) { // 0값 필터링을 위한 인덱스 저장
                    zero_dist_index[index+16] = false;
                } else {
                    zero_dist_index[index+16] = true;
                }
            }

            tPoint p;
            p.x = input_tof_dist[index];
            p.y = input_tof_dist[index] * xy_sine[col];
            p.z = input_tof_dist[index] * z_sine[row];
            points.push_back(p);
        }
    }
    
    return points;
}

std::vector<tPoint> PointCloud::transformCameraMsg2PointsOnSensorFrame(std::vector<robot_custom_msgs::msg::AIData> input_camera_objs)
{
    std::vector<tPoint> points;
    tPoint point;
    for(auto obj: input_camera_objs) {
        point.x = obj.distance * std::cos(obj.theta);
        point.y = obj.distance * std::sin(obj.theta);
        point.z = -camera_sensor_frame_z_translate_;
    }
    
    return points;
}

/**
 * @ Param
 * ### left = true, right = false
 */
std::vector<tPoint> PointCloud::transformTofSensor2RobotFrame(const std::vector<tPoint>& input_points, bool left)
{
    std::vector<tPoint> points;
    tPoint p;

    for (const auto& point : input_points) {
        if (left) {
            p.x = point.x * tof_left_sensor_frame_yaw_cosine_ - point.y * tof_left_sensor_frame_yaw_sine_ + tof_bot_sensor_frame_x_translate_;
            p.y = point.x * tof_left_sensor_frame_yaw_sine_ + point.y * tof_left_sensor_frame_yaw_cosine_ + tof_bot_sensor_frame_y_translate_;
            p.z = point.z + tof_bot_sensor_frame_z_translate_;
        } else {
            p.x = point.x * tof_right_sensor_frame_yaw_cosine_ - point.y * tof_right_sensor_frame_yaw_sine_ + tof_bot_sensor_frame_x_translate_;
            p.y = point.x * tof_right_sensor_frame_yaw_sine_ + point.y * tof_right_sensor_frame_yaw_cosine_ - tof_bot_sensor_frame_y_translate_;
            p.z = point.z + tof_bot_sensor_frame_z_translate_;
        }
        points.push_back(p);
    }

    return points;
}

std::vector<tPoint> PointCloud::transformCameraSensor2RobotFrame(const std::vector<tPoint>& input_points)
{
    std::vector<tPoint> points;
    tPoint p;

    for (const auto& point : input_points) {
        p.x = point.x + camera_sensor_frame_x_translate_;
        p.y = point.y + camera_sensor_frame_y_translate_;
        p.z = point.z + camera_sensor_frame_z_translate_;

        points.push_back(p);
    }

    return points;
}

std::vector<tPoint> PointCloud::transformRobot2GlobalFrame(const std::vector<tPoint>& input_points)
{
    std::vector<tPoint> global_points;
    tPoint global_point;
    const float robotCosine = std::cos(robotPose.orientation.yaw);
    const float robotSine = std::sin(robotPose.orientation.yaw);
    for (const auto& local_point : input_points) {
        global_point.x = local_point.x*robotCosine - local_point.y*robotSine + robotPose.position.x;
        global_point.y = local_point.x*robotSine + local_point.y*robotCosine + robotPose.position.y;
        global_point.z = local_point.z + robotPose.position.z;
        global_points.push_back(global_point);
    }
    return global_points;
}

std::vector<tPoint> PointCloud::transformRobot2GlobalFrame(const tPoint& input_point)
{
    std::vector<tPoint> points = {input_point};
    return transformRobot2GlobalFrame(points);
}

std::vector<tPoint> PointCloud::filterPoints(const std::vector<tPoint> &input_points)
{
    std::vector<tPoint> filtered_points;

    for (int i=0; i<zero_dist_index.size(); ++i) {
        if (!zero_dist_index[i]) {
            filtered_points.push_back(input_points[i]);
        }
    }
    return filtered_points;
}

/**
 * ## Convert Multi ToF Data
 */
sensor_msgs::msg::PointCloud2 PointCloud::createTofBotPointCloud2Message(const std::vector<tPoint>& points)
{
    sensor_msgs::msg::PointCloud2 msg;

    if (points.empty()) {
        // RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Input data is empty!");
        return msg;
    }

    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = target_frame;

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
 * ## Convert 1D ToF Data
 */
sensor_msgs::msg::PointCloud2 PointCloud::createTofTopPointCloud2Message(const tPoint& intpu_point)
{
    std::vector<tPoint> points = {intpu_point};
    return createTofBotPointCloud2Message(points);
}

/**
 * ## Convert Camera Data
 */
sensor_msgs::msg::PointCloud2 PointCloud::createCameraPointCloud2Message(const vision_msgs::msg::BoundingBox2DArray input_bbox_array, float resolution) 
{
    sensor_msgs::msg::PointCloud2 cloud_msg;

    if (input_bbox_array.boxes.empty()) {
        // RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Input data is empty!");
        return cloud_msg;
    }
    if (resolution <= 0) {
        // RCLCPP_ERROR(rclcpp::get_logger("PointCloud"), "Invalid resolution: %f", resolution);
        return cloud_msg;
    }

    size_t total_points = 0;
    int point_size_x, point_size_y;
    for (const auto& box : input_bbox_array.boxes) {
        if (box.size_x <= 0 || box.size_y <= 0 ) { // width, height 음수인 경우는 예외처리 (계산 망가짐)
            return cloud_msg;
        }
        point_size_x = static_cast<int>(box.size_x/resolution) + 1;
        point_size_y = static_cast<int>(box.size_y/resolution) + 1;
        total_points += point_size_x * point_size_y;
    }
    cloud_msg.width = total_points;

    cloud_msg.header = input_bbox_array.header;
    cloud_msg.height = 1;
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;
    cloud_msg.point_step = 12;
    cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;

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

    cloud_msg.fields = {field_x, field_y, field_z};
    
    cloud_msg.data.resize(cloud_msg.row_step);
    size_t max_size = cloud_msg.data.size();
    uint8_t* ptr = cloud_msg.data.data();
    for (const auto& box : input_bbox_array.boxes) {
        const double center_x = box.center.position.x;
        const double center_y = box.center.position.y;
        const double size_x = box.size_x;
        const double size_y = box.size_y;

        for (int i = 0; i < point_size_x; ++i) {
            for (int j = 0; j < point_size_y; ++j) {

                if ((ptr - cloud_msg.data.data()) + cloud_msg.point_step > max_size) {
                    RCLCPP_ERROR(rclcpp::get_logger("PointCloud"), "############## IGNORE! #############");
                    RCLCPP_ERROR(rclcpp::get_logger("PointCloud"), "##### Memory overflow detected! ####");
                    return cloud_msg;
                }

                float x = (center_x - size_x/2) + i*resolution;
                float y = (center_y - size_y/2) + j*resolution;
                float z = 0.0f;
                memcpy(ptr, &x, sizeof(float));
                memcpy(ptr + 4, &y, sizeof(float));
                memcpy(ptr + 8, &z, sizeof(float));
                ptr += cloud_msg.point_step;
            }
        }
    }
#if USE_Camera_LOG
    RCLCPP_INFO(rclcpp::get_logger("PointCloud"), "Complete to make PointCloud2 | resolution: %f, total_points: %zu", resolution, total_points);
#endif
    return cloud_msg;
}

vision_msgs::msg::BoundingBox2DArray PointCloud::createBoundingBoxMessage(const robot_custom_msgs::msg::AIDataArray::SharedPtr msg)
{
    auto bbox_array = vision_msgs::msg::BoundingBox2DArray();

    if (msg->data_array.empty() || msg->num == 0) {
        // RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Input data is empty!");
        return bbox_array;
    }

#if USE_Camera_LOG
    RCLCPP_INFO(rclcpp::get_logger("PointCloud"),
        "[Camera]==============================================");
    RCLCPP_INFO(rclcpp::get_logger("PointCloud"),
        "[BBox] Number of Objects: %d", msg->num);
#endif

    bbox_array.header.stamp = rclcpp::Clock().now();
    bbox_array.header.frame_id = target_frame;

    std::vector<robot_custom_msgs::msg::AIData> objects(msg->data_array.begin(), msg->data_array.end());
    for (const auto &obj : objects)
    {
        if (obj.height >= 0.0 && obj.width >= 0.0) {
            auto bbox = vision_msgs::msg::BoundingBox2D();

            tPoint point_on_sensor_frame, point_on_robot_frame;

            point_on_sensor_frame.x = obj.distance * std::cos(obj.theta) + obj.height/2;
            point_on_sensor_frame.y = obj.distance * std::sin(obj.theta);
            point_on_sensor_frame.z = -camera_sensor_frame_z_translate_;

            point_on_robot_frame.x = point_on_sensor_frame.x + camera_sensor_frame_x_translate_;
            point_on_robot_frame.y = point_on_sensor_frame.y + camera_sensor_frame_y_translate_;
            point_on_robot_frame.z = point_on_sensor_frame.z + camera_sensor_frame_z_translate_;
            if (target_frame == "map") {    
                tPoint point_on_global_frame;
                const float robotCosine = std::cos(robotPose.orientation.yaw);
                const float robotSine = std::sin(robotPose.orientation.yaw);
                point_on_global_frame.x = point_on_robot_frame.x*robotCosine - point_on_robot_frame.y*robotSine + robotPose.position.x;
                point_on_global_frame.y = point_on_robot_frame.x*robotSine + point_on_robot_frame.y*robotCosine + robotPose.position.y;
                point_on_global_frame.z = point_on_robot_frame.z + robotPose.position.z;
                bbox.center.position.x = point_on_global_frame.x;
                bbox.center.position.y = point_on_global_frame.y;
            } else if (target_frame == "base_link") {
                bbox.center.position.x = point_on_robot_frame.x;
                bbox.center.position.y = point_on_robot_frame.y;
            } else {
                RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Select Wrong Target Frame: %s", target_frame.c_str());
            }
            bbox.center.theta = 0.0;
            bbox.size_x = obj.height;
            bbox.size_y = obj.width;
#if USE_Camera_LOG
        RCLCPP_INFO(rclcpp::get_logger("PointCloud"),
            "[Sensor Frame] Object ID: %d",obj.id);
        RCLCPP_INFO(rclcpp::get_logger("PointCloud"),
            "X, Y, Z: %f, %f, %f",
            point_on_sensor_frame.x, point_on_sensor_frame.y, point_on_sensor_frame.z);

        RCLCPP_INFO(rclcpp::get_logger("PointCloud"),
            "[Robot Frame] Object ID: %d",obj.id);
        RCLCPP_INFO(rclcpp::get_logger("PointCloud"),
            "X, Y, Z: %f, %f, %f",
            point_on_robot_frame.x, point_on_robot_frame.y, point_on_robot_frame.z);
#endif
            bbox_array.boxes.push_back(bbox);
        }
    }

    return bbox_array;
}

void PointCloud::setCameraBoundingBoxMessage(vision_msgs::msg::BoundingBox2DArray bbox_array)
{
    camera_bbox_array = bbox_array;
}
