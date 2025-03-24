// 2025-02-26 clabil v1.3
#include "A1_keepout/A1_keepout.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#define PRINT_INFO false

NavKeepoutService::NavKeepoutService() : Node("A1_keepout")
{
    std::string package_name = "A1_keepout";
    std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
    this->declare_parameter("reqeust_time_ms", 1000);
    this->declare_parameter("json_file_path", package_share_directory + "/json/");
    this->declare_parameter("publish_topic_name", "/A1_keepout/point_cloud");
    this->declare_parameter("obstacle_point_resolution", 0.05);

    send_keepout_ = false;
    reqeust_time_ms_ = this->get_parameter("reqeust_time_ms").as_int();
    json_file_path_ = this->get_parameter("json_file_path").get_value<std::string>();
    publish_topic_name_ = this->get_parameter("publish_topic_name").get_value<std::string>();
    this->get_parameter("obstacle_point_resolution", obs_point_resolution_);

    RCLCPP_INFO(this->get_logger(), "==============");
    RCLCPP_INFO(this->get_logger(), "Keepout json file : %s", json_file_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Reqeust time(ms) : %d", reqeust_time_ms_);
    RCLCPP_INFO(this->get_logger(), "Publish topic name : %s", publish_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Obstacle point resolution : %f", obs_point_resolution_);
    RCLCPP_INFO(this->get_logger(), "==============");

    // Ros2 에서 QoS 설정을 하지 않으면 데이터가 안넘어오는 경우가 생김
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.best_effort();

    block_line_sub_ = this->create_subscription<robot_custom_msgs::msg::BlockAreaList>(
        "/block_walls", qos_profile, std::bind(&NavKeepoutService::block_wall_callback, this, std::placeholders::_1));

    block_area_sub_ = this->create_subscription<robot_custom_msgs::msg::BlockAreaList>(
        "/block_areas", qos_profile, std::bind(&NavKeepoutService::block_area_callback, this, std::placeholders::_1));

    // keepout_zone_pub_ =
    // this->create_publisher<sensor_msgs::msg::PointCloud2>("/A1_keepout/point_cloud",
    // 10);
    keepout_zone_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(publish_topic_name_, 10);
    create_directory_if_not_exists(json_file_path_);
    load_keepout_zones_from_json(json_file_path_ + "area.json", json_area_);
    load_keepout_zones_from_json(json_file_path_ + "wall.json", json_wall_);

    RCLCPP_INFO(this->get_logger(), "A1 keepout service node start!");
}

// 경로가 존재하지 않으면 디렉토리 생성
void NavKeepoutService::create_directory_if_not_exists(const std::string& path)
{
    if (!std::filesystem::exists(path))
    {
        try
        {
            std::filesystem::create_directory(path);
        }
        catch (const std::exception& e)
        {
            RCLCPP_INFO(this->get_logger(), "Cannot create directory! Error: %s", e.what());
        }
    }
}

// JSON 파일 열기, 없으면 생성, 저장
void NavKeepoutService::load_keepout_zones_from_json(const std::string& file_path, json& json_data)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        // 파일을 생성하고 내용을 작성
        std::ofstream outputFile(file_path);
        if (outputFile.is_open())
        {
            outputFile << "{\"keepout_zones\":[]}";
            outputFile.close();  // 파일 닫기
            load_keepout_zones_from_json(file_path, json_data);
        }
    }
    else
    {
        file >> json_data;
    }
}

void NavKeepoutService::save_json_to_file(const json& json_data, const std::string& file_name)
{
    std::ofstream file(file_name);
    if (file.is_open())
    {
        file << json_data.dump(4);
        file.close();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        send_keepout_ = false;
        RCLCPP_INFO(this->get_logger(), "JSON data saved to %s", file_name.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file %s for writing", file_name.c_str());
    }
}

void NavKeepoutService::block_area_callback(const robot_custom_msgs::msg::BlockAreaList::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "block_area_callback callback");
    json keepout_zones;
    keepout_zones["keepout_zones"] = json::array();
    int zone_index = 0;
    for (const auto& vwall : msg->block_area_list)
    {
        std::vector<point> origin;
        std::string id = vwall.id.c_str();
        int index = 0;
        nlohmann::json data;
        keepout_zones["keepout_zones"][zone_index]["id"] = id;
        for (const auto& position : vwall.robot_path)
        {
            keepout_zones["keepout_zones"][zone_index]["coordinates"][index]["x"] = (double)position.x;
            keepout_zones["keepout_zones"][zone_index]["coordinates"][index]["y"] = (double)position.y;
            index++;
        }
        zone_index++;
    }
    std::string filename = "area.json";
    json_area_ = keepout_zones;
    save_json_to_file(keepout_zones, json_file_path_ + filename);
}

// Callback to handle incoming BlockWall messages
void NavKeepoutService::block_wall_callback(const robot_custom_msgs::msg::BlockAreaList::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "block_wall_callback callback");
    json keepout_zones;
    keepout_zones["keepout_zones"] = json::array();
    int zone_index = 0;
    for (const auto& vwall : msg->block_area_list)
    {
        std::vector<point> origin;
        std::string id = vwall.id.c_str();
        int index = 0;
        nlohmann::json data;
        keepout_zones["keepout_zones"][zone_index]["id"] = id;
        for (const auto& position : vwall.robot_path)
        {
            keepout_zones["keepout_zones"][zone_index]["coordinates"][index]["x"] = (double)position.x;
            keepout_zones["keepout_zones"][zone_index]["coordinates"][index]["y"] = (double)position.y;
            index++;
        }
        zone_index++;
    }
    std::string filename = "wall.json";
    json_wall_ = keepout_zones;
    save_json_to_file(keepout_zones, json_file_path_ + filename);
}

void NavKeepoutService::run()
{
    auto timer_ = this->create_wall_timer(
        std::chrono::milliseconds(reqeust_time_ms_), std::bind(&NavKeepoutService::timer_callback, this));
    rclcpp::spin(shared_from_this());
}

void NavKeepoutService::timer_callback()
{
    // if (send_keepout_ == false) {
#if PRINT_INFO
    RCLCPP_INFO(this->get_logger(), "Keepout zone timer callback");
#endif
    std::vector<point> result = generate_pointcloud_by_json();
    publish_point_cloud(result);
    // }
}

std::vector<point> NavKeepoutService::generate_pointcloud_by_json(void)
{
    std::vector<point> result;
    if (json_area_["keepout_zones"].size() > 0)
    {
        auto keepout_zones = json_area_["keepout_zones"];
        for (const auto& zone : keepout_zones)
        {
            std::vector<point> origin;
            for (const auto& coord : zone["coordinates"])
            {
                point p;
                p.x = (double)coord["x"];
                p.y = (double)coord["y"];
                origin.push_back(p);
                std::cout << "(" << coord["x"] << ", " << coord["y"] << ") ";
            }
            std::vector<point> cloud = generate_pointcloud_by_coordinates(origin);
            result.insert(result.end(), cloud.begin(), cloud.end());
        }
    }
    if (json_wall_["keepout_zones"].size() > 0)
    {
        auto keepout_zones = json_wall_["keepout_zones"];
        for (const auto& zone : keepout_zones)
        {
            std::vector<point> origin;
            for (const auto& coord : zone["coordinates"])
            {
                point p;
                p.x = (double)coord["x"];
                p.y = (double)coord["y"];
                origin.push_back(p);
                std::cout << "(" << coord["x"] << ", " << coord["y"] << ") ";
            }
            std::vector<point> cloud = generate_pointcloud_by_coordinates(origin);
            result.insert(result.end(), cloud.begin(), cloud.end());
        }
    }
    return result;
}

std::vector<point> NavKeepoutService::generate_pointcloud_by_coordinates(const std::vector<point>& origin)
{
    std::vector<point> result;
    point start;
    point end;
    point generated;

    if (origin.size() == 4)
    {  // virtual-area
        for (size_t i = 0; i < origin.size(); ++i)
        {
            start = origin[i];
            end = origin[(i + 1) % origin.size()];
            // 거리 계산
            double distance = get_distance_two_points(start, end);

            // 점 개수 계산
            int numPoints = std::ceil(distance / obs_point_resolution_);

            // 증분값 계산
            double dx = (end.x - start.x) / numPoints;
            double dy = (end.y - start.y) / numPoints;

            // 점 생성
            for (int j = 0; j <= numPoints; ++j)
            {
                generated.x = start.x + j * dx;
                generated.y = start.y + j * dy;
                result.emplace_back(generated);
            }
        }
    }
    else if (origin.size() == 2)
    {  // virtual-line
        for (size_t i = 0; i < origin.size() - 1; ++i)
        {
            start = origin[i];
            end = origin[(i + 1)];
            // 거리 계산
            double distance = get_distance_two_points(start, end);

            // 점 개수 계산
            int numPoints = std::ceil(distance / obs_point_resolution_);

            // 증분값 계산
            double dx = (end.x - start.x) / numPoints;
            double dy = (end.y - start.y) / numPoints;

            // 점 생성
            for (int j = 0; j <= numPoints; ++j)
            {
                generated.x = start.x + j * dx;
                generated.y = start.y + j * dy;
                result.emplace_back(generated);
            }
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Input is not valid. 2,4 point allowed but %ld", origin.size());
    }

    return result;
}

double NavKeepoutService::get_distance_two_points(point robot, point target)
{
    return std::sqrt(std::pow(target.x - robot.x, 2) + std::pow(target.y - robot.y, 2));
}

void NavKeepoutService::publish_point_cloud(const std::vector<point>& points)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;

    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = rclcpp::Clock().now();
    cloud_msg.height = 1;
    cloud_msg.width = points.size();
    cloud_msg.is_dense = true;
    cloud_msg.is_bigendian = false;

    cloud_msg.fields.resize(3);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;

    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;

    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;

    cloud_msg.data.resize(points.size() * 3 * sizeof(float));
    cloud_msg.point_step = 3 * sizeof(float);
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const auto& p : points)
    {
        *iter_x = static_cast<float>(p.x);
        *iter_y = static_cast<float>(p.y);
        *iter_z = static_cast<float>(p.z);
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    keepout_zone_pub_->publish(cloud_msg);
#if PRINT_INFO
    std::cout << "Published " << points.size() << " points to the /point_cloud topic." << std::endl;
#endif
}
