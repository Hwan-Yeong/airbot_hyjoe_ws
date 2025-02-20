// 2024-12-26 clabil v1.1
#ifndef __A1_KEEPOUT_H__
#define __A1_KEEPOUT_H__
#include "rclcpp/rclcpp.hpp"
#include "robot_custom_msgs/msg/block_area_list.hpp" 
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nlohmann/json.hpp"
#include "std_msgs/msg/header.hpp"
#include "fstream"
#include "iostream"


using json = nlohmann::json;

struct point
{
    double x;
    double y;
    double z;

    point() : x(0.0), y(0.0), z(1.0){} //생성자
};

class NavKeepoutService : public rclcpp::Node
{
public:
    NavKeepoutService();
    ~NavKeepoutService() = default;
    void run();

private:
    unsigned int  reqeust_time_ms_;
    bool          send_keepout_;
    float         obs_point_resolution_;
    std::string   json_file_path_;
    std::string   publish_topic_name_;

    json          json_area_;
    json          json_wall_;
    
    rclcpp::Subscription<robot_custom_msgs::msg::BlockAreaList>::SharedPtr  block_line_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::BlockAreaList>::SharedPtr  block_area_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr             keepout_zone_pub_;
    
    // callback functions
    void block_wall_callback(const robot_custom_msgs::msg::BlockAreaList::SharedPtr msg);
    void block_area_callback(const robot_custom_msgs::msg::BlockAreaList::SharedPtr msg);
    void timer_callback(void);

    // json functions
    void load_keepout_zones_from_json(const std::string &file_path, json &json_data) ;
    void save_json_to_file(const json& json_data, const std::string& file_name);

    // publish
    std::vector<point> generate_pointcloud_by_json(void);
    std::vector<point> generate_pointcloud_by_coordinates(const std::vector<point> &origin);
    void publish_point_cloud(const std::vector<point>& points);

    void create_directory_if_not_exists(const std::string &path);
    double get_distance_two_points(point robot, point target);
};
#endif