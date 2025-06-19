#include "lidar/node_lidar_ros.h"
#include "node_lidar.h"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include "std_msgs/msg/bool.hpp"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>

#include <iostream>
#include <chrono>
#include <thread>

using namespace std::chrono;
static bool bLidarCmd = true;
static bool bLidarRun = false;
static bool bErrorState = false;

class MinimalSubscriber
{
  public:
	MinimalSubscriber(const rclcpp::Node::SharedPtr& node) : node_(node)
	{
		cmd_lidar_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
		"cmd_lidar", 10, std::bind(&MinimalSubscriber::lidar_cmd_callback, this, std::placeholders::_1));
	}


  private:
	void lidar_cmd_callback(const std_msgs::msg::Bool::SharedPtr msg) const
	{
		if (msg->data) // turn on lidar
		{
			bLidarCmd = true;
			RCLCPP_INFO(node_->get_logger(), "[Subscribe] Lidar ON");
		}
		else // turn off lidar
		{
			bLidarCmd = false;
			RCLCPP_INFO(node_->get_logger(), "[Subscribe] Lidar OFF");
		}
	}
	rclcpp::Node::SharedPtr node_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_lidar_sub_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("airbot_lidar");
    MinimalSubscriber minimal_subscriber(node); // MinimalSubscriber 객체 생성 (subscriber 부분)

	node->declare_parameter("port","/dev/sc_mini");
	node->get_parameter("port", node_lidar.lidar_general_info.port);
	RCLCPP_INFO(node->get_logger(), "Port: %s", node_lidar.lidar_general_info.port.c_str());

	std::string frame_id;
	node->declare_parameter("frame_id", "laser_link");
	node->get_parameter("frame_id", frame_id);
	RCLCPP_INFO(node->get_logger(), "framd_id: %s", frame_id.c_str());

	auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
	auto scan_error_pub = node->create_publisher<std_msgs::msg::Bool>("scan_error", 10);
	auto scan_dirty_error_pub = node->create_publisher<std_msgs::msg::Bool>("scan_dirty", 10);

	auto error_msg = std::make_shared<std_msgs::msg::Bool>();
	error_msg->data = false;
	std_msgs::msg::Bool dirty_msg;
	dirty_msg.data = false;
	static const unsigned int total_points_num = 400; 		// number of lidar ranges
	static const float min_range_th = 0.03;					// 3cm
	static const unsigned int dirty_percentage_th = 10; 	// 0~100 %
	static const unsigned int dirty_points_th = static_cast<int>(total_points_num/100 * dirty_percentage_th);
	static const unsigned int dirty_cnt_th = 600;
	static unsigned int dirty_points = 0;
	static unsigned int dirty_cnt = 0;
	static unsigned int dirty_reset_cnt = 0;


	static int start_cnt = 0;
    // 별도 스레드에서 데이터 처리(publish 부분)
	std::thread lidar_thread([&]() {
		node_start();

		while(rclcpp::ok())
		{
			LaserScan scan;

			if (bLidarCmd && !bLidarRun) // 라이다 On 명령이지만 동작하지 않을 때 (최초 시작 or 에러 상태)
			{
				if (bErrorState)
				{
					// 라이다 재시작 시도
					RCLCPP_INFO(node->get_logger(), "Retry to Restart LiDAR");
					node_lidar.lidar_status.lidar_ready = true;
					node_lidar.serial_port->write_data(start_lidar,4);

					bLidarRun = data_handling(scan); // timeout 3sec

					if (bLidarRun)
					{
						RCLCPP_INFO(node->get_logger(), "[ERROR/Released] Publish LiDAR Communication Error Released");
						bErrorState = false;
						error_msg->data = false;
						scan_error_pub->publish(*error_msg);
					}
				}
				else
				{
					// 라이다  START
					RCLCPP_INFO(node->get_logger(), "LiDAR START");
					node_lidar.lidar_status.lidar_ready = true;
					node_lidar.serial_port->write_data(start_lidar,4);

					bLidarRun = data_handling(scan); // timeout 3sec

					if (!bLidarRun)
					{
						if (start_cnt > 2) { // 최대 2번까지 시도 (한번엔 안켜지는 경우가 있음)
							RCLCPP_INFO(node->get_logger(), "[ERROR/Occured] LiDAR START FAIL. Publishing LiDAR Communication Error.");
							bErrorState = true;
							error_msg->data = true;
							scan_error_pub->publish(*error_msg);
						}
						start_cnt++;
						RCLCPP_INFO(node->get_logger(),
							"Waiting to start LiDAR, Lidar Running: %s, retry cnt: %d",
							bLidarRun ? "true" : "false", start_cnt
						);
					} else {
						RCLCPP_INFO(node->get_logger(),
							"Success to start LiDAR, Lidar Running: %s, retry cnt: %d",
							bLidarRun ? "true" : "false", start_cnt
						);
						start_cnt = 0;
					}
				}
			}
			else if (bLidarCmd && bLidarRun)
			{
				bLidarRun = data_handling(scan);

				if (!bLidarRun)
				{
					// 에러 퍼블리싱
					RCLCPP_INFO(node->get_logger(), "[ERROR/Occured] LiDAR communication lost during operation. Publishing LiDAR Communication Error.");
					bErrorState = true;
					error_msg->data = true;
					scan_error_pub->publish(*error_msg);
				}
				else
				{
					// 기존 scan 퍼블리싱 동작
					auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

					float min_angle = 70 * M_PI/180;
					float max_angle = 290 * M_PI/180;

					scan_msg->angle_min = min_angle;
					scan_msg->angle_max = max_angle;
					scan_msg->angle_increment = scan.config.angle_increment;
					scan_msg->scan_time = scan.config.scan_time;
					scan_msg->time_increment = scan.config.time_increment;
					scan_msg->range_min = 0.10;
					scan_msg->range_max = scan.config.max_range;

					std::vector<float> filtered_ranges;
					std::vector<float> filtered_intensities;

					for (size_t i = 0; i < scan.points.size(); i++) {
						float angle = scan.config.min_angle + i * scan.config.angle_increment;

						if (angle >= min_angle && angle <= max_angle) {
							filtered_ranges.push_back(scan.points[i].range);
							if (scan.points[i].range > 1e-6 && scan.points[i].range <= min_range_th) {
								dirty_points += 1;
							}
						}
					}

					if (dirty_points > dirty_points_th) {
						dirty_cnt += 1;
						if (dirty_cnt % 20 == 0) {
							RCLCPP_INFO(node->get_logger(), "[ERROR/Monitor] dirty_cnt: %d", dirty_cnt);
						}
					} else {
						dirty_cnt = 0;
						if (dirty_msg.data) {
							if (dirty_reset_cnt > 0) {
								dirty_reset_cnt--;
								if(dirty_reset_cnt == 0){
									dirty_msg.data = false;
									scan_dirty_error_pub->publish(dirty_msg);
									RCLCPP_INFO(node->get_logger(), "[ERROR/Released] scan_dirty: dirty error cleared after 10 time checked");
								}
							}
						}
					}

					if (dirty_cnt > dirty_cnt_th) {
						dirty_msg.data = true;
						scan_dirty_error_pub->publish(dirty_msg);
						RCLCPP_INFO(node->get_logger(), "[ERROR/Occured] scan_dirty: lidar pollution error");
						dirty_cnt = 0;
						dirty_reset_cnt = 10;
					}

					dirty_points = 0;

					scan_msg->ranges = filtered_ranges;
					scan_msg->intensities = filtered_intensities;

					rclcpp::Time now = node->now();
					scan_msg->header.stamp = now;
					scan_msg->header.frame_id = frame_id;
					scan_pub->publish(*scan_msg);
				}
			}
			else if (!bLidarCmd && bLidarRun)
			{
				// 라이다 STOP
				RCLCPP_INFO(node->get_logger(), "LiDAR STOP");
				node_lidar.lidar_status.lidar_ready = false;
				node_lidar.lidar_status.close_lidar = true;
				node_lidar.serial_port->write_data(end_lidar,4);

				bLidarRun = data_handling(scan); // timeout 3sec

				if (bLidarRun)
				{
					RCLCPP_INFO(node->get_logger(), "Wait to LIDAR STOP");
				}
			}
			else
			{
				// 라이다 재시작 시도 (에러 해제)
				if (bErrorState) {
					RCLCPP_INFO(node->get_logger(), "Retry to Restart LiDAR");
					node_lidar.lidar_status.lidar_ready = true;
					node_lidar.serial_port->write_data(start_lidar,4);

					bLidarRun = data_handling(scan); // timeout 3sec

					if (bLidarRun)
					{
						RCLCPP_INFO(node->get_logger(), "[ERROR/Released] Publish LiDAR Communication Error Released");
						bErrorState = false;
						error_msg->data = false;
						scan_error_pub->publish(*error_msg);
					}
				}
			}
		}

		node_lidar.serial_port->write_data(end_lidar,4);
	});

	// rclcpp::spin()을 호출하여 콜백 처리 ( subscriber callback을 위해서)
	rclcpp::spin(node);

	if (lidar_thread.joinable())
	{
		lidar_thread.join();
	}

	rclcpp::shutdown();	
	return 0;
}