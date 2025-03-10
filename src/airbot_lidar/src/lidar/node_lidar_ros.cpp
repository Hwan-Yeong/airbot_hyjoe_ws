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
std::chrono::time_point<std::chrono::steady_clock> last_data_time;
bool isLidarCmd = false;

class MinimalSubscriber
{
  public:
	MinimalSubscriber(const rclcpp::Node::SharedPtr& node)
	{
		cmd_lidar_sub_ = node->create_subscription<std_msgs::msg::Bool>(
		"cmd_lidar", 10, std::bind(&MinimalSubscriber::lidar_cmd_callback, this, std::placeholders::_1));
	}

  private:
	void lidar_cmd_callback(const std_msgs::msg::Bool::SharedPtr msg) const
	{
		bool lidarCmd = msg->data;
		size_t r;

		// RCLCPP_INFO(rclcpp::get_logger("cmd_lidar"), "lidar is %s ", lidarCmd ? "true" : "false");

		if (lidarCmd)
		{
			//turn on lidar
			last_data_time = std::chrono::steady_clock::now();
			node_lidar.lidar_status.lidar_ready = true;
			isLidarCmd = true;
			node_lidar.lidar_status.lidar_abnormal_state = 0;
			RCLCPP_INFO(rclcpp::get_logger("cmd_lidar"), "lidar is start ");	
		}
		else
		{
			//turn off lidar
			node_lidar.lidar_status.lidar_ready = false;
			node_lidar.lidar_status.close_lidar = true;
			isLidarCmd = false;

			r = node_lidar.serial_port->write_data(end_lidar,4);
			if (r < 1)
			{
				RCLCPP_INFO(rclcpp::get_logger("cmd_lidar"), "stop lidar is failure ");
			}
			else
			{
				RCLCPP_INFO(rclcpp::get_logger("cmd_lidar"), "lidar is stop ");
			}
		}
	}
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_lidar_sub_;
};

//int topic_thread()
//{
//	rclcpp::spin(std::make_shared<MinimalSubscriber>());
//}

int main(int argc, char **argv)
{ 
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("airbot_lidar");	
	// MinimalSubscriber 객체 생성 (subscriber 부분)
    MinimalSubscriber minimal_subscriber(node);

	node->declare_parameter("port","/dev/sc_mini");
  	node->get_parameter("port", node_lidar.lidar_general_info.port);
	RCLCPP_INFO(node->get_logger(), "Port: %s", node_lidar.lidar_general_info.port.c_str());

	node->declare_parameter("frame_id", "laser_link");
        std::string frame_id;
        node->get_parameter("frame_id", frame_id);

	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub;
	error_pub = node->create_publisher<std_msgs::msg::String>("lsd_error", 10);
	std_msgs::msg::String pubdata;
    //error_check 마지막 데이터 수신 시간
	last_data_time = std::chrono::steady_clock::now();
	auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    //lidar of the error of the remap from lidar_dual_launch.py
	auto laser_error_pub = node->create_publisher<std_msgs::msg::Bool>("scan_error", 10);
	
	///// lidar pollution (dirty) /////
	auto laser_dirty_pub = node->create_publisher<std_msgs::msg::Bool>("scan_dirty", 10);
	std_msgs::msg::Bool dirty_msg;
	static const unsigned int total_points_num = 400; 		// number of lidar ranges
	static const float min_range_th = 0.03;					// 3cm
	static const unsigned int dirty_percentage_th = 10; 	// 0~100 %
	static const unsigned int dirty_points_th = static_cast<int>(total_points_num/100 * dirty_percentage_th);
	static const unsigned int dirty_cnt_th = 100;
	static unsigned int dirty_points = 0;
	static unsigned int dirty_cnt = 0;
	///////////////////////////////////

    // 별도 스레드에서 데이터 처리(publish 부분)
	std::thread lidar_thread([&]() {
		node_start();
		static bool initial_error_handled = false;
		while(rclcpp::ok())
		{
			if(node_lidar.lidar_status.lidar_abnormal_state != 0)
			{
				if(node_lidar.lidar_status.lidar_abnormal_state & 0x01)
				{
					pubdata.data="node_lidar is trapped\n";
					error_pub->publish(pubdata);
					RCLCPP_INFO(node->get_logger(), "[error/lidar] lsd_error: trapped");
				}
				if(node_lidar.lidar_status.lidar_abnormal_state & 0x02)
				{
					pubdata.data="node_lidar frequence abnormal\n";
					error_pub->publish(pubdata);
					RCLCPP_INFO(node->get_logger(), "[error/lidar] lsd_error: frequence abnormal");
				}
				if(node_lidar.lidar_status.lidar_abnormal_state & 0x04)
				{
					pubdata.data="node_lidar is blocked\n";
					error_pub->publish(pubdata);
					RCLCPP_INFO(node->get_logger(), "[error/lidar] lsd_error: blocked");
				}
				// node_lidar.serial_port->write_data(end_lidar,4);
				// node_lidar.lidar_status.lidar_ready = false;

				// std::this_thread::sleep_for(std::chrono::seconds(1));
			}
			LaserScan scan;
			
			if(data_handling(scan))
			{
				auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

				float min_angle = 60 * M_PI/180;
				float max_angle = 300 * M_PI/180;

				// scan_msg->ranges.resize(scan.points.size());
				// scan_msg->intensities.resize(scan.points.size());

				scan_msg->angle_min = min_angle; //scan.config.min_angle;
				scan_msg->angle_max = max_angle; //scan.config.max_angle;
				scan_msg->angle_increment = scan.config.angle_increment;
				scan_msg->scan_time = scan.config.scan_time;
				scan_msg->time_increment = scan.config.time_increment;
				scan_msg->range_min = 0.10;
				scan_msg->range_max = scan.config.max_range;

				// for(int i=0; i < scan.points.size(); i++) {
				// 	scan_msg->ranges[i] = scan.points[i].range;
				// 	scan_msg->intensities[i] = scan.points[i].intensity;
				// }

				std::vector<float> filtered_ranges;
				std::vector<float> filtered_intensities;

				for (size_t i = 0; i < scan.points.size(); i++) {
					float angle = scan.config.min_angle + i * scan.config.angle_increment;

					if (angle >= min_angle && angle <= max_angle) {
						filtered_ranges.push_back(scan.points[i].range);
						// filtered_intensities.push_back(scan.points[i].intensity);
						if (scan.points[i].range > 1e-6 && scan.points[i].range <= min_range_th) {
							dirty_points += 1;
						}
					}
				}

				if (dirty_points > dirty_points_th) {
					dirty_cnt += 1;
					if (dirty_cnt % 20 == 0) {
						RCLCPP_INFO(node->get_logger(), "[error/lidar] dirty_cnt: %d", dirty_cnt);
					}
				} else {
					dirty_cnt = 0;
				}

				if (dirty_cnt > dirty_cnt_th) {
					dirty_msg.data = true;
					laser_dirty_pub->publish(dirty_msg);
					RCLCPP_INFO(node->get_logger(), "[error/lidar] scan_dirty: lidar pollution error");
				}
				
				dirty_points = 0;

				scan_msg->ranges = filtered_ranges;
				scan_msg->intensities = filtered_intensities;

				rclcpp::Time now = node->now();
				scan_msg->header.stamp = now;
				scan_msg->header.frame_id = frame_id; // Get the current ROS 2 time
				laser_pub->publish(*scan_msg);

				//error_스캔 데이터가 정상적으로 수신되었으므로 타이머 갱신
				last_data_time = std::chrono::steady_clock::now();
				auto error_msg = std::make_shared<std_msgs::msg::Bool>();
				error_msg->data = false;
				laser_error_pub->publish(*error_msg);
			}
			else
			{
				if (!initial_error_handled)
				{
					RCLCPP_INFO(node->get_logger(), "except!");
					initial_error_handled = true;
				}
				else
				{
					if (node_lidar.lidar_status.lidar_ready) {
						//error_마지막 데이터 수신 후 경과 시간 계산
						//lidar time out = 2sec
						auto current_time = std::chrono::steady_clock::now();
						std::chrono::duration<double> elapsed_time = current_time - last_data_time;
						if (isLidarCmd) {
							if (elapsed_time.count() >= 10.0) {
								// 라이다 On 명령이 들어오고 나서, 10초 이상 데이터가 없는 경우 에러 메시지 퍼블리시
								auto error_msg = std::make_shared<std_msgs::msg::Bool>();
								error_msg->data = true;
								laser_error_pub->publish(*error_msg);
								RCLCPP_INFO(node->get_logger(), "[error/lidar] scan_error: lidar communication error - No scan data for 10 seconds [lidar: ON]");
							} else {
								// RCLCPP_INFO(node->get_logger(), "[Lidar ON] elapsed_time: %.2f seconds",elapsed_time.count());
							}
						} else {
							if (elapsed_time.count() >= 3.0) {
								// normal 상태에서 3초 이상 데이터가 없는 경우 에러 메시지 퍼블리시
								auto error_msg = std::make_shared<std_msgs::msg::Bool>();
								error_msg->data = true;
								laser_error_pub->publish(*error_msg);
								RCLCPP_INFO(node->get_logger(), "[error/lidar] scan_error: lidar communication error - No scan data for 3 seconds");
							} else {
								// RCLCPP_INFO(node->get_logger(), "[Lidar Normal] elapsed_time: %.2f seconds",elapsed_time.count());
							}
						}
					}
				}
			}
		}
		
		node_lidar.serial_port->write_data(end_lidar,4);
	});

	// rclcpp::spin()을 호출하여 콜백 처리 ( subscriber callback을 위해서)
	rclcpp::spin(node);

	// 스레드 종료
	if (lidar_thread.joinable())
	{
		lidar_thread.join();
	}

	rclcpp::shutdown();	
	return 0;
}