#include "lidar/node_lidar_ros.h"

LidarManager::LidarManager(const rclcpp::Node::SharedPtr& node)
: node_(node),
  bLidarCmd(true),
  bLidarRun(false),
  bErrorState(false),
  state_(LidarState::WAITING_TO_START),
  prev_state_(LidarState::WAITING_TO_START),
  start_cnt_(0),
  lidar_run_cnt_(0),
  dirty_points_(0),
  dirty_cnt_(0),
  dirty_reset_cnt_(0)
{
    initParams();

    scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    scan_error_pub_ = node_->create_publisher<std_msgs::msg::Bool>("scan_error", 10);
    scan_dirty_error_pub_ = node_->create_publisher<std_msgs::msg::Bool>("scan_dirty", 10);

	cmd_lidar_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
		"cmd_lidar", 10, std::bind(&LidarManager::cmdLidarCallback, this, std::placeholders::_1));

    error_msg_.data = false;
    dirty_msg_.data = false;
}

void LidarManager::initParams()
{
    node_->declare_parameter("port", "/dev/sc_mini");
    node_->get_parameter("port", node_lidar.lidar_general_info.port);
    RCLCPP_INFO(node_->get_logger(), "Port: %s", node_lidar.lidar_general_info.port.c_str());

    node_->declare_parameter("frame_id", "laser_link");
    node_->get_parameter("frame_id", frame_id_);
    RCLCPP_INFO(node_->get_logger(), "frame_id: %s", frame_id_.c_str());
}

void LidarManager::cmdLidarCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
	{
		bLidarCmd = true;
		RCLCPP_INFO(node_->get_logger(), "[Subscribe] Lidar ON");
	}
	else
	{
		bLidarCmd = false;
		RCLCPP_INFO(node_->get_logger(), "[Subscribe] Lidar OFF");
	}
}

void LidarManager::runLoop()
{
	node_start();

	while (rclcpp::ok()) {
		prev_state_ = state_;
		switch (state_)
		{
			case LidarState::WAITING_TO_START:
				if (bLidarCmd && !bLidarRun && !bErrorState) state_ = procWaitingToStart();
				else RCLCPP_INFO(node_->get_logger(), "Enter Wrong State: %s", enumToString(state_).c_str());
				break;

			case LidarState::RUNNING:
				if (bLidarCmd && bLidarRun && !bErrorState) state_ = procRunning();
				else RCLCPP_INFO(node_->get_logger(), "Enter Wrong State: %s", enumToString(state_).c_str());
				break;

			case LidarState::WAITING_TO_STOP:
				if (!bLidarCmd && bLidarRun && !bErrorState) state_ = procWaitingToStop();
				else RCLCPP_INFO(node_->get_logger(), "Enter Wrong State: %s", enumToString(state_).c_str());
				break;

			case LidarState::IDLE:
				if (!bLidarCmd && !bLidarRun && !bErrorState) state_ = procIdle();
				else RCLCPP_INFO(node_->get_logger(), "Enter Wrong State: %s", enumToString(state_).c_str());
				break;

			case LidarState::ERROR_WHILE_RUNNING:
				if (bLidarCmd && !bLidarRun && bErrorState) state_ = procErrorWhileRunning();
				else RCLCPP_INFO(node_->get_logger(), "Enter Wrong State: %s", enumToString(state_).c_str());
				break;

			case LidarState::ERROR_WHILE_STOPPING:
				if (!bLidarCmd && !bLidarRun && bErrorState) state_ = procErrorWhileStopping();
				else RCLCPP_INFO(node_->get_logger(), "Enter Wrong State: %s", enumToString(state_).c_str());
				break;

			default:
				RCLCPP_WARN(node_->get_logger(), "Unknown State Detected: %s", enumToString(state_).c_str());
				break;
		}
		if (prev_state_ != state_) {
			RCLCPP_INFO(node_->get_logger(), "LiDAR State Changed: %s -> %s", enumToString(prev_state_).c_str(), enumToString(state_).c_str());
		}
	}

    node_lidar.serial_port->write_data(end_lidar, 4);
}

LidarState LidarManager::procWaitingToStart()
{
	LidarState ret = LidarState::WAITING_TO_START;

    RCLCPP_INFO(node_->get_logger(), "LiDAR START");
    node_lidar.lidar_status.lidar_ready = true;
    node_lidar.serial_port->write_data(start_lidar, 4);
    bLidarRun = data_handling(scan_);

    if (!bLidarRun) {
        if (start_cnt_ > 2) {
			RCLCPP_INFO(node_->get_logger(), "[ERROR/Occured] LiDAR START FAIL. Publishing LiDAR Communication Error.");
            bErrorState = true;
            error_msg_.data = true;
            scan_error_pub_->publish(error_msg_);
			ret = LidarState::ERROR_WHILE_RUNNING;
        }
		start_cnt_++;
		RCLCPP_INFO(node_->get_logger(),
			"Waiting to start LiDAR, Lidar Running: %s, retry cnt: %d",
			bLidarRun ? "true" : "false", start_cnt_
		);
    } else {
		RCLCPP_INFO(node_->get_logger(),
			"Success to start LiDAR, Lidar Running: %s, retry cnt: %d",
			bLidarRun ? "true" : "false", start_cnt_
		);
		start_cnt_ = 0;
		ret = LidarState::RUNNING;
	}

    return ret;
}

LidarState LidarManager::procRunning()
{
	LidarState ret = LidarState::RUNNING;

    bLidarRun = data_handling(scan_);

    if (!bLidarRun) {
		RCLCPP_INFO(node_->get_logger(), "[ERROR/Occured] LiDAR communication lost during operation. Publishing LiDAR Communication Error.");
        bErrorState = true;
        error_msg_.data = true;
        scan_error_pub_->publish(error_msg_);
		if (bLidarCmd) {
			ret = LidarState::ERROR_WHILE_RUNNING;
		} else {
			ret = LidarState::ERROR_WHILE_STOPPING;
		}
    } else {
		auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
		float min_angle = 70 * M_PI / 180;
		float max_angle = 290 * M_PI / 180;

		scan_msg->angle_min = min_angle;
		scan_msg->angle_max = max_angle;
		scan_msg->angle_increment = scan_.config.angle_increment;
		scan_msg->scan_time = scan_.config.scan_time;
		scan_msg->time_increment = scan_.config.time_increment;
		scan_msg->range_min = 0.10;
		scan_msg->range_max = scan_.config.max_range;

		std::vector<float> ranges;
		for (size_t i = 0; i < scan_.points.size(); ++i) {
			float angle = scan_.config.min_angle + i * scan_.config.angle_increment;
			if (angle >= min_angle && angle <= max_angle) {
				float r = scan_.points[i].range;
				ranges.push_back(r);
				if (r > 1e-6 && r <= 0.03) { // 3cm 이하 거리의 데이터를 이물질이라고 판정
					dirty_points_ += 1;
				}
			}
		}

		scan_msg->ranges = ranges;
		scan_msg->header.stamp = node_->now();
		scan_msg->header.frame_id = frame_id_;
		scan_pub_->publish(*scan_msg);

		// scan dirty error monitoring
		if (dirty_points_ > 24) { //현재 lidar FOV (220deg) -> 총 244~245개 point 중 10% (25개) 가 오염되면 count
			dirty_cnt_ += 1;
			if (dirty_cnt_ % 20 == 0) {
				RCLCPP_INFO(node_->get_logger(), "[ERROR/Monitor] dirty_cnt: %d", dirty_cnt_);
			}
		} else {
			dirty_cnt_ = 0;
			if (dirty_msg_.data && dirty_reset_cnt_ > 0) {
				dirty_reset_cnt_--;
				if (dirty_reset_cnt_ == 0) {
					dirty_msg_.data = false;
					scan_dirty_error_pub_->publish(dirty_msg_);
					RCLCPP_INFO(node_->get_logger(), "[ERROR/Released] scan_dirty: dirty error cleared after 10 time checked");
				}
			}
		}
		if (dirty_cnt_ > 600) { // 이물감지 "1분(600회 * 100ms)" 동안 지속되면 에러 발생
			RCLCPP_INFO(node_->get_logger(), "[ERROR/Occured] LiDAR dirty error detected. dirty_cnt: %d", dirty_cnt_);
			dirty_msg_.data = true;
			scan_dirty_error_pub_->publish(dirty_msg_);
			dirty_cnt_ = 0;
			dirty_reset_cnt_ = 10;
		}
		dirty_points_ = 0;

		if (!bLidarCmd) {
			ret = LidarState::WAITING_TO_STOP;
		}
	}

	start_cnt_ = 0;

    return ret;
}

LidarState LidarManager::procWaitingToStop()
{
	LidarState ret = LidarState::WAITING_TO_STOP;

    RCLCPP_INFO(node_->get_logger(), "LiDAR STOP");
    node_lidar.lidar_status.lidar_ready = false;
    node_lidar.lidar_status.close_lidar = true;
    node_lidar.serial_port->write_data(end_lidar, 4);
    bLidarRun = data_handling(scan_);
	if (bLidarRun) {
		RCLCPP_INFO(node_->get_logger(), "Wait to LIDAR STOP");
	} else {
		ret = LidarState::IDLE;
	}

	if (bLidarCmd) {
		ret = LidarState::RUNNING;
	}

	dirty_cnt_ = 0;
	dirty_reset_cnt_ = 10;

	return ret;
}

LidarState LidarManager::procIdle()
{
	LidarState ret = LidarState::IDLE;

	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	if (bLidarCmd) {
		ret = LidarState::WAITING_TO_START;
	}

	start_cnt_ = 0;

    return ret;
}

LidarState LidarManager::procErrorWhileRunning()
{
	LidarState ret = LidarState::ERROR_WHILE_RUNNING;

	RCLCPP_INFO(node_->get_logger(), "Retry to Restart LiDAR");
	node_lidar.lidar_status.lidar_ready = true;
	node_lidar.serial_port->write_data(start_lidar, 4);

	bool is_lidar_run = data_handling(scan_); // timeout 3sec

	if (is_lidar_run) {
		if (lidar_run_cnt_ > 10) {
			bLidarRun = true;
			lidar_run_cnt_ = 0;
		}
		lidar_run_cnt_++;
	} else {
		lidar_run_cnt_ = 0;
	}

	if (bLidarRun) {
		RCLCPP_INFO(node_->get_logger(), "[ERROR/Released] Publish LiDAR Communication Error Released");
		bErrorState = false;
		error_msg_.data = false;
		scan_error_pub_->publish(error_msg_);
		ret = LidarState::RUNNING;
		start_cnt_ = 0;
	}

	if (!bLidarCmd) {
		ret = LidarState::ERROR_WHILE_STOPPING;
	}

	start_cnt_ = 0;
	dirty_cnt_ = 0;
	dirty_reset_cnt_ = 10;

	return ret;
}

LidarState LidarManager::procErrorWhileStopping()
{
	LidarState ret = LidarState::ERROR_WHILE_STOPPING;

	RCLCPP_INFO(node_->get_logger(), "Retry to Restart LiDAR");
	node_lidar.lidar_status.lidar_ready = true;
	node_lidar.serial_port->write_data(start_lidar, 4);

	bool is_lidar_run = data_handling(scan_); // timeout 3sec

	if (is_lidar_run) {
		if (lidar_run_cnt_ > 10) {
			bLidarRun = true;
			lidar_run_cnt_ = 0;
		}
		lidar_run_cnt_++;
	} else {
		lidar_run_cnt_ = 0;
	}

	if (bLidarRun) {
		RCLCPP_INFO(node_->get_logger(), "[ERROR/Released] Publish LiDAR Communication Error Released");
		bErrorState = false;
		error_msg_.data = false;
		scan_error_pub_->publish(error_msg_);
		ret = LidarState::WAITING_TO_STOP;
	}

	if (bLidarCmd) {
		ret = LidarState::ERROR_WHILE_RUNNING;
	}

	dirty_cnt_ = 0;
	dirty_reset_cnt_ = 10;

	return ret;
}
