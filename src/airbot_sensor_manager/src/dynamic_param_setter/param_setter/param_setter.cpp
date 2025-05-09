#include "dynamic_param_setter/param_setter/param_setter.hpp"

ParamSetterNode::ParamSetterNode() : Node("param_setter") {
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/airbot_sensor_to_pointcloud");

    while (!parameters_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for parameter server... please check 'sensor_to_pointcloud' node is alive");
    }

    subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/soc_cmd", 10,
        std::bind(&ParamSetterNode::socCmdCallback, this, std::placeholders::_1));

    sub_cell_idx_left_subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/set_mtof_left_sub_cell_idx", 10,
        std::bind(&ParamSetterNode::subCellIdxLeftCallback, this, std::placeholders::_1));

    sub_cell_idx_right_subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/set_mtof_right_sub_cell_idx", 10,
        std::bind(&ParamSetterNode::subCellIdxRightCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[param_setter] Node init finished!");
}

ParamSetterNode::~ParamSetterNode()
{
    parameters_client_.reset();
}

void ParamSetterNode::socCmdCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
    std::string frame_id;

    if (msg->data == 1) { // Auto Mapping
        frame_id = "map";
    } else if (msg->data == 3) { // Navigation
        frame_id = "map";
    } else {
        return;
    }

    if (frame_id.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty frame_id. Ignoring.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Setting parameter 'target_frame' to: %s", frame_id.c_str());

    auto future = parameters_client_->set_parameters(
        {rclcpp::Parameter("target_frame", frame_id)}
    );

    std::thread([future]() mutable {
        try {
            future.get();
            RCLCPP_INFO(rclcpp::get_logger("param_setter"), "Successfully set parameter!");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("param_setter"), "Exception in set_parameters(): %s", e.what());
        }
    }).detach();
}

void ParamSetterNode::subCellIdxLeftCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() != 16) {
        RCLCPP_WARN(this->get_logger(), "Received set_mtof_left_sub_cell_idx with invalid size: %zu", msg->data.size());
        return;
    }

    // 유효하지 않은 값 검사: 값이 0~3이 아닌 경우
    for (size_t i = 0; i < msg->data.size(); ++i) {
        int val = msg->data[i];
        if (val < 0 || val > 3) {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid Left subcell index value at index %zu: %d (valid range: 0~3). Ignoring message.",
                        i, val);
            return;
        }
    }

    rclcpp::Parameter param("tof.multi.left.sub_cell_idx_array", msg->data);
    auto future = parameters_client_->set_parameters({param});

    std::thread([future]() mutable {
        try {
            future.get();
            RCLCPP_INFO(rclcpp::get_logger("param_setter"), "Successfully updated left sub_cell_index_map parameter.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("param_setter"), "Failed to set left sub_cell_index_map: %s", e.what());
        }
    }).detach();
}

void ParamSetterNode::subCellIdxRightCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() != 16) {
        RCLCPP_WARN(this->get_logger(), "Received set_mtof_right_sub_cell_idx with invalid size: %zu", msg->data.size());
        return;
    }

    // 유효하지 않은 값 검사: 값이 0~3이 아닌 경우
    for (size_t i = 0; i < msg->data.size(); ++i) {
        int val = msg->data[i];
        if (val < 0 || val > 3) {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid Right subcell index value at index %zu: %d (valid range: 0~3). Ignoring message.",
                        i, val);
            return;
        }
    }

    rclcpp::Parameter param("tof.multi.right.sub_cell_idx_array", msg->data);
    auto future = parameters_client_->set_parameters({param});

    std::thread([future]() mutable {
        try {
            future.get();
            RCLCPP_INFO(rclcpp::get_logger("param_setter"), "Successfully updated right sub_cell_index_map parameter.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("param_setter"), "Failed to set right sub_cell_index_map: %s", e.what());
        }
    }).detach();
}
