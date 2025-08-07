#include "tf_monitor.hpp"

using namespace std::chrono_literals;

TFMonitorNode::TFMonitorNode()
    : Node("tf_monitor_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
{
    monitor_enabled_ = false;
    received_map_to_base = false;
    received_map_to_odom = false;
    received_odom_to_base = false;

    tf_monitor_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/tf_monitor_cmd", 10,
        std::bind(&TFMonitorNode::cmdCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(1s, std::bind(&TFMonitorNode::checkTFs, this));
    RCLCPP_INFO(this->get_logger(), "TF Monitor Node Initialized.");
}

void TFMonitorNode::cmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    monitor_enabled_ = msg->data;
    //clear flags debuging for tf
    received_map_to_base = false;
    received_map_to_odom = false;
    received_odom_to_base = false;
    RCLCPP_INFO(this->get_logger(), "TF Monitor %s", monitor_enabled_ ? "ENABLED" : "DISABLED");
}

void TFMonitorNode::checkTFs()
{
    if (!monitor_enabled_)
        return;
        
    checkTF("map", "odom");
    checkTF("odom", "base_link");
    checkTF("map", "base_link");
}

void TFMonitorNode::checkTF(const std::string &parent, const std::string &child)
{
    rclcpp::Time now = this->get_clock()->now();
    if (tf_buffer_.canTransform(parent, child, now, tf2::durationFromSec(0.5))) {
        try {
            auto transform = tf_buffer_.lookupTransform(parent, child, now);
            const auto &q = transform.transform.rotation;
            tf2::Quaternion quat(q.x, q.y, q.z, q.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            // RCLCPP_INFO(this->get_logger(),
            //             "[TF] %s -> %s: Pos[%.2f, %.2f, %.2f] Yaw[%.2f deg]",
            //             parent.c_str(), child.c_str(),
            //             transform.transform.translation.x,
            //             transform.transform.translation.y,
            //             transform.transform.translation.z,
            //             yaw * 180.0 / M_PI);  // rad → deg
            if(parent == "map" && child == "base_link"){
                if(!received_map_to_base){
                    RCLCPP_INFO(this->get_logger(),
                                "[TF] %s -> %s: Pos[%.2f, %.2f, %.2f] Yaw[%.2f deg]",
                                parent.c_str(), child.c_str(),
                                transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z,
                                yaw * 180.0 / M_PI);  // rad → deg
                }
                received_map_to_base = true;
            }else if(parent == "map" && child == "odom"){
                if(!received_map_to_odom){
                    RCLCPP_INFO(this->get_logger(),
                                "[TF] %s -> %s: Pos[%.2f, %.2f, %.2f] Yaw[%.2f deg]",
                                parent.c_str(), child.c_str(),
                                transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z,
                                yaw * 180.0 / M_PI);  // rad → deg
                }
                received_map_to_odom = true;
            }else if(parent == "odom" && child == "base_link"){
                if(!received_odom_to_base){
                    RCLCPP_INFO(this->get_logger(),
                                "[TF] %s -> %s: Pos[%.2f, %.2f, %.2f] Yaw[%.2f deg]",
                                parent.c_str(), child.c_str(),
                                transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z,
                                yaw * 180.0 / M_PI);  // rad → deg
                }
                received_odom_to_base = true;
            }
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(),
                        "TF Lookup failed: %s -> %s: %s",
                        parent.c_str(), child.c_str(), ex.what());
        }
    } else {
        if(parent == "map" && child == "base_link"){
            if(received_map_to_base){
                RCLCPP_WARN(this->get_logger(),"[TF MISSING] %s -> %s not available.", parent.c_str(), child.c_str());
            }
            received_map_to_base = false;
        }else if(parent == "map" && child == "odom"){
            if(received_map_to_odom){
                RCLCPP_WARN(this->get_logger(),"[TF MISSING] %s -> %s not available.", parent.c_str(), child.c_str());
            }
            received_map_to_odom = false;
        }else if(parent == "odom" && child == "base_link"){
            if(received_odom_to_base){
                RCLCPP_WARN(this->get_logger(),"[TF MISSING] %s -> %s not available.", parent.c_str(), child.c_str());
            }
            received_odom_to_base = false;
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}