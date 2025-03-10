#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class NodeListPublisher : public rclcpp::Node
{
public:
    NodeListPublisher() : Node("node_list_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("running_node_list", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&NodeListPublisher::publish_node_list, this));
    }

private:
    void publish_node_list()
    {
        auto node_graph = this->get_node_graph_interface();
        std::vector<std::string> node_names = node_graph->get_node_names();
        std_msgs::msg::String msg;
        msg.data = "";
        for (const auto &name : node_names)
        {
            msg.data += "<" + name + ">";
        }
        RCLCPP_INFO(get_logger(), "%s \n", msg.data.c_str());
        publisher_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeListPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

