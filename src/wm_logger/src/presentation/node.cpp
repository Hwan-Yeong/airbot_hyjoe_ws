#include "presentation/node.hpp"

LoggerNode::LoggerNode()
    : Node(NODE_NAME)
{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *){});

    if (this->node_ == nullptr)
	{
		return;
	}
	else
	{
		RCL_LOG_INFO(this->node_->get_logger(), "%s created", this->node_->get_name());

        this->parameter_ = std::make_shared<Parameter>();
        this->declare_parameters();

        this->logger_service_ = std::make_shared<LoggerService>(this->node_, this->parameter_);
	}
}

LoggerNode::~LoggerNode() = default;

void
LoggerNode::declare_parameters()
{
    std::vector<std::pair<std::string, std::variant<std::string, std::vector<std::string>, int>>> param_vec =
    {
        {PARAM_WARM_UP_STATUS_TOPIC, ""},
        {PARAM_FRONTIER_STATUS_TOPIC, ""},
        {PARAM_NAVIGATE_TO_POSE_STATUS_TOPIC, ""},
        {PARAM_CMD_VEL_TOPIC, ""},
        {PARAM_CMD_VEL_NAV_TOPIC, ""},
        {PARAM_NODE_WAVEM_LIST, std::vector<std::string>{}},
        {PARAM_NODE_COMMON_LIST, std::vector<std::string>{}},
        {PARAM_TIMER_FREQUENCY_LOGGING, 0}
    };

    for (const std::pair<std::string, std::variant<std::string, std::vector<std::string>, int>> &param : param_vec)
    {
        const std::string &param_name = param.first;
        const auto &default_value = param.second;

        std::visit([this, &param_name](auto &&value)
        {
            using T = std::decay_t<decltype(value)>;
            this->node_->declare_parameter<T>(param_name, value);
        }, default_value);
    }

    this->node_->get_parameter<std::string>(PARAM_WARM_UP_STATUS_TOPIC, this->parameter_->warm_up_status_topic_);
    this->node_->get_parameter<std::string>(PARAM_FRONTIER_STATUS_TOPIC, this->parameter_->frontier_status_topic_);
    this->node_->get_parameter<std::string>(PARAM_NAVIGATE_TO_POSE_STATUS_TOPIC, this->parameter_->navigate_to_pose_status_topic_);
    this->node_->get_parameter<std::string>(PARAM_CMD_VEL_TOPIC, this->parameter_->cmd_vel_topic_);
    this->node_->get_parameter<std::string>(PARAM_CMD_VEL_NAV_TOPIC, this->parameter_->cmd_vel_nav_topic_);
    this->node_->get_parameter<std::vector<std::string>>(PARAM_NODE_WAVEM_LIST, this->parameter_->wavem_node_list_);
    this->node_->get_parameter<std::vector<std::string>>(PARAM_NODE_COMMON_LIST, this->parameter_->common_node_list_);
    this->node_->get_parameter<int>(PARAM_TIMER_FREQUENCY_LOGGING, this->parameter_->timer_frequency_logging_);

    RCL_LOG_INFO(this->node_->get_logger(), "======= PARAMETERS DECLARED =======");
}