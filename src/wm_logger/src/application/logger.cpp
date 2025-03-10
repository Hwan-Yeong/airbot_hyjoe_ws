#include "application/logger.hpp"

LoggerService::LoggerService(const rclcpp::Node::SharedPtr &node, const Parameter::SharedPtr &parameter)
    : node_(node)
    , parameter_(parameter)
    , is_warm_up_started_(false)
{
    this->data_ = std::make_shared<Data>();
    this->process_service_ = std::make_shared<ProcessService>();

    this->warm_up_status_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions warm_up_status_subscription_opts;
    warm_up_status_subscription_opts.callback_group = this->warm_up_status_subscription_cb_group_;
    this->warm_up_status_subscription_ = this->node_->create_subscription<std_msgs::msg::Bool>(
        this->parameter_->warm_up_status_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&LoggerService::warm_up_status_subscription_cb, this, _1),
        warm_up_status_subscription_opts);

    this->logging_timer_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->logging_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(this->parameter_->timer_frequency_logging_),
        std::bind(&LoggerService::logging_timer_cb, this),
        this->logging_timer_cb_group_);

    this->cmd_vel_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cmd_vel_subscription_opts;
    cmd_vel_subscription_opts.callback_group = this->cmd_vel_subscription_cb_group_;
    this->cmd_vel_subscription_ = this->node_->create_subscription<geometry_msgs::msg::Twist>(
        this->parameter_->cmd_vel_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&LoggerService::cmd_vel_subscription_cb, this, _1),
        cmd_vel_subscription_opts);

    this->cmd_vel_nav_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cmd_vel_nav_subscription_opts;
    cmd_vel_nav_subscription_opts.callback_group = this->cmd_vel_nav_subscription_cb_group_;
    this->cmd_vel_nav_subscription_ = this->node_->create_subscription<geometry_msgs::msg::Twist>(
        this->parameter_->cmd_vel_nav_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&LoggerService::cmd_vel_nav_subscription_cb, this, _1),
        cmd_vel_nav_subscription_opts);

    rclcpp::CallbackGroup::SharedPtr navigate_to_pose_status_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions navigate_to_pose_status_subscription_opts;
    navigate_to_pose_status_subscription_opts.callback_group = this->navigate_to_pose_status_subscription_cb_group_;
    this->navigate_to_pose_status_subscription_ = this->node_->create_subscription<actionlib_msgs::msg::GoalStatusArray>(
        this->parameter_->navigate_to_pose_status_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&LoggerService::navigate_to_pose_status_subscription_cb, this, _1),
        navigate_to_pose_status_subscription_opts);
}

LoggerService::~LoggerService() = default;

void
LoggerService::logging_timer_cb()
{
    this->log_topic_data();

    RCL_LOG_INFO(this->node_->get_logger(), "\n");
    this->log_node_list();
}

void
LoggerService::log_topic_data()
{
    this->log_big_line();
    this->log_small_line();

    if (this->data_->get__cmd_vel() == nullptr)
    {
        this->log_topic_nullptr(this->parameter_->cmd_vel_topic_);
    }
    else
    {
        const double &linear_x = this->data_->get__cmd_vel()->linear.x;
        const double &angular_z = this->data_->get__cmd_vel()->angular.z;

        RCL_LOG_INFO(this->node_->get_logger(), 
            "[%s] data\n\t\tlinear x : [%f]\n\t\tangular_z : [%f]",
            this->parameter_->cmd_vel_topic_.c_str(),
            linear_x, angular_z);
        
        this->data_->set__cmd_vel(nullptr);
    }

    this->log_small_line();

    if (this->data_->get__cmd_vel_nav() == nullptr)
    {
        this->log_topic_nullptr(this->parameter_->cmd_vel_nav_topic_);
    }
    else
    {
        const double &linear_x = this->data_->get__cmd_vel_nav()->linear.x;
        const double &angular_z = this->data_->get__cmd_vel_nav()->angular.z;

        RCL_LOG_INFO(this->node_->get_logger(),
            "[%s] data\n\t\tlinear x : [%f]\n\t\tangular_z : [%f]",
            this->parameter_->cmd_vel_nav_topic_.c_str(),
            linear_x, angular_z);

        this->data_->set__cmd_vel_nav(nullptr);
    }

    this->log_small_line();

    if (this->data_->get__warm_up_status() == nullptr)
    {
        this->log_topic_nullptr(this->parameter_->warm_up_status_topic_);
    }
    else
    {
        this->is_warm_up_started_ = this->data_->get__warm_up_status()->data;

        if (this->is_warm_up_started_)
        {
            RCL_LOG_INFO(this->node_->get_logger(), "[%s] warm up started", this->parameter_->warm_up_status_topic_.c_str());
        }
    }

    this->log_small_line();

    if (this->data_->get__navigate_to_pose_status() == nullptr)
    {
        this->log_topic_nullptr(this->parameter_->navigate_to_pose_status_topic_);
    }
    else
    {
        const int8_t &status_code = this->data_->get__navigate_to_pose_status()->status_list.back().status;

        if (this->is_warm_up_started_)
        {
            RCL_LOG_INFO(this->node_->get_logger(), "!!!!! N2P WITH WARM UP [%d] !!!!!", status_code);
        }
        else
        {
            RCL_LOG_ERROR(this->node_->get_logger(), "@@@@@ N2P WITHOUT WARM UP [%d] @@@@@", status_code);
        }

        switch (status_code)
        {
            case 2:
            {
                RCL_LOG_INFO(this->node_->get_logger(), "N2P Goal Status, Goal Started");
                break;
            }
            case 3:
            {
                RCL_LOG_INFO(this->node_->get_logger(), "N2P Goal Status, Goal Canceling");
                break;
            }
            case 4:
            {
                RCL_LOG_INFO(this->node_->get_logger(), "!!!!! N2P Goal Status, Goal Succeded !!!!!");
                break;
            }
            case 5:
            {
                RCL_LOG_INFO(this->node_->get_logger(), "N2P Goal Status, Goal Canceled");
                break;
            }
            case 6:
            {
                RCL_LOG_INFO(this->node_->get_logger(), "N2P Goal Status, Goal Aborted");
                break;
            }
        }
    }

    this->log_small_line();
    this->log_big_line();
    RCL_LOG_INFO(this->node_->get_logger(), "\n");
}

void
LoggerService::log_node_list() const
{
    const std::string process_result = this->process_service_->execute_command(this->parameter_->process_command_);
    std::vector<std::string> current_node_process_list;

    if (process_result != "")
    {
        std::vector<std::string> lines = this->process_service_->split(process_result, '\n');

        for (const std::string &line : lines)
        {
            if (line.find("r2ps") != std::string::npos ||
                line.find("_ros2") != std::string::npos ||
                line.find("daemon") != std::string::npos ||
                line.find("echo") != std::string::npos ||
                line.find("grep") != std::string::npos)
            {
                continue;
            }

            std::vector<std::string> words = this->process_service_->split(line, ' ');
            words.erase(std::remove(words.begin(), words.end(), ""), words.end());

            if (words.size() >= 1)
            {
                const int32_t &process_id = std::stoi(words[1]);
                std::string process_name = "";
                process_name = words[10];
                current_node_process_list.push_back(process_name);
            }
        }
    }

    this->data_->set__current_node_list(current_node_process_list);

    this->log_big_line();

    const std::vector<std::string> &wavem_node_list = this->parameter_->wavem_node_list_;
    const std::vector<std::string> &common_node_list = this->parameter_->common_node_list_;
    const std::vector<std::string> &current_node_list = this->data_->get__current_node_list();

    if (current_node_list.size() == 0)
    {
        RCL_LOG_ERROR(this->node_->get_logger(), "NODE LIST IS EMPTY...");
        return;
    }
    else
    {
        RCL_LOG_ERROR(this->node_->get_logger(), "NODE LIST IS SIZE [%d]", current_node_list.size());

        for (const std::string &process : current_node_list)
        {
            RCL_LOG_INFO(this->node_->get_logger(), "process : [%s]", process.c_str());
        }

        this->log_small_line();
    }

    RCL_LOG_INFO(this->node_->get_logger(), "!!!!! WAVEM NODE LIST !!!!!");
    
    for (const std::string &wavem_node : wavem_node_list)
    {
        bool is_found = std::any_of(
            current_node_list.begin(), current_node_list.end(),
            [&](const std::string &process)
            {
                return process.find(wavem_node) != std::string::npos; // 부분 문자열 검사
            });
    
        if (is_found)
        {
            RCL_LOG_INFO(this->node_->get_logger(), "[%s] \033[32mOK\033[0m", wavem_node.c_str());
        }
        else
        {
            RCL_LOG_INFO(this->node_->get_logger(), "[%s] \033[31mNOT OK\033[0m", wavem_node.c_str());
        }
    }

    this->log_small_line();

    RCL_LOG_INFO(this->node_->get_logger(), "!!!!! COMMON NODE LIST !!!!!");
    for (const std::string &common_node : common_node_list)
    {
        bool is_found = std::any_of(
            current_node_list.begin(), current_node_list.end(),
            [&](const std::string &process)
            {
                return process.find(common_node) != std::string::npos; // 부분 문자열 검사
            });
    
        if (is_found)
        {
            RCL_LOG_INFO(this->node_->get_logger(), "[%s] \033[32mOK\033[0m", common_node.c_str());
        }
        else
        {
            RCL_LOG_INFO(this->node_->get_logger(), "[%s] \033[31mNOT OK\033[0m", common_node.c_str());
        }
    }

    // this->data_->set__current_node_list({});
    this->log_big_line();
}

void
LoggerService::warm_up_status_subscription_cb(const std_msgs::msg::Bool::SharedPtr status)
{
    this->data_->set__warm_up_status(status);
}

void
LoggerService::cmd_vel_subscription_cb(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
{
    this->data_->set__cmd_vel(cmd_vel);
}

void
LoggerService::cmd_vel_nav_subscription_cb(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_nav)
{
    this->data_->set__cmd_vel_nav(cmd_vel_nav);
}

void
LoggerService::navigate_to_pose_status_subscription_cb(const actionlib_msgs::msg::GoalStatusArray::SharedPtr navigate_to_pose_status)
{
    this->data_->set__navigate_to_pose_status(navigate_to_pose_status);
}

void
LoggerService::log_big_line() const
{
    RCL_LOG_INFO(this->node_->get_logger(), "%s", LOG_BIG_LINE);
}

void
LoggerService::log_small_line() const
{
    RCL_LOG_INFO(this->node_->get_logger(), "%s", LOG_SMALL_LINE);
}

void
LoggerService::log_topic_nullptr(const std::string &topic) const
{
    RCL_LOG_INFO(this->node_->get_logger(), "[%s] topic's data isn't appeared yet...", topic.c_str());
}