#include "WatchdogNode.hpp"

#define ANSI_GREEN "\033[32m"
#define ANSI_RESET "\033[0m"

WatchdogNode::WatchdogNode()
    : Node("watchdog_node")
{

    // 설정 파일 경로를 파라미터로 받을 수 있게 함
    auto config_path = this->declare_parameter<std::string>
        ("config_path", "config.yaml");

    watchdog_ = std::make_unique<WatchdogSystem>(config_path);
    createTopicSubscriptions();

    // 주기적 체크를 위한 타이머 생성
    check_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(watchdog_->getCheckInterval() * 1000)),
        std::bind(&WatchdogNode::checkCallback, this)
    );
}

void WatchdogNode::createTopicSubscriptions()
{
    for (const auto& [node_name, node_config] : watchdog_->getNodeConfigs())
    {
        for (const auto& topic : node_config.topics)
        {
            try
            {
                auto callback = [this, topic_name = topic.name]
                    (std::shared_ptr<rclcpp::SerializedMessage> msg)
                    {
                        this->topicCallback(topic_name);
                    };

                RCLCPP_INFO(this->get_logger(),
                    "Creating subscription for topic %s with message type %s",
                    topic.name.c_str(), topic.msg_type.c_str());

                auto sub = create_generic_subscription(
                    topic.name, topic.msg_type, 10, callback);
                topic_subs_[topic.name] = sub;
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(),
                    "Failed to create subscription for topic %s: %s",
                    topic.name.c_str(), e.what());
            }
        }
    }
}

void WatchdogNode::checkCallback()
{
    for (const auto& [node_name, _] : watchdog_->getNodeConfigs())
    {
        WatchdogSystem::NodeStatus status = watchdog_->checkNode(node_name);

        switch(status)
        {
            case WatchdogSystem::NodeStatusFlag::NONE:
                RCLCPP_ERROR(this->get_logger(),
                    "Node %s is not configured in watchdog system",
                    node_name.c_str());
                break;

            case WatchdogSystem::NodeStatusFlag::NOT_RUNNING:
                RCLCPP_WARN(this->get_logger(),
                    "Node %s is not running, attempting to start",
                    node_name.c_str());
                break;

            case WatchdogSystem::NodeStatusFlag::TOPIC_UNHEALTHY:
                RCLCPP_WARN(this->get_logger(),
                    "Node %s has unhealthy topics, attempting restart",
                    node_name.c_str());
                break;

            case WatchdogSystem::NodeStatusFlag::RESTART_ATTEMPTS_EXCEEDED:
                RCLCPP_ERROR(this->get_logger(),
                    "Node %s has exceeded maximum restart attempts",
                    node_name.c_str());
                //TODO: Restart strategy
                break;

            case WatchdogSystem::NodeStatusFlag::NODE_HEALTHY:
                RCLCPP_DEBUG(this->get_logger(),
                    ANSI_GREEN "Node %s is healthy" ANSI_RESET, node_name.c_str());
                break;

            // case WatchdogSystem::NodeStatusFlag::DEPENDENCY_UNHEALTHY:
            //     RCLCPP_WARN(this->get_logger(),
            //         "Node %s has unhealthy dependencies",
            //         node_name.c_str());
            //     // TODO: Recovery strategy
            //     break;
        }
    }
}

void WatchdogNode::topicCallback(const std::string& topic_name)
{
    watchdog_->updateTopicTimestamp(topic_name);
}
