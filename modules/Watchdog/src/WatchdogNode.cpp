#include "WatchdogNode.hpp"

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
    // std::vector<std::string> unhealthy_nodes;

    for (const auto& [node_name, _] : watchdog_->getNodeConfigs())
    {
        if (!watchdog_->checkNode(node_name))
        {
            // unhealthy_nodes.push_back(node_name);
            RCLCPP_WARN(this->get_logger(),
                "Node %s is not healthy", node_name.c_str());
        }
    }

    // if (!unhealthy_nodes.empty())
    // {
    //     for (const auto& node_name: unhealthy_nodes)
    //     {
    //         RCLCPP_WARN(this->get_logger(),
    //             "Restarting node %s", node_name.c_str());
    //         watchdog_->restartNode(node_name);
    //         std::this_thread::sleep_for(std::chrono::seconds(2));
    //     }
    //     unhealthy_nodes.clear();
    // }
}

void WatchdogNode::topicCallback(const std::string& topic_name)
{
    watchdog_->updateTopicTimestamp(topic_name);
}
