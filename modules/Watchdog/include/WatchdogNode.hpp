#ifndef WATCHDOG__WATCHDOGNODE_HPP
#define WATCHDOG__WATCHDOGNODE_HPP
#include <memory>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "Watchdog.hpp"

/**
 * @brief Node for the watchdog system
 */
class WatchdogNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Watchdog Node object
     */
    WatchdogNode();

    /**
     * @brief Destroy the Watchdog Node object
     */
    ~WatchdogNode() = default;

private:
    std::unique_ptr<WatchdogSystem> watchdog_; ///< Watchdog system
    rclcpp::TimerBase::SharedPtr check_timer_; ///< Timer for periodic checks
    std::map<std::string, rclcpp::GenericSubscription::SharedPtr> topic_subs_; ///< Map of topic subscriptions

    /**
     * @brief Check the status of the nodes
     */
    void checkCallback();

    /**
     * @brief Create subscriptions for topics
     */
    void createTopicSubscriptions();

    /**
     * @brief Callback for topic subscriptions
     * @param topic_name Name of the topic
     */
    void topicCallback(const std::string& topic_name);
};
#endif // WATCHDOG__WATCHDOGNODE_HPP
